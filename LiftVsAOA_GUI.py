import sys
import os
import csv
import re
from statistics import mean
from collections import defaultdict

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
    QHeaderView,
    QDoubleSpinBox,
)

# Matplotlib Qt backend
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
try:
    import numpy as np
    HAS_NUMPY = True
except Exception:
    HAS_NUMPY = False


class LiftVsAOAGUI(QWidget):
    """GUI to select multiple CSV files, assign AOAs, and plot lift vs AOA per speed regime.

    Behavior:
    - Select many CSVs (each corresponds to one AOA value entered by the user).
    - For each CSV, for each row take Wind Speed (MPH) and Lift (lbs).
    - Assign each speed to nearest 5 mph bin if within 2.0 mph of the bin center; otherwise skip that row.
    - For each file and speed bin, compute mean lift (across rows matching that bin in that file).
    - For each speed bin, plot lift vs AOA (one line per bin); x=AOA, y=lift.

    CSV format expected: header row with columns like "Wind Speed (MPH)\tLift (lbs)\tDrag (lbs)" or plain CSV
    with three numeric columns per row: speed, lift, drag.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AOA Plotter")
        self.resize(900, 700)

        main_layout = QVBoxLayout(self)

        # Controls
        ctrl_layout = QHBoxLayout()
        about_btn = QPushButton("About")
        about_btn.clicked.connect(self.show_about)
        ctrl_layout.addWidget(about_btn)
        # allow user to pick which speed bins to show
        select_bins_btn = QPushButton("Select Speed Bins")
        select_bins_btn.clicked.connect(self.show_bin_selector)
        ctrl_layout.addWidget(select_bins_btn)
        add_btn = QPushButton("Add CSV Files")
        add_btn.clicked.connect(self.add_files)
        ctrl_layout.addWidget(add_btn)

        rem_btn = QPushButton("Remove Selected")
        rem_btn.clicked.connect(self.remove_selected)
        ctrl_layout.addWidget(rem_btn)

        plot_lift_btn = QPushButton("Plot Lift vs AOA")
        plot_lift_btn.clicked.connect(lambda: self.plot(metric='lift'))
        ctrl_layout.addWidget(plot_lift_btn)

        plot_drag_btn = QPushButton("Plot Drag vs AOA")
        plot_drag_btn.clicked.connect(lambda: self.plot(metric='drag'))
        ctrl_layout.addWidget(plot_drag_btn)

        clear_btn = QPushButton("Clear Figure")
        clear_btn.clicked.connect(self.clear_figure)
        ctrl_layout.addWidget(clear_btn)

        export_data_btn = QPushButton("Export Sorted, Compiled Data")
        export_data_btn.clicked.connect(self.export_plot_data)
        ctrl_layout.addWidget(export_data_btn)

        save_btn = QPushButton("Save Figure")
        save_btn.clicked.connect(self.save_figure)
        ctrl_layout.addWidget(save_btn)
        

        # checkbox to toggle per-bin dashed fit curves
        self.fit_cb = QtWidgets.QCheckBox("Apply Line of Best Fit")
        self.fit_cb.setChecked(False)
        self.fit_cb.stateChanged.connect(self._on_fit_toggled)
        ctrl_layout.addWidget(self.fit_cb)

        ctrl_layout.addStretch()
        main_layout.addLayout(ctrl_layout)

        # selected bins (None => all)
        self._selected_bins = None
        # show fitted curves per bin (requires numpy)
        self._show_fit_curves = False

        # Table: file path + AOA input
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["CSV File", "AOA (deg)"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        main_layout.addWidget(self.table)

        # Matplotlib canvas
        self.fig = Figure(figsize=(6, 4))
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, 1)

        # status
        self.status_label = QLabel("")
        main_layout.addWidget(self.status_label)

    def add_files(self):
        paths, _ = QFileDialog.getOpenFileNames(self, "Select CSV files", os.getcwd(), "CSV Files (*.csv);;All Files (*)")
        if not paths:
            return
        for p in paths:
            self._add_file_row(p)

    def _add_file_row(self, path):
        r = self.table.rowCount()
        self.table.insertRow(r)
        item = QTableWidgetItem(path)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        self.table.setItem(r, 0, item)

        # try to infer AOA from filename using patterns like _0AOA or _0_AOA or -2AOA
        basename = os.path.basename(path)
        aoa_val = 0.0
        m = re.search(r'([-+]?[0-9]+\.?[0-9]*)\s*_?AOA', basename, re.IGNORECASE)
        if not m:
            # try pattern like _0deg or _0_deg
            m = re.search(r'([-+]?[0-9]+\.?[0-9]*)\s*_?deg', basename, re.IGNORECASE)
        if m:
            try:
                aoa_val = float(m.group(1))
            except Exception:
                aoa_val = 0.0

        spin = QDoubleSpinBox()
        spin.setRange(-180.0, 180.0)
        spin.setDecimals(2)
        spin.setValue(aoa_val)
        spin.setSingleStep(0.25)
        self.table.setCellWidget(r, 1, spin)

    def remove_selected(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()}, reverse=True)
        for r in rows:
            self.table.removeRow(r)

    def _read_csv_file(self, path):
        """Return list of (speed, lift) tuples parsed from the CSV file."""
        rows = []
        try:
            with open(path, newline='') as f:
                # dialect sniff might help but keep it simple
                reader = csv.reader(f)
                # Read first row to check for header
                first = None
                try:
                    first = next(reader)
                except StopIteration:
                    return []
                # try to parse numbers from first row; if fails, assume header and continue
                def try_parse_row(rlist):
                    if not rlist:
                        return None
                    # take first two numeric columns as speed, lift
                    vals = []
                    for c in rlist[:3]:
                        try:
                            # remove stray chars like units or non-breaking spaces
                            s = str(c).strip()
                            s = re.sub(r'[^0-9+\-\.eE]', '', s)
                            vals.append(float(s))
                        except Exception:
                            vals.append(None)
                    # return speed, lift, drag (drag may be None)
                    if vals and vals[0] is not None and vals[1] is not None:
                        dval = vals[2] if len(vals) > 2 else None
                        return (vals[0], vals[1], dval)
                    return None

                parsed = try_parse_row(first)
                if parsed is not None:
                    rows.append(parsed)
                # iterate remaining
                for r in reader:
                    p = try_parse_row(r)
                    if p is not None:
                        rows.append(p)
        except Exception as e:
            # fallback: empty
            print(f"Error reading {path}: {e}")
            return []
        return rows

    def plot(self, metric='lift'):
        n = self.table.rowCount()
        if n == 0:
            QtWidgets.QMessageBox.information(self, "No files", "Please add CSV files first.")
            return

        # Collect per-file AOA and per-file per-bin mean lifts
        # bins: multiples of 5 mph like 0,5,10,15,...
        bin_to_points = defaultdict(list)  # bin -> list of (aoa, mean_lift)
        any_data = False

        for r in range(n):
            item = self.table.item(r, 0)
            if not item:
                continue
            path = item.text()
            spin = self.table.cellWidget(r, 1)
            try:
                aoa = float(spin.value()) if spin is not None else 0.0
            except Exception:
                aoa = 0.0

            rows = self._read_csv_file(path)
            if not rows:
                continue

            # group rows in this file by bin
            per_bin_vals = defaultdict(list)
            for speed, lift, drag in rows:
                try:
                    # ensure numeric
                    sp = float(speed)
                    lf = float(lift)
                    df = float(drag) if drag is not None else None
                except Exception:
                    continue
                # nearest 5
                b = round(sp / 5.0) * 5
                if abs(sp - b) <= 2.0:
                    # choose metric value
                    if metric == 'lift':
                        val = lf
                    else:
                        val = df
                    # skip if metric value missing
                    if val is None:
                        continue
                    per_bin_vals[int(b)].append(val)

            # For each bin present in this file compute mean lift and add a point
            for b, lifts in per_bin_vals.items():
                if lifts:
                    mean_lift = mean(lifts)
                    bin_to_points[b].append((aoa, mean_lift))
                    any_data = True

        if not any_data:
            QtWidgets.QMessageBox.information(self, "No matching data", "No data matched the binning rules (within 2.0 mph of a 5-mph multiple).")
            return

        # Plot: for each bin, build sorted sequences by AOA
        self.fig.clear()
        # reset last-plot storage and remember metric
        self._last_plot_data = {}
        self._last_plot_metric = metric
        ax = self.fig.add_subplot(111)
        colors = None
        # determine which bins to plot based on user selection
        all_bins = sorted(bin_to_points.keys())
        if self._selected_bins is None:
            bins_to_plot = all_bins
        else:
            bins_to_plot = [b for b in all_bins if b in self._selected_bins]
        if not bins_to_plot:
            QtWidgets.QMessageBox.information(self, "No bins selected", "No speed bins selected to plot. Use 'Select Speed Bins' to choose.")
            return

        # sort bins ascending for consistent color ordering
        for b in bins_to_plot:
            pts = bin_to_points[b]
            # sort by aoa
            pts.sort(key=lambda x: x[0])
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            if not xs:
                continue
            # show markers only (no connecting lines) per user request
            p = ax.plot(xs, ys, marker='o', linestyle='None', markersize=6, label=f"{b} mph")
            # color of markers (use same color for fit curve)
            try:
                color = p[0].get_color()
            except Exception:
                color = None
            # save points for possible export: normalize aoa to float
            self._last_plot_data.setdefault(b, []).extend([(float(x), float(y)) for x, y in pts])

            # optional: draw a dashed best-fit curve per bin
            if getattr(self, '_show_fit_curves', False) and HAS_NUMPY:
                try:
                    arr_x = np.array(xs, dtype=float)
                    arr_y = np.array(ys, dtype=float)
                    if arr_x.size >= 3:
                        coeffs = np.polyfit(arr_x, arr_y, 2)
                        xsmooth = np.linspace(arr_x.min(), arr_x.max(), 200)
                        yfit = np.polyval(coeffs, xsmooth)
                    elif arr_x.size == 2:
                        coeffs = np.polyfit(arr_x, arr_y, 1)
                        xsmooth = np.linspace(arr_x.min(), arr_x.max(), 2)
                        yfit = np.polyval(coeffs, xsmooth)
                    else:
                        xsmooth = None
                    if xsmooth is not None:
                        ax.plot(xsmooth, yfit, linestyle='--', color=color, linewidth=1)
                except Exception:
                    # on any fit error, skip drawing the fit for this bin
                    pass
        ax.set_xlabel("Angle of Attack (deg)")
        ax.set_ylabel("Lift (lbs)" if metric == 'lift' else "Drag (lbs)")
        ax.set_title(("Lift" if metric == 'lift' else "Drag") + " vs AOA by Speed Regime")
        ax.grid(True)

        ax.legend(title="Speed bins")
        self.fig.tight_layout()
        self.canvas.draw_idle()
        self.status_label.setText(f"Plotted {metric}: " + ", ".join(str(b) for b in sorted(bin_to_points.keys())))

    def save_figure(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save figure as PNG", os.getcwd(), "PNG Files (*.png);;All Files (*)")
        if not path:
            return
        try:
            self.fig.savefig(path)
            QtWidgets.QMessageBox.information(self, "Saved", f"Figure saved to {path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Save error", str(e))
    
    def _on_ref_lines_toggled(self, state):
        """Handler for reference-lines checkbox. Replot if we have plotted data before."""
        self._show_ref_lines = bool(state)
        # if there is previous plotted data/metric, replot to reflect the change
        if getattr(self, '_last_plot_metric', None) and getattr(self, '_last_plot_data', None):
            try:
                # re-run plot with the same metric
                self.plot(metric=self._last_plot_metric)
            except Exception as e:
                print(f"Error replotting with ref lines: {e}")

    def _on_fit_toggled(self, state):
        """Handler for fit-curves checkbox. Replot if we have plotted data before; warn if numpy missing."""
        if state and not HAS_NUMPY:
            QtWidgets.QMessageBox.information(self, "Missing dependency", "NumPy is required to compute fit curves. Install with: python -m pip install numpy")
            # reset checkbox
            try:
                self.fit_cb.setChecked(False)
            except Exception:
                pass
            return
        self._show_fit_curves = bool(state)
        if getattr(self, '_last_plot_metric', None) and getattr(self, '_last_plot_data', None):
            try:
                self.plot(metric=self._last_plot_metric)
            except Exception as e:
                print(f"Error replotting with fit curves: {e}")

    def show_about(self):
        """Show About dialog with usage notes."""
        msg = (
            "-Only select 1 file per AOA\n"
            "-You can choose which speed regimes to show with the 'Select Speed Bins' button.\n"
            "-Files that contain multiple lift/drag measurements per airspeed will be averaged (mean).\n"
            "-Airspeeds are binned to the nearest 5 mph. Data not within 2.0 mph of a 5-mph multiple will be excluded."
        )
        QtWidgets.QMessageBox.information(self, "About", msg)

    def clear_figure(self):
        """Clear the Matplotlib figure and reset status text."""
        try:
            self.fig.clear()
            self.canvas.draw_idle()
            self.status_label.setText("")
        except Exception as e:
            print(f"Error clearing figure: {e}")

    def export_plot_data(self):
        """Export the last plotted data to a CSV arranged as rows of AOA and one column per speed bin.

        The CSV format:
        AOA (deg), <bin1 mph>, <bin2 mph>, ...
        0.00, 0.002, ,  ...
        """
        try:
            data = getattr(self, '_last_plot_data', None)
            if not data:
                QtWidgets.QMessageBox.information(self, "No data", "No plotted data to export. Plot first.")
                return

            # bins sorted
            bins = sorted(data.keys())
            # collect unique AOAs
            aoa_set = set()
            for b in bins:
                for aoa, val in data[b]:
                    aoa_set.add(float(aoa))
            aoas = sorted(aoa_set)

            # build lookup per bin
            lookup = {b: {float(aoa): val for aoa, val in data[b]} for b in bins}

            path, _ = QFileDialog.getSaveFileName(self, "Save data CSV", os.getcwd(), "CSV Files (*.csv);;All Files (*)")
            if not path:
                return

            # Ensure .csv extension
            if not path.lower().endswith('.csv'):
                path = path + '.csv'

            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['AOA (deg)'] + [f"{b} mph" for b in bins]
                writer.writerow(header)
                for aoa in aoas:
                    row = [f"{aoa:.6f}"]
                    for b in bins:
                        v = lookup[b].get(aoa, '')
                        if v != '':
                            row.append(f"{v:.6f}")
                        else:
                            row.append('')
                    writer.writerow(row)

            QtWidgets.QMessageBox.information(self, "Saved", f"Data exported to {path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Export error", str(e))

    def available_bins(self):
        """Scan the currently-added CSV files and return a sorted list of available bins.

    This uses the same binning rule as plotting (round to nearest 5, within 2.0 mph).
        """
        bins = set()
        n = self.table.rowCount()
        for r in range(n):
            item = self.table.item(r, 0)
            if not item:
                continue
            path = item.text()
            try:
                rows = self._read_csv_file(path)
            except Exception:
                rows = []
            for speed, lift, drag in rows:
                try:
                    sp = float(speed)
                except Exception:
                    continue
                b = round(sp / 5.0) * 5
                if abs(sp - b) <= 1.5:
                    bins.add(int(b))
        return sorted(bins)

    def show_bin_selector(self):
        """Show a dialog with checkboxes for each available bin so user can select which to plot."""
        bins = self.available_bins()
        if not bins:
            QtWidgets.QMessageBox.information(self, "No bins", "No speed bins detected from the added CSV files.")
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Select Speed Bins to Plot")
        dlg_layout = QVBoxLayout(dialog)

        # scroll area for many bins
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QtWidgets.QWidget()
        form = QVBoxLayout(inner)

        checkboxes = {}
        for b in bins:
            cb = QtWidgets.QCheckBox(f"{b} mph")
            # default: checked if no previous selection or if previously selected
            if self._selected_bins is None:
                cb.setChecked(True)
            else:
                cb.setChecked(b in self._selected_bins)
            form.addWidget(cb)
            checkboxes[b] = cb

        # add Select All / Deselect All buttons
        sel_row = QtWidgets.QHBoxLayout()
        sel_all_btn = QPushButton("Select All")
        sel_none_btn = QPushButton("Deselect All")
        sel_row.addWidget(sel_all_btn)
        sel_row.addWidget(sel_none_btn)
        form.addLayout(sel_row)

        # wire actions
        def _select_all():
            for cb in checkboxes.values():
                cb.setChecked(True)

        def _select_none():
            for cb in checkboxes.values():
                cb.setChecked(False)

        sel_all_btn.clicked.connect(_select_all)
        sel_none_btn.clicked.connect(_select_none)

        inner.setLayout(form)
        scroll.setWidget(inner)
        dlg_layout.addWidget(scroll)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        dlg_layout.addWidget(buttons)

        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            sel = {b for b, cb in checkboxes.items() if cb.isChecked()}
            self._selected_bins = sel
            self.status_label.setText("Selected bins: " + ", ".join(str(b) for b in sorted(self._selected_bins)))


def main():
    app = QApplication(sys.argv)
    w = LiftVsAOAGUI()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
