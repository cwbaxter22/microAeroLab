from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTextEdit,
    QPushButton,
    QLabel,
    QLineEdit,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
    QHeaderView,
    QSpinBox,
)

import sys
import threading
import time
import csv

import serial

from SerialFinder import select_serial_port  # uses your existing helper

# Use Matplotlib canvas instead of Bokeh
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

DEFAULT_BAUD = 9600
READ_TIMEOUT = 2.0


class SerialReaderThread(QThread):
    line_received = pyqtSignal(str)

    def __init__(self, ser, lock, parent=None):
        super().__init__(parent)
        self.ser = ser
        self.lock = lock
        self._running = True
        self._paused = False

    def run(self):
        while self._running:
            if self._paused:
                time.sleep(0.05)
                continue
            try:
                # Acquire lock so Take Data doesn't race with log reader
                with self.lock:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                    else:
                        raw = None
                if raw:
                    try:
                        line = raw.decode(errors="ignore").strip()
                    except Exception:
                        line = str(raw)
                    if line:
                        self.line_received.emit(line)
                else:
                    time.sleep(0.05)
            except Exception as e:
                self.line_received.emit(f"[reader error] {e}")
                time.sleep(0.5)

    def stop(self):
        self._running = False
        self.wait(500)

    def pause(self):
        """Pause background reading (thread-safe enough for this use)."""
        self._paused = True

    def resume(self):
        """Resume background reading."""
        self._paused = False


class SerialGUI(QWidget):
    # use a Qt signal for thread-safe logging from worker threads
    log_signal = QtCore.pyqtSignal(str)
    # signal to deliver averaged row (use object to pass Python list safely)
    averaged_ready = QtCore.pyqtSignal(object)

    def __init__(self, preferred_port="COM5", baud=DEFAULT_BAUD):
        super().__init__()
        print("[debug] SerialGUI.__init__: starting", flush=True)
        self.setWindowTitle("Serial COM GUI — PyQt")
        self.resize(1100, 650)

        # Serial state
        self.baud = baud
        self.ser = None
        self.ser_lock = threading.Lock()
        self.reader = None

        # Layouts
        main_layout = QVBoxLayout(self)

        # Top: manual send (2-line)
        manual_layout = QHBoxLayout()
        self.manual_text = QTextEdit()
        self.manual_text.setFixedHeight(60)
        manual_layout.addWidget(self.manual_text)

        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_manual)
        manual_layout.addWidget(self.send_btn)
        main_layout.addLayout(manual_layout)

        # Middle: controls for Take Data
        control_layout = QHBoxLayout()
        self.samples_label = QLabel("Samples to Average:")
        control_layout.addWidget(self.samples_label)

        self.samples_input = QLineEdit("1")
        self.samples_input.setFixedWidth(80)
        self.samples_input.setValidator(QIntValidator(1, 1000000, self))
        self.samples_input.editingFinished.connect(self.update_hz_estimate)
        control_layout.addWidget(self.samples_input)

        hz_layout = QHBoxLayout()
        self.hz_label = QLabel("Hz:")
        self.hz_value = QLabel("N/A")
        hz_layout.addWidget(self.hz_label)
        hz_layout.addWidget(self.hz_value)
        control_layout.addLayout(hz_layout)

        # Current airspeed display (updated from incoming serial 'H' responses)
        airspeed_layout = QHBoxLayout()
        self.airspeed_label = QLabel("Current Airspeed:")
        self.airspeed_value = QLabel("N/A")
        airspeed_layout.addWidget(self.airspeed_label)
        airspeed_layout.addWidget(self.airspeed_value)
        control_layout.addLayout(airspeed_layout)

        self.take_btn = QPushButton("Take Data")
        self.take_btn.clicked.connect(self.take_data)
        control_layout.addWidget(self.take_btn)

        self.save_btn = QPushButton("Export CSV")
        self.save_btn.clicked.connect(self.export_csv)
        control_layout.addWidget(self.save_btn)

        control_layout.addStretch()
        main_layout.addLayout(control_layout)

        # Content area: left = table + log, right = plot (Matplotlib canvas)
        content_layout = QHBoxLayout()

        # Left column: table + log stacked
        left_col = QVBoxLayout()

    # Table: results with horizontal scrollbar (+ remove button column)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Wind Speed (MPH)", "Lift (lbs)", "Drag (lbs)", ""])
        # Stretch numeric columns, make remove column sized to contents
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.setMinimumHeight(150)
        self.table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        left_col.addWidget(self.table)

        # Bottom: log area for incoming data
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        left_col.addWidget(self.log)

        content_layout.addLayout(left_col, 3)  # give left column more width

        # Right column: Matplotlib FigureCanvas
        self.fig = Figure(figsize=(5, 4))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Wind Speed (MPH)")
        self.ax.set_ylabel("Lift / Drag (lbs)")
        self.ax.grid(True)
        content_layout.addWidget(self.canvas, 4)

        main_layout.addLayout(content_layout)

        # connect thread-safe logger signal
        self.log_signal.connect(self.log_message)
        # connect averaged row signal to UI updater
        self.averaged_ready.connect(self._add_averaged_row)

        # Try to open serial port. If none is found, continue in offline mode
        # (keep the GUI open but disable serial-dependent controls).
        device = select_serial_port(preferred_port)
        if not device:
            # Don't close the GUI — run in offline mode so user can still use the UI
            # (export/save and plotting without serial data). Log the state and
            # disable controls that require an active serial connection.
            self.log_message("No serial device selected/found — running in offline mode.")
            self.take_btn.setEnabled(False)
            self.send_btn.setEnabled(False)
            self.save_btn.setEnabled(False)
        else:
            try:
                self.ser = serial.Serial(device, self.baud, timeout=0.1)
            except serial.SerialException as e:
                QtWidgets.QMessageBox.warning(self, "Serial Error", f"Could not open {device}: {e}\nContinuing in offline mode.")
                self.log_message(f"Could not open {device}: {e} — running in offline mode.")
                self.take_btn.setEnabled(False)
                self.send_btn.setEnabled(False)
                self.save_btn.setEnabled(False)
            else:
                self.log_message(f"Connected to {self.ser.port} at {self.baud} baud.")

                # Start reader thread
                self.reader = SerialReaderThread(self.ser, self.ser_lock)
                self.reader.line_received.connect(self.on_line_received)
                self.reader.start()

        # Estimate hz initially
        QtCore.QTimer.singleShot(200, self.update_hz_estimate)
        print("[debug] SerialGUI.__init__: finished setup", flush=True)

    def log_message(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log.append(f"[{timestamp}] {msg}")

    def on_line_received(self, line: str):
        # Show raw incoming lines in log
        self.log_message(f"Received: {line}")
        # Try to parse first value as current airspeed and update display
        try:
            parts = [p.strip() for p in line.split(',')]
            if parts and parts[0]:
                v = float(parts[0])
                # update label in GUI thread (this slot should be in GUI thread already)
                try:
                    self.airspeed_value.setText(f"{v:.3f}")
                except Exception:
                    # if label not present yet, ignore
                    pass
        except Exception:
            pass

    def send_manual(self):
        text = self.manual_text.toPlainText().strip()
        if not text:
            return
        if not self.ser or not self.ser.is_open:
            QtWidgets.QMessageBox.warning(self, "Serial", "Serial port not open.")
            return
        try:
            with self.ser_lock:
                self.ser.write((text + "\n").encode())
            self.log_message(f"Sent: {text}")
            self.manual_text.clear()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Send Error", str(e))

    def estimate_roundtrip(self, trials=1):
        """Send 'H' a number of trials to estimate single-roundtrip time (seconds).
        Returns average time per H, or None on failure.
        """
        if not self.ser or not self.ser.is_open:
            return None
        timings = []
        for _ in range(trials):
            try:
                with self.ser_lock:
                    start = time.perf_counter()
                    self.ser.write(b"H\n")
                    self.ser.flush()
                    # read one line response (blocking with serial timeout)
                    raw = self.ser.readline()
                    end = time.perf_counter()
                if raw:
                    timings.append(end - start)
                else:
                    # timeout - treat as failure
                    return None
            except Exception:
                return None
        if timings:
            return sum(timings) / len(timings)
        return None

    def update_hz_estimate(self):
        txt = self.samples_input.text().strip()
        try:
            n = int(txt)
            if n <= 0:
                raise ValueError
        except Exception:
            self.hz_value.setText("N/A")
            return

        # Start a small worker to probe without blocking UI
        def worker():
            # try a couple probes to get a stable estimate
            t = self.estimate_roundtrip(trials=1)
            if t is None or t <= 0:
                QtCore.QTimer.singleShot(0, lambda: self.hz_value.setText("N/A"))
                return
            total_time = t * n
            hz = 1.0 / total_time if total_time > 0 else 0.0
            # update GUI in main thread
            QtCore.QTimer.singleShot(0, lambda: self.hz_value.setText(f"{hz:.3f}"))

        th = threading.Thread(target=worker, daemon=True)
        th.start()

    def take_data(self):
        txt = self.samples_input.text().strip()
        try:
            n = int(txt)
            if n <= 0:
                raise ValueError
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Input", "Enter a valid integer > 0 for samples.")
            return

        if not self.ser or not self.ser.is_open:
            QtWidgets.QMessageBox.warning(self, "Serial", "Serial port not open.")
            return

        self.take_btn.setEnabled(False)
        self.send_btn.setEnabled(False)
        self.save_btn.setEnabled(False)

        # Pause background reader so this worker reliably receives responses
        if self.reader:
            self.reader.pause()

        def worker_take():
            collected = []  # list of lists of floats per sample
            for i in range(n):
                try:
                    with self.ser_lock:
                        # temporarily increase serial timeout while waiting for response
                        prev_timeout = getattr(self.ser, "timeout", None)
                        self.ser.timeout = READ_TIMEOUT
                        try:
                            self.ser.write(b"H\n")
                            self.ser.flush()
                            raw = self.ser.readline()
                        finally:
                            # restore previous timeout
                            self.ser.timeout = prev_timeout
                    if not raw:
                        # timeout or empty - log and continue
                        self.log_signal.emit(f"[take_data] No response for sample {i+1}")
                        continue
                    line = raw.decode(errors="ignore").strip()
                    self.log_signal.emit(f"[take_data] Sample {i+1} raw: {line}")
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) < 3:
                        self.log_signal.emit(f"[take_data] Unexpected format for sample {i+1}: {line}")
                        continue
                    vals = []
                    for p in parts[:3]:
                        try:
                            vals.append(float(p))
                        except Exception:
                            vals.append(0.0)
                    collected.append(vals)
                            # update current airspeed display from this sample
                    try:
                                QtCore.QTimer.singleShot(0, lambda v=vals[0]: self.airspeed_value.setText(f"{v:.3f}"))
                    except Exception:
                                pass
                except Exception as e:
                    self.log_signal.emit(f"[take_data] error: {e}")
                    continue

            if not collected:
                self.log_signal.emit("[take_data] No valid samples collected.")
                # re-enable controls on main thread
                QtCore.QTimer.singleShot(0, self.enable_controls)
                # also resume reader
                if self.reader:
                    QtCore.QTimer.singleShot(0, self.reader.resume)
                return

            # average element-wise
            sums = [0.0, 0.0, 0.0]
            for row in collected:
                for j in range(3):
                    sums[j] += row[j]
            cnt = len(collected)
            averaged = [s / cnt for s in sums]

            # deliver averaged row to GUI thread via signal (thread-safe)
            self.averaged_ready.emit(averaged)
            # schedule reader resume on GUI thread
            if self.reader:
                QtCore.QTimer.singleShot(0, self.reader.resume)

        # ensure reader is resumed if worker exits unexpectedly
        def on_worker_done_resume():
            if self.reader:
                self.reader.resume()
            QtCore.QTimer.singleShot(0, self.enable_controls)

        t = threading.Thread(target=worker_take, daemon=True)
        t.start()
        # ensure resume eventually if something unexpected happens
        QtCore.QTimer.singleShot(max(5000, int(n * READ_TIMEOUT * 1000) + 500), on_worker_done_resume)

    def _add_averaged_row(self, averaged):
        """Run in main thread to add averaged row and re-enable controls."""
        r = self.table.rowCount()
        self.table.insertRow(r)
        for j, val in enumerate(averaged):
            item = QTableWidgetItem(f"{val:.3f}")
            # clear editable flag
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(r, j, item)
        # add remove button in last column
        btn = QPushButton("x")
        btn.setFixedWidth(24)
        btn.setToolTip("Remove row")
        btn.clicked.connect(self._remove_row_button_clicked)
        self.table.setCellWidget(r, 3, btn)
        self.log_message(f"[take_data] Averaged result: {averaged}")
        self.enable_controls()
        # update plot when table changes
        self.update_plot()

    def _remove_row_button_clicked(self):
        """Remove the row containing the clicked 'x' button and update the plot."""
        sender = self.sender()
        if sender is None:
            return
        try:
            pt = sender.mapTo(self.table, QtCore.QPoint(0, 0))
            idx = self.table.indexAt(pt)
            row = idx.row()
        except Exception:
            row = -1
        if row >= 0:
            self.table.removeRow(row)
            self.update_plot()

    @QtCore.pyqtSlot()
    def enable_controls(self):
        self.take_btn.setEnabled(True)
        self.send_btn.setEnabled(True)
        self.save_btn.setEnabled(True)

    def update_plot(self):
        # Read data from the table
        rows = []
        for r in range(self.table.rowCount()):
            try:
                w_item = self.table.item(r, 0)
                l_item = self.table.item(r, 1)
                d_item = self.table.item(r, 2)
                if not (w_item and l_item and d_item):
                    continue
                w = float(w_item.text())
                l = float(l_item.text())
                d = float(d_item.text())
                rows.append((w, l, d))
            except Exception:
                continue

        if not rows:
            # clear axes
            self.ax.clear()
            self.ax.set_xlabel("Wind Speed (MPH)")
            self.ax.set_ylabel("Lift / Drag (lbs)")
            self.ax.grid(True)
            self.canvas.draw_idle()
            return

        # sort by wind speed
        rows.sort(key=lambda x: x[0])
        xs = [r[0] for r in rows]
        lifts = [r[1] for r in rows]
        drags = [r[2] for r in rows]

        # plot with matplotlib: markers only (no connecting lines)
        self.ax.clear()
        self.ax.plot(xs, lifts, marker="o", linestyle="None", color="tab:blue", label="Lift (lbs)")
        self.ax.plot(xs, drags, marker="s", linestyle="None", color="tab:orange", label="Drag (lbs)")
        self.ax.set_xlabel("Wind Speed (MPH)")
        self.ax.set_ylabel("Lift / Drag (lbs)")
        self.ax.legend()
        self.ax.grid(True)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def export_csv(self):
        if self.table.rowCount() == 0:
            QtWidgets.QMessageBox.information(self, "Export", "No data to export.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv);;All Files (*)")
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                header = ["Wind Speed (MPH)", "Lift (lbs)", "Drag (lbs)"]
                writer.writerow(header)
                for r in range(self.table.rowCount()):
                    row = []
                    # only export the first three numeric columns
                    for c in range(3):
                        item = self.table.item(r, c)
                        row.append(item.text() if item else "")
                    writer.writerow(row)
            QtWidgets.QMessageBox.information(self, "Export", f"Saved CSV to {path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Export Error", str(e))

    def closeEvent(self, event):
        # clean up threads and serial
        try:
            if self.reader:
                self.reader.stop()
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = SerialGUI(preferred_port="COM5", baud=DEFAULT_BAUD)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()