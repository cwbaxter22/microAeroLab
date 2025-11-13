from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    # QTextEdit removed (log console removed)
    QPushButton,
    QLabel,
    QLineEdit,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
    QHeaderView,
    QComboBox,
    QSpinBox,
)

import sys
import os
import threading
import time
import csv

import serial
from serial.tools import list_ports

from SerialFinder import select_serial_port  # uses your existing helper

# Use Matplotlib canvas instead of Bokeh
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtGui import QIcon


def make_app_icon(filename="app_icon.ico"):
    """Resolve icon path relative to the script or PyInstaller _MEIPASS and return a QIcon.

    This handles running from a different CWD and PyInstaller one-file bundles.
    """
    # If running in a PyInstaller bundle, resources are extracted to _MEIPASS
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        base = meipass
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    icon_path = os.path.join(base, filename)
    if os.path.exists(icon_path):
        return QIcon(icon_path)
    return QIcon()

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
        # when polling=True the thread will actively send 'H' and read the response
        self.polling = True
        # seconds between polls (can be tuned)
        self.poll_interval = 0.05

        # set window and application icon (helps show on taskbar)
        try:
            icon = make_app_icon("app_icon.ico")
            try:
                self.setWindowIcon(icon)
            except Exception:
                pass
            try:
                QtWidgets.QApplication.setWindowIcon(icon)
            except Exception:
                pass
        except Exception:
            pass

    def run(self):
        while self._running:
            if self._paused:
                time.sleep(0.05)
                continue
            try:
                # If polling is enabled, actively send 'H' and wait for the response.
                if self.polling:
                    with self.lock:
                        try:
                            # send probe
                            self.ser.write(b"H\n")
                            self.ser.flush()
                        except Exception:
                            # writing may fail if port closed
                            pass
                        # read one line response (respecting serial timeout)
                        raw = self.ser.readline()
                    if raw:
                        try:
                            line = raw.decode(errors="ignore").strip()
                        except Exception:
                            line = str(raw)
                        if line:
                            self.line_received.emit(line)
                    # sleep small interval before next poll
                    time.sleep(self.poll_interval)
                else:
                    # passive read: grab any waiting data
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
        self.setWindowTitle("MicroTunnel DAQ")
        # Increase initial window size so controls have room and text won't be clipped
        # Keep application font at the system default so text size remains unchanged
        self.resize(1600, 900)

        # Serial state
        self.baud = baud
        self.preferred_port = preferred_port
        self.ser = None
        # timestamp of last received valid data (seconds since epoch)
        self._last_data_ts = None
        # whether a serial port object is present/open (separate from data flow)
        self._port_present = False
        # connection tracking
        self._last_connected = False
        self.ser_lock = threading.Lock()
        self.reader = None
        # collection state for Take Data
        self._collecting = False
        self._collect_target = 0
        self._collect_buffer = []
        self._collect_lock = threading.Lock()
        # timer used to abort collection if it takes too long
        self._collect_timer = None
        # real-time airspeed data buffer (timestamp, value)
        self.air_data = []
        # y-axis displacement for airspeed plot (upper/lower = val +/- displacement)
        self.displacement = 5.0

        # Layouts
        main_layout = QVBoxLayout(self)

    # Top: (manual send removed)

        # Middle: controls for Take Data
        control_layout = QHBoxLayout()
        self.samples_label = QLabel("Samples to Average:")
        control_layout.addWidget(self.samples_label)

        self.samples_input = QLineEdit("1")
        self.samples_input.setFixedWidth(160)
        self.samples_input.setValidator(QIntValidator(1, 1000000, self))
        self.samples_input.editingFinished.connect(self.update_hz_estimate)
        control_layout.addWidget(self.samples_input)

        # Frequency display removed per user request

        # Current airspeed display (updated from incoming serial 'H' responses)
        airspeed_layout = QHBoxLayout()
        self.airspeed_label = QLabel("Current Airspeed:")
        self.airspeed_value = QLabel("N/A")
        airspeed_layout.addWidget(self.airspeed_label)
        airspeed_layout.addWidget(self.airspeed_value)
        control_layout.addLayout(airspeed_layout)

        # add a block of space before the Connected label so it sits separated to the right
        control_layout.addSpacing(30)

        # Port controls on the right: connected port label and dropdown of available ports
        # status light, static label, port name, and status text
        # connection label
        self.connection_label = QLabel("Connection:")
        control_layout.addWidget(self.connection_label)

        # port dropdown next to the Connection label
        self.port_combo = QComboBox()
        self.port_combo.setFixedWidth(220)
        self.port_combo.activated.connect(lambda _: self.on_port_selected())
        control_layout.addWidget(self.port_combo)

        # small status light (will be updated to green/red or flashing)
        self.status_light = QLabel()
        self.status_light.setFixedSize(14, 14)
        self.status_light.setStyleSheet("border-radius:7px; background: transparent;")
        control_layout.addWidget(self.status_light)

        # textual status (CONNECTED / OFFLINE)
        self.status_text_label = QLabel("OFFLINE")
        control_layout.addWidget(self.status_text_label)
        # keep a non-visible port name label for internal updates (not shown)
        self.port_name_label = QLabel("None")
        # suppress prompts for a longer window after startup so initial auto-
        # connect logic doesn't prompt the user immediately (move this before
        # refresh_ports so early calls respect suppression). Increase the
        # window from 4s to 10s per user request.
        try:
            self._suppress_initial_prompts = True
            QtCore.QTimer.singleShot(10000, lambda: setattr(self, '_suppress_initial_prompts', False))
        except Exception:
            self._suppress_initial_prompts = False

        # populate available ports
        try:
            ports = self.refresh_ports()
        except Exception:
            ports = []
        # create a port-refresh timer (used while offline to detect newly-plugged devices)
        try:
            self._port_refresh_timer = QtCore.QTimer(self)
            # shorter interval to detect re-plugs faster
            self._port_refresh_timer.setInterval(800)
            self._port_refresh_timer.timeout.connect(self.refresh_ports)
            # always start the port-refresh timer so we continuously scan for ports
            try:
                self._port_refresh_timer.start()
            except Exception:
                pass
        except Exception:
            self._port_refresh_timer = None
        # connection monitor: always check whether serial device is still present
        try:
            self._conn_monitor_timer = QtCore.QTimer(self)
            self._conn_monitor_timer.setInterval(1000)
            self._conn_monitor_timer.timeout.connect(self._check_connection)
            self._conn_monitor_timer.start()
        except Exception:
            self._conn_monitor_timer = None
        # NOTE: suppression is set earlier (before the initial refresh_ports call)
        # so we don't prompt immediately when the app opens.

    # displacement control for airspeed plot y-bounds (moved below the time plot)

        # add a bit of spacing to the left of the Take Data button
        control_layout.addSpacing(12)
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
        # Clear All button above the table
        self.clear_all_btn = QPushButton("Clear All")
        self.clear_all_btn.clicked.connect(self.clear_all)
        left_col.addWidget(self.clear_all_btn)

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

    # Bottom: (log console removed) — incoming lines will be printed to stdout

        content_layout.addLayout(left_col, 3)  # give left column more width

        # Right column: Matplotlib FigureCanvas with two subplots (top: lift/drag, bottom: airspeed vs time)
        self.fig = Figure(figsize=(5, 6))
        self.canvas = FigureCanvas(self.fig)
        # top axes for lift/drag scatter
        self.ax = self.fig.add_subplot(211)
        self.ax.set_xlabel("Wind Speed (MPH)")
        self.ax.set_ylabel("Lift, Drag (lbs)")
        self.ax.grid(True)
        # bottom axes for airspeed time series
        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_xlabel("Seconds Past")
        self.ax2.set_ylabel("Airspeed (MPH)")
        self.ax2.grid(True)
        self.fig.tight_layout()
        content_layout.addWidget(self.canvas, 4)

        main_layout.addLayout(content_layout)

        # Y-bound (displacement) control placed below the time plot
        disp_below_layout = QHBoxLayout()
        # push the label+input to the far right of the window
        disp_below_layout.addStretch()
        self.disp_label = QLabel("y-axis offset")
        disp_below_layout.addWidget(self.disp_label)
        self.displacement_input = QLineEdit(str(int(self.displacement)))
        self.displacement_input.setFixedWidth(160)
        self.displacement_input.setValidator(QIntValidator(0, 10000, self))
        self.displacement_input.editingFinished.connect(self.update_displacement)
        disp_below_layout.addWidget(self.displacement_input)
        main_layout.addLayout(disp_below_layout)

        # connect thread-safe logger signal
        self.log_signal.connect(self.log_message)
        # connect averaged row signal to UI updater
        self.averaged_ready.connect(self._add_averaged_row)

        # Try to open serial port. If none is found, continue in offline mode
        # (keep the GUI open but disable serial-dependent controls).
        # Prefer the combo selection if present, otherwise fall back to select_serial_port
        device = None
        try:
            if self.port_combo.count() > 0:
                device = self.port_combo.currentText()
        except Exception:
            device = None
        if not device:
            device = select_serial_port(preferred_port)
        if not device:
            # Don't close the GUI — run in offline mode so user can still use the UI
            # (export/save and plotting without serial data). Log the state and
            # disable controls that require an active serial connection.
            self.log_message("No serial device selected/found — running in offline mode.")
            self.take_btn.setEnabled(False)
            self.save_btn.setEnabled(False)
            # initialize status and start port scanning
            try:
                self.port_name_label.setText("None")
            except Exception:
                pass
            try:
                self.set_connection_state(False)
            except Exception:
                pass
        else:
            try:
                self.ser = serial.Serial(device, self.baud, timeout=0.1)
            except serial.SerialException as e:
                QtWidgets.QMessageBox.warning(self, "Serial Error", f"Could not open {device}: {e}\nContinuing in offline mode.")
                self.log_message(f"Could not open {device}: {e} — running in offline mode.")
                self.take_btn.setEnabled(False)
                self.save_btn.setEnabled(False)
            else:
                # update UI with connected port
                self.log_message(f"Connected to {self.ser.port} at {self.baud} baud.")
                try:
                    self.port_name_label.setText(self.ser.port)
                except Exception:
                    pass
                try:
                    # reset last-data timestamp for a fresh connection
                    try:
                        self._last_data_ts = None
                    except Exception:
                        pass
                    self.set_connection_state(True)
                except Exception:
                    pass

                # Start reader thread
                self.reader = SerialReaderThread(self.ser, self.ser_lock)
                self.reader.line_received.connect(self.on_line_received)
                # start polling thread so H is continuously queried
                self.reader.polling = True
                # set a modest poll interval; tuning may be necessary for your device
                self.reader.poll_interval = 0.05
                self.reader.start()

        # Estimate hz initially
        QtCore.QTimer.singleShot(200, self.update_hz_estimate)

    def log_message(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        # Print to stdout instead of a GUI log box
        print(f"[{timestamp}] {msg}", flush=True)

    def _update_recv_status(self):
        """Update the CONNECTED/OFFLINE UI based on whether data is being received.

        Rules:
        - If no serial port is present, show OFFLINE.
        - If a serial port is present and we've received valid data within
          a short threshold, show CONNECTED. Otherwise show OFFLINE.
        """
        try:
            now = time.time()
            # threshold: consider data recent if within ~2.5 seconds
            threshold = max(1.0, READ_TIMEOUT * 1.25)
            has_recent = self._last_data_ts is not None and (now - self._last_data_ts) <= threshold

            if not getattr(self, '_port_present', False):
                # no port object -> offline visual state
                self.status_text_label.setText("OFFLINE")
                self.status_text_label.setStyleSheet("font-weight:bold; color: red;")
                # ensure offline flash timer exists and is started
                if not hasattr(self, '_offline_flash_timer'):
                    self._offline_flash_timer = QtCore.QTimer(self)
                    self._offline_flash_timer.setInterval(600)
                    self._offline_flash_timer.timeout.connect(self._toggle_offline_light)
                    self._offline_light_on = False
                try:
                    if not self._offline_flash_timer.isActive():
                        self._offline_flash_timer.start()
                except Exception:
                    pass
                try:
                    self.status_light.setStyleSheet("border-radius:7px; background: transparent;")
                except Exception:
                    pass
            else:
                # port present: reflect data reception
                if has_recent:
                    # show connected
                    try:
                        if hasattr(self, '_offline_flash_timer') and self._offline_flash_timer.isActive():
                            try:
                                self._offline_flash_timer.stop()
                            except Exception:
                                pass
                    except Exception:
                        pass
                    try:
                        self.status_text_label.setText("CONNECTED")
                        self.status_text_label.setStyleSheet("font-weight:bold; color: green;")
                        self.status_light.setStyleSheet("border-radius:7px; background: green;")
                    except Exception:
                        pass
                else:
                    # no recent data -> offline visuals
                    try:
                        self.status_text_label.setText("OFFLINE")
                        self.status_text_label.setStyleSheet("font-weight:bold; color: red;")
                        self.status_light.setStyleSheet("border-radius:7px; background: transparent;")
                    except Exception:
                        pass
        except Exception:
            pass

    def refresh_ports(self):
        """Refresh the list of available serial ports into the combo box."""
        ports = []
        try:
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        # remember selection
        cur = None
        try:
            cur = self.port_combo.currentText()
        except Exception:
            cur = None
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(p)
        # restore selection if possible
        if cur and cur in ports:
            idx = ports.index(cur)
            self.port_combo.setCurrentIndex(idx)
        self.port_combo.blockSignals(False)
        # Always prefer the configured preferred_port if it appears. If preferred
        # is found and we're not already connected to it, switch to it. If no
        # preferred is present and we're not connected, auto-connect to the
        # first available port.
        try:
            try:
                ser_port = getattr(self.ser, 'port', None) if self.ser else None
                is_open = bool(self.ser and getattr(self.ser, 'is_open', False))
            except Exception:
                ser_port = None
                is_open = False
            pref = getattr(self, 'preferred_port', None)
            if pref and pref in ports:
                # connect to preferred if not already connected
                if not (is_open and ser_port == pref):
                    # During initial startup we auto-connect silently to preferred
                    if getattr(self, '_suppress_initial_prompts', False):
                        try:
                            idx = ports.index(pref)
                            self.port_combo.setCurrentIndex(idx)
                            QtCore.QTimer.singleShot(0, self.on_port_selected)
                        except Exception:
                            pass
                        return
                    try:
                        # Prompt the user before switching ports
                        reply = QtWidgets.QMessageBox.question(
                            self,
                            "Switch serial port",
                            f"Preferred port {pref} is available. Switch from {ser_port or 'None'} to {pref}?",
                            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                        )
                        if reply == QtWidgets.QMessageBox.Yes:
                            idx = ports.index(pref)
                            self.port_combo.setCurrentIndex(idx)
                            QtCore.QTimer.singleShot(0, self.on_port_selected)
                        # if No, leave current connection alone
                    except Exception:
                        # fallback: auto-connect to preferred if prompting fails
                        try:
                            idx = ports.index(pref)
                            self.port_combo.setCurrentIndex(idx)
                            QtCore.QTimer.singleShot(0, self.on_port_selected)
                        except Exception:
                            pass
                    return
            # no preferred or preferred not present: if we're currently offline,
            # try to auto-connect to the first available port
            if not (is_open) and ports:
                try:
                    self.port_combo.setCurrentIndex(0)
                    QtCore.QTimer.singleShot(0, self.on_port_selected)
                except Exception:
                    pass
        except Exception:
            pass

    def on_port_selected(self):
        """Handler when the user selects a port from the dropdown.
        Try to open the selected port and start the reader.
        """
        try:
            dev = self.port_combo.currentText()
        except Exception:
            return
        if not dev:
            return
        # if already connected to this port, do nothing
        try:
            if self.ser and getattr(self.ser, 'port', None) == dev and self.ser.is_open:
                return
        except Exception:
            pass
        # close existing reader/serial if present
        try:
            if self.reader:
                self.reader.stop()
                self.reader = None
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        # attempt to open new port
        try:
            self.ser = serial.Serial(dev, self.baud, timeout=0.1)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Serial Error", f"Could not open {dev}: {e}")
            try:
                self.port_name_label.setText("None")
            except Exception:
                pass
            try:
                self.set_connection_state(False)
            except Exception:
                pass
            self.take_btn.setEnabled(False)
            self.save_btn.setEnabled(False)
            return

        # success
        try:
            self.port_name_label.setText(self.ser.port)
        except Exception:
            pass
        try:
            # reset last-data timestamp for a fresh connection; don't consider
            # previous data as valid for this newly-opened port
            try:
                self._last_data_ts = None
            except Exception:
                pass
            self.set_connection_state(True)
        except Exception:
            pass
        self.log_message(f"Connected to {self.ser.port} at {self.baud} baud.")
        self.take_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        # start reader
        self.reader = SerialReaderThread(self.ser, self.ser_lock)
        self.reader.line_received.connect(self.on_line_received)
        self.reader.polling = True
        self.reader.poll_interval = 0.05
        self.reader.start()

    def set_connection_state(self, connected: bool):
        """Update status text and light. If connected=True show green light and
        'CONNECTED'. If False, show 'OFFLINE' and flash a red light.
        """
        try:
            # remember whether a serial port object is present; actual CONNECTED
            # vs OFFLINE visuals are driven by recent data reception in
            # _update_recv_status()
            self._port_present = bool(connected)
            # If we are not connected, clear the last-data timestamp so a
            # stale timestamp from a previous session doesn't make the UI
            # show CONNECTED.
            if not connected:
                try:
                    self._last_data_ts = None
                except Exception:
                    pass
            try:
                self._update_recv_status()
            except Exception:
                pass
        except Exception:
            pass

    def _toggle_offline_light(self):
        """Toggle the offline status light on/off to create a flashing effect."""
        try:
            if getattr(self, '_offline_light_on', False):
                self.status_light.setStyleSheet("border-radius:7px; background: transparent;")
            else:
                self.status_light.setStyleSheet("border-radius:7px; background: red;")
            self._offline_light_on = not getattr(self, '_offline_light_on', False)
        except Exception:
            pass

    def _check_connection(self):
        """Periodic check to detect if the serial device was unplugged.
        If we were connected and the port is no longer open, mark offline and
        start scanning for ports.
        """
        try:
            ports = []
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []

            ser_port = None
            try:
                ser_port = getattr(self.ser, 'port', None) if self.ser else None
            except Exception:
                ser_port = None

            is_open = bool(self.ser and getattr(self.ser, 'is_open', False))

            # If the serial object's port no longer appears in the system list,
            # treat it as unplugged and transition to OFFLINE.
            if ser_port and ser_port not in ports:
                # handle disconnect
                try:
                    if self.reader:
                        self.reader.stop()
                        self.reader = None
                except Exception:
                    pass
                try:
                    if self.ser and getattr(self.ser, 'is_open', False):
                        self.ser.close()
                except Exception:
                    pass
                try:
                    self.ser = None
                except Exception:
                    pass
                # clear last-data timestamp so stale data doesn't show CONNECTED
                try:
                    self._last_data_ts = None
                except Exception:
                    pass
                try:
                    self.port_name_label.setText("None")
                except Exception:
                    pass
                try:
                    self.set_connection_state(False)
                except Exception:
                    pass
                # ensure port refresh is running so newly-plugged devices are discovered
                try:
                    if hasattr(self, '_port_refresh_timer') and self._port_refresh_timer is not None and not self._port_refresh_timer.isActive():
                        self._port_refresh_timer.start()
                except Exception:
                    pass
                self._last_connected = False
            else:
                # update last connected flag if we detect an open port
                if is_open and not getattr(self, '_last_connected', False):
                    self._last_connected = True
        except Exception:
            pass

    def on_line_received(self, line: str):
        # Show raw incoming lines in log
        self.log_message(f"Received: {line}")
        # Try to parse first value as current airspeed and update display
        parts = []
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
                # append to real-time airspeed buffer and update airspeed plot
                try:
                    ts = time.time()
                    self.air_data.append((ts, v))
                    # record time of last successful data reception
                    try:
                        self._last_data_ts = ts
                    except Exception:
                        pass
                    # prune to a small buffer slightly longer than 5s to avoid growth
                    cutoff = ts - 6.0
                    self.air_data = [(t, val) for (t, val) in self.air_data if t >= cutoff]
                    # update airspeed time-series plot
                    try:
                        self.update_airspeed_plot()
                    except Exception:
                        pass
                except Exception:
                    pass
                # update status visuals to reflect that data was just received
                try:
                    self._update_recv_status()
                except Exception:
                    pass
        except Exception:
            pass

        # If we are currently collecting samples for Take Data, append this sample
        try:
            if self._collecting:
                # expect at least three comma-separated numeric values
                if parts and len(parts) >= 3:
                    # input order is: airspeed, drag, lift
                    # we want to store as: airspeed, lift, drag
                    try:
                        a = float(parts[0])
                    except Exception:
                        a = 0.0
                    try:
                        d = float(parts[1])
                    except Exception:
                        d = 0.0
                    try:
                        l = float(parts[2])
                    except Exception:
                        l = 0.0
                    vals = [a, l, d]
                    with self._collect_lock:
                        self._collect_buffer.append(vals)
                        # if we've reached the requested number of samples, finalize
                        if len(self._collect_buffer) >= self._collect_target:
                            # stop collection timer if running
                            try:
                                if self._collect_timer and self._collect_timer.isActive():
                                    self._collect_timer.stop()
                            except Exception:
                                pass
                            collected = list(self._collect_buffer)
                            self._collecting = False
                            # compute element-wise average
                            sums = [0.0, 0.0, 0.0]
                            for row in collected:
                                for j in range(3):
                                    sums[j] += row[j]
                            cnt = len(collected)
                            averaged = [s / cnt for s in sums]
                            # deliver averaged row to GUI thread via signal
                            self.averaged_ready.emit(averaged)
        except Exception:
            # non-critical - ignore
            pass

    def send_manual(self):
        # manual send removed
        return

    def estimate_roundtrip(self, trials=1):
        """Send 'H' a number of trials to estimate single-roundtrip time (seconds).
        Returns average time per H, or None on failure.
        """
        if not self.ser or not self.ser.is_open:
            return None
        # Pause background reader while probing so it doesn't consume responses
        resume_reader = False
        if self.reader and not getattr(self.reader, '_paused', False):
            # schedule pause on reader thread-safe flag
            QtCore.QTimer.singleShot(0, self.reader.pause)
            resume_reader = True
            # give the reader a moment to pause
            time.sleep(0.05)
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
            result = sum(timings) / len(timings)
        else:
            result = None
        # resume reader if we paused it
        if resume_reader and self.reader:
            QtCore.QTimer.singleShot(0, self.reader.resume)
        return result

    def estimate_total_time(self, trials=1):
        """Send 'H' trials times and return the total elapsed seconds for all trials.
        Returns total_time (float) on success, or None on any failure/timeout.
        This is useful to measure the actual time to perform N samples (including
        per-call overhead), so Hz for N samples = 1.0 / total_time.
        """
        if not self.ser or not self.ser.is_open:
            return None
        # Pause background reader while probing so it doesn't consume responses
        resume_reader = False
        try:
            if self.reader and not getattr(self.reader, '_paused', False):
                QtCore.QTimer.singleShot(0, self.reader.pause)
                resume_reader = True
                time.sleep(0.05)

            total = 0.0
            for _ in range(trials):
                try:
                    with self.ser_lock:
                        start = time.perf_counter()
                        self.ser.write(b"H\n")
                        self.ser.flush()
                        raw = self.ser.readline()
                        end = time.perf_counter()
                    if raw:
                        total += (end - start)
                    else:
                        # timeout or empty - treat as failure
                        total = None
                        break
                except Exception:
                    total = None
                    break
            return total
        finally:
            if resume_reader and self.reader:
                QtCore.QTimer.singleShot(0, self.reader.resume)

    def update_hz_estimate(self):
        txt = self.samples_input.text().strip()
        try:
            n = int(txt)
            if n <= 0:
                raise ValueError
        except Exception:
            # no frequency display - nothing to update
            return
        # frequency display removed; keep input validation but do not show value
        return

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
        self.save_btn.setEnabled(False)
        # Start collecting the next n incoming samples that the background poller emits
        with self._collect_lock:
            self._collect_buffer.clear()
            self._collect_target = n
            self._collecting = True

        # set up a timeout to abort collection if it takes too long
        timeout_ms = max(5000, int(n * READ_TIMEOUT * 1000) + 500)
        # stop any existing timer
        try:
            if self._collect_timer and self._collect_timer.isActive():
                self._collect_timer.stop()
        except Exception:
            pass

        self._collect_timer = QtCore.QTimer(self)
        self._collect_timer.setSingleShot(True)

        def on_collect_timeout():
            with self._collect_lock:
                collected = list(self._collect_buffer)
                self._collecting = False
            if not collected:
                self.log_message("[take_data] No valid samples collected (timeout).")
                self.enable_controls()
                return
            # average element-wise and emit
            sums = [0.0, 0.0, 0.0]
            for row in collected:
                for j in range(3):
                    sums[j] += row[j]
            cnt = len(collected)
            averaged = [s / cnt for s in sums]
            self.averaged_ready.emit(averaged)

        self._collect_timer.timeout.connect(on_collect_timeout)
        self._collect_timer.start(timeout_ms)

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
        btn.setFixedWidth(48)
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

    def clear_all(self):
        """Clear all rows from the results table and update the plot."""
        try:
            self.table.setRowCount(0)
        except Exception:
            # fallback removal
            try:
                while self.table.rowCount() > 0:
                    self.table.removeRow(0)
            except Exception:
                pass
        # refresh plot
        try:
            self.update_plot()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def enable_controls(self):
        self.take_btn.setEnabled(True)
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


    def update_airspeed_plot(self):
        """Update the airspeed vs time line plot to show the last 5 seconds.
        The x-axis shows seconds in range [0, 5] where 0 is now-5 and 5 is now.
        The y-axis is centered on the latest airspeed and extends +/- displacement.
        """
        now = time.time()
        # keep data within a small rolling window (slightly longer than 5s)
        cutoff = now - 6.0
        self.air_data = [(t, v) for (t, v) in self.air_data if t >= cutoff]

        if not self.air_data:
            self.ax2.clear()
            self.ax2.set_xlabel("Seconds Past")
            self.ax2.set_ylabel("Airspeed (MPH)")
            self.ax2.grid(True)
            self.canvas.draw_idle()
            return

        # convert to 'seconds past' where 0 is now and larger values are in the past
        xs = [now - t for (t, v) in self.air_data]
        ys = [v for (t, v) in self.air_data]

        # draw
        self.ax2.clear()
        self.ax2.plot(xs, ys, linestyle='-', marker=None, color='tab:green')
        # display last 5 seconds with 0 at the right and 5 at the left
        self.ax2.set_xlim(5.0, 0.0)
        # center y-limits on latest value
        latest = ys[-1]
        d = float(self.displacement) if self.displacement is not None else 5.0
        self.ax2.set_ylim(latest - d, latest + d)
        self.ax2.set_xlabel("Seconds Past")
        self.ax2.set_ylabel("Airspeed (MPH)")
        self.ax2.grid(True)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def update_displacement(self):
        txt = self.displacement_input.text().strip()
        try:
            v = float(txt)
            if v < 0:
                raise ValueError
            self.displacement = v
        except Exception:
            # restore previous value
            try:
                self.displacement_input.setText(str(int(self.displacement)))
            except Exception:
                self.displacement_input.setText("5")
        # refresh plot to apply new displacement
        try:
            self.update_airspeed_plot()
        except Exception:
            pass

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
    # Increase application font size to improve readability (scale ~1.6x)
    try:
        f = app.font()
        ps = f.pointSize()
        if ps <= 0:
            ps = 10
        f.setPointSize(max(1, int(ps * 1.6)))
        app.setFont(f)
    except Exception:
        pass
    w = SerialGUI(preferred_port="COM5", baud=DEFAULT_BAUD)
    w.show()
    # If PyInstaller bootloader splash was used, close it now so it doesn't persist.
    try:
        import pyi_splash
        try:
            pyi_splash.close()
        except Exception:
            pass
    except Exception:
        pass
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()