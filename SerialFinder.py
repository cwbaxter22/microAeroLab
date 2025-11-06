import serial
import time
from serial.tools import list_ports

def select_serial_port(preferred=None):
    """
    Detect available COM ports on Windows.
    Optionally pick a preferred one (e.g., COM5).
    """
    ports = list_ports.comports()
    if not ports:
        print("❌ No serial ports detected.")
        return None

    if preferred:
        for p in ports:
            if preferred.lower() in p.device.lower():
                print(f"✅ Using preferred port: {p.device} ({p.description})")
                return p.device
        print(f"⚠️ Preferred port {preferred} not found. Using first available instead.")

    print("\nAvailable COM ports:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")

    if len(ports) == 1:
        chosen = ports[0].device
        print(f"\n✅ Auto-selected: {chosen}")
    else:
        # Avoid blocking console input (which can hang when running under
        # VS Code's Run/Debug without an interactive stdin). Prefer the first
        # available port when multiple are present. If you want an interactive
        # selection, call this helper from a terminal or implement a GUI picker.
        chosen = ports[0].device
        print(f"\n⚠️ Multiple ports found — auto-selecting: {chosen}")

    return chosen


def read_serial_data(port=None, baudrate=9600):
    """
    Continuously read and print data from the serial port.
    """
    device = select_serial_port(port)
    if not device:
        return

    try:
        ser = serial.Serial(device, baudrate, timeout=1)
        print(f"\n📡 Connected to {device} at {baudrate} baud.")
        print("Press Ctrl+C to stop.\n")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"> {line}")
            else:
                time.sleep(0.01)
    except serial.SerialException as e:
        print(f"⚠️ Serial error: {e}")
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
    finally:
        try:
            ser.close()
            print("🔌 Port closed.")
        except:
            pass


if __name__ == "__main__":
    # Set your preferred port here (e.g., "COM5") or leave as None to select interactively
    preferred_port = "COM5"
    baud = 9600

    read_serial_data(preferred_port, baud)
