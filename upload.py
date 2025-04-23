import os
import time
import shutil
import serial.tools.list_ports
import subprocess
from pathlib import Path

def find_rpi_drive():
    for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
        path = f"{letter}:/"
        if os.path.exists(os.path.join(path, "INFO_UF2.TXT")):
            return path
    return None

def reset_to_bootsel(port):
    try:
        print(f"[INFO] Opening {port} at 1200bps to trigger reset...")
        ser = serial.Serial(port, 1200)
        ser.close()
    except Exception as e:
        print(f"[WARN] Could not open port {port}: {e}")

def upload(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    uf2_path = os.path.join(build_dir, "firmware.uf2")
    upload_port = env.subst("$UPLOAD_PORT") or "COM20"  # Default if not set

    # Step 1: Trigger BOOTSEL mode
    reset_to_bootsel(upload_port)

    # Step 2: Wait for RPI-RP2 drive
    print("[INFO] Waiting for RPI-RP2 to appear...")
    for _ in range(40):  # ~20 seconds max
        drive = find_rpi_drive()
        if drive:
            print(f"[INFO] Found RPI-RP2 at {drive}")
            break
        time.sleep(0.5)
    else:
        print("[ERROR] RPI-RP2 drive not found. Please press BOOTSEL manually.")
        return

    # Step 3: Copy .uf2 file
    print(f"[INFO] Copying {uf2_path} to {drive}")
    shutil.copy(uf2_path, os.path.join(drive, "firmware.uf2"))
    print("[INFO] Done uploading.")

    # Optional wait before serial monitor
    time.sleep(2)

    # Step 4: Launch serial monitor
    print("[INFO] Launching serial monitor...")
    subprocess.run(["platformio", "device", "monitor"])
