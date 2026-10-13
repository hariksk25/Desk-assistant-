#!/usr/bin/env python3
"""Read the ESP32 serial log for a fixed time and exit (arduino-cli monitor never exits).
usage: python3 scripts/serial_log.py <port> [seconds=60] [baud=115200]"""
import sys, time
try:
    import serial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install --user pyserial")
port = sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 60
baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
# Do not assert DTR/RTS on open. The v1 CH340 tolerates it, but on the S3's native
# USB port those lines trigger a reset into the bootloader and the sketch stops.
s = serial.Serial()
s.port = port; s.baudrate = baud; s.timeout = 0.5
s.dtr = False; s.rts = False
s.open()
with s:
    t0 = time.time()
    while time.time() - t0 < secs:
        line = s.readline()
        if line:
            print(f"{time.time() - t0:7.1f}  {line.decode('utf-8', 'replace').rstrip()}", flush=True)
