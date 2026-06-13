import serial
import struct
import time
import numpy as np
import sys
import csv
from datetime import datetime

# ─────────────────────────────────────────────
# CONFIGURATION — change PORT to your CH340 port
# ─────────────────────────────────────────────
PORT     = 'COM4'    # ← change this to your CH340 COM port
BAUD     = 115200
WIN_SIZE = 256
FS       = 10000     # 10kHz sampling rate
START    = 0xAA
END      = 0xBB

# ─────────────────────────────────────────────
# CONNECT
# ─────────────────────────────────────────────
print("=" * 50)
print("  Case C Gateway — STM32 FFT Offload")
print("=" * 50)

try:
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    print(f"[OK] Connected to {PORT} at {BAUD} baud")
except Exception as e:
    print(f"[ERROR] Cannot open {PORT}: {e}")
    print("Check: Is CH340 plugged in? Correct COM port?")
    sys.exit(1)

# ─────────────────────────────────────────────
# LOG FILE
# ─────────────────────────────────────────────
log_filename = f"gateway_log_{datetime.now().strftime('%H%M%S')}.csv"
log_file = open(log_filename, 'w', newline='')
log_writer = csv.writer(log_file)
log_writer.writerow(['frame', 'freq_hz', 't_compute_ms', 'rtt_approx_ms'])
print(f"[OK] Logging to {log_filename}")
print()
print("Waiting for STM32 offload requests...")
print("Press Ctrl+C to stop")
print("-" * 50)

# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────
frame_count  = 0
total_frames = 0
buf          = b''

def compute_fft_welch(raw_bytes):
    """
    Compute FFT on received samples.
    Returns dominant frequency in Hz.
    """
    samples = np.frombuffer(raw_bytes, dtype=np.float32).copy()
    fft_out = np.fft.rfft(samples)
    mags    = np.abs(fft_out[1:])
    peak    = int(np.argmax(mags)) + 1
    freq_hz = float(peak) * float(FS) / float(WIN_SIZE)
    return freq_hz

try:
    while True:
        # Read incoming bytes
        chunk = ser.read(64)
        if not chunk:
            continue
        buf += chunk

        # Process complete frames
        while True:
            # Find start byte
            start_idx = buf.find(bytes([START]))
            if start_idx == -1:
                buf = b''
                break

            # Discard bytes before start
            if start_idx > 0:
                buf = buf[start_idx:]

            # Check if we have enough bytes
            # Frame = START(1) + floats(WIN*4) + END(1)
            needed = 1 + WIN_SIZE * 4 + 1
            if len(buf) < needed:
                break  # wait for more data

            # Check end byte
            end_idx = 1 + WIN_SIZE * 4
            if buf[end_idx] != END:
                # Bad frame — skip this start byte
                buf = buf[1:]
                continue

            # Extract float data
            raw_data = buf[1 : 1 + WIN_SIZE * 4]

            # Remove processed frame
            buf = buf[needed:]

            # Compute FFT
            t_start     = time.perf_counter()
            freq_hz     = compute_fft_welch(raw_data)
            t_compute_ms = float(
                (time.perf_counter() - t_start) * 1000.0)

            frame_count  += 1
            total_frames += 1

            # Build response packet
            # Format: START(1) + freq(4) + t_compute(4) + csum(1) + END(1)
            freq_bytes  = struct.pack('<f', freq_hz)
            tcomp_bytes = struct.pack('<f', t_compute_ms)
            payload     = freq_bytes + tcomp_bytes
            csum        = 0
            for b_val in payload:
                csum ^= b_val
            packet = (bytes([START]) +
                      payload +
                      bytes([csum, END]))
            ser.write(packet)

            # Log
            log_writer.writerow([
                total_frames,
                f"{freq_hz:.2f}",
                f"{t_compute_ms:.4f}",
                ""
            ])
            log_file.flush()

            # Print every frame
            print(f"[{total_frames:04d}] "
                  f"freq={freq_hz:6.1f}Hz  "
                  f"t_compute={t_compute_ms:.4f}ms  "
                  f"result sent ✓")

except KeyboardInterrupt:
    print()
    print("-" * 50)
    print(f"[DONE] Total frames processed: {total_frames}")
    print(f"[DONE] Log saved to: {log_filename}")
    ser.close()
    log_file.close()
    sys.exit(0)

except Exception as e:
    print(f"[ERROR] {e}")
    ser.close()
    log_file.close()
    sys.exit(1)
