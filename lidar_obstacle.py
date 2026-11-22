#!/usr/bin/env python3
"""
Lidar reader for the ROSMASTER X3 (RPLIDAR) -- no ROS needed.

Talks to the RPLIDAR directly over USB serial (same approach as
random_drive.py, which talks to the motor board directly with
Rosmaster_Lib). Only needs pyserial, which Rosmaster_Lib already uses.

Run this file by itself to TEST / CALIBRATE the lidar.
The car's wheels never move in this mode:

    python3 lidar_obstacle.py
"""

import math
import threading
import time

import serial

# ---------------- Lidar configuration ----------------
LIDAR_PORT = "/dev/rplidar"   # Yahboom udev name; fallback is usually /dev/ttyUSB1
LIDAR_BAUD = 115200           # RPLIDAR A1 = 115200, S2 = 1000000

# Raw lidar angle (degrees) that points at the FRONT of the car.
# Find it with calibration mode (see bottom of this file).
FRONT_ANGLE_OFFSET = 0.0

# RPLIDAR reports angles clockwise. We convert so that
# +angle = LEFT of the car, -angle = RIGHT (same sign as the
# LEFT/RIGHT turn rates in random_drive.py). If the calibration shows
# left/right swapped, flip this.
LIDAR_CLOCKWISE = True

MIN_VALID_DIST = 0.12   # m, closer hits are the car's own parts -> ignored
MAX_VALID_DIST = 8.0    # m
DATA_TIMEOUT = 0.5      # s, readings older than this are not trusted

_STOP_CMD = b"\xA5\x25"
_SCAN_CMD = b"\xA5\x20"


class Lidar(object):
    """Reads RPLIDAR scans in a background thread.

    Keeps one distance per degree (index 0..359, robot frame:
    0 = front, 90 = left, 180 = back, 270 = right).
    """

    def __init__(self, port=LIDAR_PORT, baudrate=LIDAR_BAUD,
                 front_offset=FRONT_ANGLE_OFFSET, clockwise=LIDAR_CLOCKWISE):
        self.port = port
        self.baudrate = baudrate
        self.front_offset = front_offset
        self.clockwise = clockwise

        self._dist = [float("inf")] * 360   # m
        self._raw = [0.0] * 360             # raw angle that filled each bin
        self._stamp = [0.0] * 360           # time of last reading
        self._lock = threading.Lock()

        self._ser = None
        self._thread = None
        self._running = False
        self.last_data_time = 0.0
        self.error = None

    # ---------- start / stop ----------
    def start(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
        self._ser.setDTR(False)          # spins the motor on RPLIDAR A1
        time.sleep(0.5)
        self._ser.write(_STOP_CMD)       # clear any old scan
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._ser.write(_SCAN_CMD)

        descriptor = self._ser.read(7)
        if (len(descriptor) != 7 or descriptor[0] != 0xA5
                or descriptor[1] != 0x5A):
            self.stop()
            raise RuntimeError(
                "No scan reply from lidar on {} at {} baud. "
                "Check the port/baud rate.".format(self.port, self.baudrate))

        self._running = True
        self._thread = threading.Thread(target=self._read_loop)
        self._thread.daemon = True
        self._thread.start()

        # wait (up to 3 s) for the first real data before returning
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.last_data_time > 0:
                return
            time.sleep(0.05)
        self.stop()
        raise RuntimeError("Lidar answered but sent no scan data")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.write(_STOP_CMD)
                time.sleep(0.01)
                self._ser.setDTR(True)   # stop motor (A1)
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ---------- background reading ----------
    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                waiting = self._ser.in_waiting
                buf.extend(self._ser.read(max(5, waiting)))
            except Exception as e:      # USB unplugged, etc.
                self.error = str(e)
                self._running = False
                break

            points, used = parse_packets(buf)
            del buf[:used]
            if points:
                self._store(points, time.monotonic())

    def _store(self, points, now):
        with self._lock:
            for raw_angle, dist_m in points:
                angle = raw_angle - self.front_offset
                if self.clockwise:
                    angle = -angle
                idx = int(round(angle)) % 360
                if dist_m < MIN_VALID_DIST or dist_m > MAX_VALID_DIST:
                    dist_m = float("inf")   # no hit / self hit = clear
                self._dist[idx] = dist_m
                self._raw[idx] = raw_angle
                self._stamp[idx] = now
            self.last_data_time = now

    # ---------- queries used by the drive code ----------
    def is_healthy(self):
        """True if fresh lidar data is arriving."""
        return (self._running and self.error is None and
                time.monotonic() - self.last_data_time < DATA_TIMEOUT)

    def min_distance(self, from_deg, to_deg):
        """Closest obstacle between two robot-frame angles (deg).

        Example: min_distance(-30, 30) = closest thing in front.
        Returns (distance_m, angle_deg); distance is inf if clear.
        """
        now = time.monotonic()
        best, best_angle = float("inf"), None
        with self._lock:
            for deg in range(int(from_deg), int(to_deg) + 1):
                idx = deg % 360
                if now - self._stamp[idx] <= DATA_TIMEOUT:
                    if self._dist[idx] < best:
                        best, best_angle = self._dist[idx], deg
        return best, best_angle

    def nearest_raw(self):
        """(distance_m, robot_angle, raw_lidar_angle) of nearest point."""
        dist, angle = self.min_distance(-180, 179)
        if angle is None:
            return dist, None, None
        return dist, angle, self._raw[angle % 360]


def parse_packets(buf):
    """Decode RPLIDAR standard-scan packets (5 bytes each).

    Returns ([(raw_angle_deg, distance_m), ...], bytes_used).
    Skips bytes until packets line up (re-sync after noise).
    """
    points = []
    i = 0
    n = len(buf)
    while n - i >= 5:
        b0, b1 = buf[i], buf[i + 1]
        start_flag = b0 & 0x01
        inv_start_flag = (b0 >> 1) & 0x01
        check_bit = b1 & 0x01
        if start_flag == inv_start_flag or check_bit != 1:
            i += 1            # not aligned, slide one byte
            continue
        b2, b3, b4 = buf[i + 2], buf[i + 3], buf[i + 4]
        i += 5

        quality = b0 >> 2
        angle = ((b1 >> 1) | (b2 << 7)) / 64.0
        dist_mm = (b3 | (b4 << 8)) / 4.0
        if quality == 0 or dist_mm == 0:
            dist_m = float("inf")       # no return at this angle
        else:
            dist_m = dist_mm / 1000.0
        points.append((angle % 360.0, dist_m))
    return points, i


# ---------------- Calibration / test mode ----------------
def _fmt(d):
    return "  clear" if math.isinf(d) else "{:5.2f} m".format(d)


def main():
    lidar = Lidar()
    print("Opening lidar on {} ...".format(lidar.port))
    lidar.start()
    print("Lidar running. Ctrl+C to quit. (Motors are NOT used.)\n")
    try:
        while True:
            time.sleep(0.5)
            if not lidar.is_healthy():
                print("!! NO FRESH LIDAR DATA  error={}".format(lidar.error))
                continue
            front = lidar.min_distance(-30, 30)[0]
            left = lidar.min_distance(30, 100)[0]
            right = lidar.min_distance(-100, -30)[0]
            dist, angle, raw = lidar.nearest_raw()
            if angle is None:
                nearest = "nearest: none"
            else:
                nearest = "nearest: {:.2f} m at robot {:+4d} deg (raw {:5.1f})".format(
                    dist, angle, raw)
            print("FRONT {} | LEFT {} | RIGHT {} | {}".format(
                _fmt(front), _fmt(left), _fmt(right), nearest))
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        print("\nLidar stopped")


if __name__ == "__main__":
    main()
