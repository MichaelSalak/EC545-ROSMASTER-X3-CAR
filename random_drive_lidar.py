#!/usr/bin/env python3
"""
random_drive.py + lidar obstacle detection.

Same semi-random path as random_drive.py (straight / left / right
segments of 2-4 s), but the lidar watches the area in front of the car:

  - obstacle inside SLOW_DISTANCE  -> slow down
  - obstacle inside STOP_DISTANCE  -> stop, turn in place toward the
                                      more open side until the front is
                                      clear, then resume random driving
  - lidar stops sending data       -> stop the car (fail-safe)

Every loop is logged to logs/drive_<time>.csv for the experiments.

Usage:
    python3 random_drive_lidar.py            # normal run
    python3 random_drive_lidar.py --dry-run  # lidar on, motors OFF
"""

import argparse
import csv
import os
import random
import time

from lidar_obstacle import Lidar

# Motion configuration (same values as random_drive.py)
FORWARD_SPEED = 0.08       # m/s
TURN_RATE = 0.15           # rad/s

MIN_DURATION = 2.0         # seconds
MAX_DURATION = 4.0

TEST_DURATION = 40.0       # seconds of driving (after the start delay)
START_DELAY = 15.0         # seconds to unplug + place car on floor

# Straight is more likely than turning
DIRECTIONS = [
    ("STRAIGHT", 0.0),
    ("STRAIGHT", 0.0),
    ("LEFT", TURN_RATE),
    ("RIGHT", -TURN_RATE)
]

# Obstacle detection configuration
FRONT_HALF_ANGLE = 30      # deg, front zone = -30..+30
SIDE_MIN_ANGLE = 30        # deg, side zones used to pick turn direction
SIDE_MAX_ANGLE = 100
STOP_DISTANCE = 0.35       # m, stop and avoid if anything is closer
SLOW_DISTANCE = 0.60       # m, start slowing down
CLEAR_DISTANCE = 0.50      # m, front must be this clear to drive again
MIN_SLOW_FACTOR = 0.4      # never go slower than 40% of FORWARD_SPEED

AVOID_TURN_RATE = 0.4      # rad/s, turn-in-place speed while avoiding
AVOID_TIMEOUT = 15.0       # s, can't find a way out -> end run
LIDAR_FAULT_TIMEOUT = 3.0  # s, no lidar data this long -> end run

LOOP_PERIOD = 0.05         # s (20 Hz control loop)


class DryRunBot(object):
    """Stands in for Rosmaster when testing without moving the car."""

    def set_car_motion(self, v_x, v_y, v_z):
        pass


def new_segment(now):
    direction, angular_velocity = random.choice(DIRECTIONS)
    duration = random.uniform(MIN_DURATION, MAX_DURATION)
    print("{} for {:.1f} seconds".format(direction, duration))
    return direction, angular_velocity, now + duration


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                        help="read lidar and print decisions, motors off")
    parser.add_argument("--no-delay", action="store_true",
                        help="skip the 15 s start delay")
    args = parser.parse_args()

    if args.dry_run:
        bot = DryRunBot()
        print("DRY RUN: motors will not move")
    else:
        from Rosmaster_Lib import Rosmaster
        bot = Rosmaster()

    lidar = Lidar()

    if not os.path.isdir("logs"):
        os.makedirs("logs")
    log_name = time.strftime("logs/drive_%Y%m%d_%H%M%S.csv")
    log_file = open(log_name, "w")
    log = csv.writer(log_file)
    log.writerow(["t", "state", "segment", "front_m", "left_m",
                  "right_m", "cmd_vx", "cmd_vz"])

    obstacle_count = 0

    try:
        lidar.start()
        print("Lidar started")

        if not args.no_delay:
            print("Starting random drive in {:.0f} seconds...".format(START_DELAY))
            time.sleep(START_DELAY)

        start_time = time.monotonic()
        state = "DRIVE"
        direction, angular_velocity, segment_end = new_segment(start_time)
        avoid_dir = 1
        avoid_start = 0.0
        fault_start = None
        last_cmd = None

        while time.monotonic() - start_time < TEST_DURATION:
            now = time.monotonic()
            front = left = right = float("inf")

            if not lidar.is_healthy():
                # FAIL-SAFE: never drive blind
                if fault_start is None:
                    fault_start = now
                    print("LIDAR FAULT - stopping car (error={})".format(lidar.error))
                if now - fault_start > LIDAR_FAULT_TIMEOUT:
                    print("Lidar did not recover - ending run")
                    break
                cmd = (0.0, 0.0)
                state_name = "LIDAR_FAULT"
            else:
                if fault_start is not None:
                    print("Lidar recovered")
                    fault_start = None

                front = lidar.min_distance(-FRONT_HALF_ANGLE, FRONT_HALF_ANGLE)[0]
                left = lidar.min_distance(SIDE_MIN_ANGLE, SIDE_MAX_ANGLE)[0]
                right = lidar.min_distance(-SIDE_MAX_ANGLE, -SIDE_MIN_ANGLE)[0]

                if state == "DRIVE" and front < STOP_DISTANCE:
                    obstacle_count += 1
                    avoid_dir = 1 if left >= right else -1
                    state = "AVOID"
                    avoid_start = now
                    print("OBSTACLE {:.2f} m ahead -> turning {}".format(
                        front, "LEFT" if avoid_dir > 0 else "RIGHT"))

                if state == "AVOID":
                    if front > CLEAR_DISTANCE:
                        print("Path clear")
                        state = "DRIVE"
                        direction, angular_velocity, segment_end = new_segment(now)
                    elif now - avoid_start > AVOID_TIMEOUT:
                        print("Could not find a clear path - ending run")
                        break

                if state == "AVOID":
                    cmd = (0.0, avoid_dir * AVOID_TURN_RATE)
                else:
                    if now >= segment_end:
                        direction, angular_velocity, segment_end = new_segment(now)
                    speed = FORWARD_SPEED
                    if front < SLOW_DISTANCE:
                        # scale speed down linearly between SLOW and STOP distance
                        factor = (front - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
                        speed *= max(MIN_SLOW_FACTOR, factor)
                    cmd = (speed, angular_velocity)
                state_name = state

            # only send to the motor board when the command changes
            if cmd != last_cmd:
                bot.set_car_motion(cmd[0], 0.0, cmd[1])
                last_cmd = cmd

            log.writerow(["{:.3f}".format(now - start_time), state_name,
                          direction, "{:.3f}".format(front),
                          "{:.3f}".format(left), "{:.3f}".format(right),
                          "{:.3f}".format(cmd[0]), "{:.3f}".format(cmd[1])])

            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Stopping robot")
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.2)
        lidar.stop()
        log_file.close()
        print("Obstacles avoided: {}".format(obstacle_count))
        print("Log saved to {}".format(log_name))


if __name__ == "__main__":
    main()
