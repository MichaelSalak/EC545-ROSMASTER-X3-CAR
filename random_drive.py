
from Rosmaster_Lib import Rosmaster
import random
import time

# Motion configuration
FORWARD_SPEED = 0.08       # m/s
TURN_RATE = 0.15           # rad/s

MIN_DURATION = 2.0         # seconds
MAX_DURATION = 4.0

TEST_DURATION = 40.0       # stop after 40 seconds

# Straight is more likely than turning
DIRECTIONS = [
    ("STRAIGHT", 0.0),
    ("STRAIGHT", 0.0),
    ("LEFT", TURN_RATE),
    ("RIGHT", -TURN_RATE)
]


def main():
    bot = Rosmaster()
    start_time = time.monotonic()

    try:
        print("Starting random drive in 15 seconds...")
        time.sleep(15)

        while time.monotonic() - start_time < TEST_DURATION:

            direction, angular_velocity = random.choice(DIRECTIONS)

            duration = random.uniform(
                MIN_DURATION,
                MAX_DURATION
            )

            # Do not exceed the test duration
            remaining = TEST_DURATION - (
                time.monotonic() - start_time
            )
            duration = min(duration, remaining)

            print("{} for {:.1f} seconds".format(
                direction, duration
            ))

            bot.set_car_motion(
                FORWARD_SPEED,
                0.0,
                angular_velocity
            )

            time.sleep(duration)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Stopping robot")
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.2)


if __name__ == "__main__":
    main()
