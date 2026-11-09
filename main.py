"""Interactively calibrate and demonstrate a two-axis solar tracker.

A4988 drivers cannot report stalls or physical positions. Instead of automatic
hard-stop homing, this module lets the operator jog each axis from the USB
serial console and manually identify its lower and upper endpoints. Console
commands act as software interrupts and stop jogging between step pulses.

The calibrated endpoints are saved to ENDPOINT_FILE_PATH and can be loaded
later. Loading assumes that both axes are physically at their lower endpoints;
otherwise, repeat manual calibration because physical position is unknown after
a reset. Always test with motor power readily disconnectable, and stop an axis
before it strikes a mechanical end stop.
"""

import math
import os
import sys
import time
import uselect

from machine import Pin


STEP_PULSE_WIDTH_US = 10
DIRECTION_SETUP_TIME_MS = 2
CONSOLE_POLL_INTERVAL_MS = 10

NORMAL_MIN_SPEED_SPS = 40.0
NORMAL_MAX_SPEED_SPS = 600.0
NORMAL_ACCELERATION_SPS2 = 400.0

MANUAL_MIN_SPEED_SPS = 15.0
MANUAL_MAX_SPEED_SPS = 80.0
MANUAL_ACCELERATION_SPS2 = 80.0
MANUAL_JOG_LIMIT_STEPS = 5000

ENDPOINT_FILE_PATH = "endpoints.txt"

TILT_LOWER_END_DIRECTION = 1
ROTATION_LOWER_END_DIRECTION = 0


class MotionProfile:
    """Generate a triangular or trapezoidal speed profile for one move.

    Short moves automatically reach a lower peak speed because the profile
    starts decelerating before the configured maximum speed can be reached.
    """

    def __init__(self, step_count, min_speed, max_speed, acceleration):
        if step_count <= 0:
            raise ValueError("step_count must be greater than zero")
        if min_speed <= 0 or max_speed < min_speed:
            raise ValueError("Speeds must be positive and max_speed >= min_speed")
        if acceleration <= 0:
            raise ValueError("acceleration must be greater than zero")

        self.step_count = step_count
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.acceleration = acceleration

    def interval_us(self, step_index):
        """Calculate the period of one step using a symmetric speed ramp.

        Args:
            step_index (int): Zero-based index of the step within the move.

        Returns:
            int: Requested step period in microseconds.

        Raises:
            ValueError: If the index lies outside the planned move.
        """
        if not 0 <= step_index < self.step_count:
            raise ValueError("step_index lies outside the planned move")

        distance_to_end = min(step_index, self.step_count - step_index - 1)
        speed = math.sqrt(
            self.min_speed**2 + 2.0 * self.acceleration * distance_to_end
        )
        speed = min(speed, self.max_speed)
        return max(STEP_PULSE_WIDTH_US + 1, round(1_000_000.0 / speed))


class ConsoleInterface:
    """Read USB serial commands and interrupt motion without blocking steps."""

    def __init__(self):
        self._poll = uselect.poll()
        self._poll.register(sys.stdin, uselect.POLLIN)
        self._pending_command = None

    def next_command(self):
        """Wait for the next complete console command.

        Returns:
            str: A trimmed, lowercase command, including a command that
                interrupted a jogging operation.
        """
        if self._pending_command is not None:
            command = self._pending_command
            self._pending_command = None
            return command

        while True:
            command = self._read_command()
            if command:
                return command
            time.sleep_ms(CONSOLE_POLL_INTERVAL_MS)

    def interrupt_requested(self):
        """Check for a console command without waiting for one.

        Returns:
            bool: True when a command is pending and motion should stop.
        """
        command = self._read_command()
        if not command:
            return False

        self._pending_command = command
        return True

    def _read_command(self):
        """Read one available line from the console without polling delay.

        Returns:
            str | None: The received command, or None if no input is available.
        """
        if not self._poll.poll(0):
            return None

        line = sys.stdin.readline()
        if line is None:
            return None
        return line.strip().lower()



class StepperMotor:
    """Control one STEP/DIR stepper axis and its calibrated working position."""

    def __init__(
        self,
        name,
        step_pin_number,
        direction_pin_number,
        lower_end_direction,
    ):
        if lower_end_direction not in (0, 1):
            raise ValueError("lower_end_direction must be 0 or 1")

        self.name = name
        self.step_pin = Pin(step_pin_number, Pin.OUT)
        self.direction_pin = Pin(direction_pin_number, Pin.OUT)
        self.lower_end_direction = lower_end_direction
        self.upper_end_direction = 1 - lower_end_direction
        self.position_steps = 0
        self.travel_steps = None
        self.is_calibrated = False
        self.stop()

    def move_by(self, steps):
        """Move a calibrated axis by a signed number of microsteps.

        Positive values move toward the upper endpoint and negative values
        move toward the lower endpoint. Every move accelerates and decelerates.

        Args:
            steps (int): Signed movement distance in microsteps.

        Raises:
            RuntimeError: If the axis has not been calibrated.
            ValueError: If the resulting position is outside the endpoints.
        """
        if not self.is_calibrated or self.travel_steps is None:
            raise RuntimeError(f"Calibrate the {self.name} axis before moving it")

        target_position = self.position_steps + steps
        if not 0 <= target_position <= self.travel_steps:
            raise ValueError("The requested move exceeds a calibrated endpoint")
        if steps == 0:
            return

        direction = (
            self.upper_end_direction if steps > 0 else self.lower_end_direction
        )
        self._execute_move(
            abs(steps),
            direction,
            NORMAL_MIN_SPEED_SPS,
            NORMAL_MAX_SPEED_SPS,
            NORMAL_ACCELERATION_SPS2,
        )
        self.position_steps = target_position

    def move_to(self, target_position):
        """Move a calibrated axis to an absolute working position.

        Args:
            target_position (int): Target in microsteps above the lower endpoint.

        Raises:
            RuntimeError: If the axis has not been calibrated.
            ValueError: If the target lies outside the calibrated endpoints.
        """
        if not self.is_calibrated or self.travel_steps is None:
            raise RuntimeError(f"Calibrate the {self.name} axis before moving it")
        if not 0 <= target_position <= self.travel_steps:
            raise ValueError("target_position lies outside the calibrated travel")

        self.move_by(target_position - self.position_steps)

    def manual_jog(self, direction, console):
        """Jog manually until a console command or the safety limit stops it.

        Args:
            direction (int): Driver direction level for the jog.
            console (ConsoleInterface): Console used to interrupt the motion.

        Returns:
            int: Number of steps executed before the jog stopped.
        """
        executed_steps = self._execute_move(
            MANUAL_JOG_LIMIT_STEPS,
            direction,
            MANUAL_MIN_SPEED_SPS,
            MANUAL_MAX_SPEED_SPS,
            MANUAL_ACCELERATION_SPS2,
            stop_check=console.interrupt_requested,
        )
        if direction == self.upper_end_direction:
            self.position_steps += executed_steps
        else:
            self.position_steps -= executed_steps
        return executed_steps

    def complete_calibration(self, lower_position, upper_position):
        """Establish working coordinates from two independently marked ends.

        Args:
            lower_position (int): Raw position marked as the lower endpoint.
            upper_position (int): Raw position marked as the upper endpoint.

        Returns:
            int: Absolute travel between the endpoints in microsteps.

        Raises:
            ValueError: If the endpoints coincide or the current position lies
                outside the marked range.
        """
        if lower_position == upper_position:
            raise ValueError("The lower and upper endpoints must be different")

        direction_sign = 1 if upper_position > lower_position else -1
        travel_steps = abs(upper_position - lower_position)
        working_position = (self.position_steps - lower_position) * direction_sign
        if not 0 <= working_position <= travel_steps:
            raise ValueError("Jog between the marked endpoints before finishing")

        if direction_sign < 0:
            self.upper_end_direction, self.lower_end_direction = (
                self.lower_end_direction,
                self.upper_end_direction,
            )

        self.travel_steps = travel_steps
        self.position_steps = working_position
        self.is_calibrated = True
        return travel_steps

    def stop(self):
        """Stop generating step pulses while leaving driver enable unchanged."""
        self.step_pin.value(0)

    def _execute_move(
        self,
        step_count,
        direction,
        min_speed,
        max_speed,
        acceleration,
        stop_check=None,
    ):
        """Execute a ramped move and stop step pulses on every exit.

        Args:
            step_count (int): Maximum number of microsteps to command.
            direction (int): Logic level for the driver's direction input.
            min_speed (float): Starting and ending speed in steps per second.
            max_speed (float): Maximum permitted speed in steps per second.
            acceleration (float): Speed change in steps per second squared.
            stop_check (callable | None): Optional callback checked before each
                step. Motion stops promptly when it returns True.

        Returns:
            int: Number of step pulses actually executed.
        """
        profile = MotionProfile(step_count, min_speed, max_speed, acceleration)
        self.direction_pin.value(direction)
        time.sleep_ms(DIRECTION_SETUP_TIME_MS)
        executed_steps = 0

        try:
            for step_index in range(step_count):
                if stop_check is not None and stop_check():
                    break

                step_started_us = time.ticks_us()
                interval_us = profile.interval_us(step_index)

                self.step_pin.value(1)
                time.sleep_us(STEP_PULSE_WIDTH_US)
                self.step_pin.value(0)
                executed_steps += 1

                elapsed_us = time.ticks_diff(time.ticks_us(), step_started_us)
                remaining_us = interval_us - elapsed_us
                if remaining_us > 0:
                    time.sleep_us(remaining_us)
        finally:
            self.stop()

        return executed_steps


class SolarTracker:
    """Guide calibration, persistence, and interactive tracker demonstrations."""

    def __init__(self, tilt_motor, rotation_motor, console):
        self.tilt_motor = tilt_motor
        self.rotation_motor = rotation_motor
        self.console = console

    def run(self):
        """Calibrate or load endpoints, then accept demonstration commands."""
        try:
            self.calibrate()
            self.operate()
        finally:
            self.stop()

    def calibrate(self):
        """Interactively mark both axes or load previously saved endpoints.

        Raises:
            RuntimeError: If the operator cancels calibration.
        """
        lower_positions = {
            motor.name: None for motor in (self.rotation_motor, self.tilt_motor)
        }
        upper_positions = lower_positions.copy()
        selected_motor = self.rotation_motor

        print("Calibrate both axes, or load saved endpoints with both at lower.")
        print("Stop each axis before it reaches a hard mechanical stop.")
        self._print_calibration_instructions()
        print("Selected axis: rotation")

        while True:
            command = self.console.next_command()

            if command in ("rotation", "r"):
                selected_motor = self.rotation_motor
                print("Selected axis: rotation")
            elif command in ("tilt", "t"):
                selected_motor = self.tilt_motor
                print("Selected axis: tilt")
            elif command == "axis":
                selected_motor = (
                    self.tilt_motor
                    if selected_motor is self.rotation_motor
                    else self.rotation_motor
                )
                print(f"Selected axis: {selected_motor.name}")
            elif command in ("+", "-"):
                if selected_motor.is_calibrated:
                    selected_motor.is_calibrated = False
                    selected_motor.travel_steps = None
                    lower_positions[selected_motor.name] = None
                    upper_positions[selected_motor.name] = None
                    print("Previous calibration cleared; mark both endpoints again")

                direction = (
                    selected_motor.upper_end_direction
                    if command == "+"
                    else selected_motor.lower_end_direction
                )
                print(f"Jogging {selected_motor.name}; send any command to stop")
                steps = selected_motor.manual_jog(direction, self.console)
                print(f"Jog stopped after {steps} steps")
            elif command == "stop":
                print(f"{selected_motor.name} stopped")
            elif command in ("lower", "upper"):
                if selected_motor.is_calibrated:
                    lower_positions[selected_motor.name] = None
                    upper_positions[selected_motor.name] = None

                positions = (
                    lower_positions if command == "lower" else upper_positions
                )
                positions[selected_motor.name] = selected_motor.position_steps
                selected_motor.is_calibrated = False
                selected_motor.travel_steps = None
                print(
                    f"{selected_motor.name} {command} endpoint marked at raw "
                    f"position {selected_motor.position_steps}"
                )
            elif command == "done":
                if not self._finish_calibration(lower_positions, upper_positions):
                    continue

                self._save_endpoints()
                print(f"Endpoints saved to {ENDPOINT_FILE_PATH}")
                return
            elif command == "load":
                try:
                    self._load_endpoints()
                except (OSError, ValueError) as error:
                    print(f"Could not load endpoints: {error}")
                    continue

                print("Endpoints loaded; both axes are assumed to be at lower")
                return
            elif command == "quit":
                raise RuntimeError("Calibration cancelled by the operator")
            elif command == "help":
                self._print_calibration_instructions()
            else:
                print("Unknown command; enter 'help' for the command list")

    def operate(self):
        """Accept movement and demonstration commands for calibrated axes."""
        self._print_operation_instructions()

        while True:
            command = self.console.next_command()

            if command in ("middle", "lowerall", "upperall"):
                self._move_all(command)
            elif command == "demo":
                self.demonstrate()
            elif command == "status":
                self._print_status()
            elif command == "save":
                self._save_endpoints()
                print(f"Endpoints saved to {ENDPOINT_FILE_PATH}")
            elif command == "help":
                self._print_operation_instructions()
            elif command == "quit":
                print("Controller stopped")
                return
            else:
                print("Unknown command; enter 'help' for the command list")

    def demonstrate(self):
        """Move each axis through lower, upper, middle, and lower positions."""
        for motor in (self.rotation_motor, self.tilt_motor):
            print(f"Demonstrating {motor.name}: lower, upper, middle, lower")
            motor.move_to(0)
            motor.move_to(motor.travel_steps)
            motor.move_to(motor.travel_steps // 2)
            motor.move_to(0)

        print("Demonstration complete: both axes are stopped at zero-zero")

    def stop(self):
        """Stop step pulses on both axes."""
        self.rotation_motor.stop()
        self.tilt_motor.stop()

    def _finish_calibration(self, lower_positions, upper_positions):
        """Apply both marked endpoint pairs and report missing or invalid marks.

        Args:
            lower_positions (dict): Raw lower positions indexed by axis name.
            upper_positions (dict): Raw upper positions indexed by axis name.

        Returns:
            bool: True when both axes were calibrated successfully.
        """
        for motor in (self.rotation_motor, self.tilt_motor):
            lower_position = lower_positions[motor.name]
            upper_position = upper_positions[motor.name]
            if lower_position is None or upper_position is None:
                print(f"Mark both endpoints for {motor.name} before finishing")
                return False

            try:
                travel_steps = motor.complete_calibration(
                    lower_position, upper_position
                )
            except ValueError as error:
                print(f"{motor.name}: {error}")
                return False

            print(f"{motor.name} travel confirmed: {travel_steps} steps")

        return True

    def _move_all(self, command):
        """Move both axes to the position named by an operation command.

        Args:
            command (str): middle, lowerall, or upperall.
        """
        for motor in (self.rotation_motor, self.tilt_motor):
            if command == "middle":
                target_position = motor.travel_steps // 2
            elif command == "lowerall":
                target_position = 0
            else:
                target_position = motor.travel_steps

            print(f"Moving {motor.name} to {target_position}")
            motor.move_to(target_position)

    def _print_status(self):
        """Print the current working position and travel of both axes."""
        for motor in (self.rotation_motor, self.tilt_motor):
            print(
                f"{motor.name}: position={motor.position_steps}, "
                f"lower=0, upper={motor.travel_steps}"
            )

    def _print_calibration_instructions(self):
        """Print the commands available during the shared calibration session."""
        print("Commands (press Enter after each command):")
        print("  rotation / r  select the rotation axis")
        print("  tilt / t      select the tilt axis")
        print("  axis          toggle between rotation and tilt")
        print("  +             jog the selected axis toward upper")
        print("  -             jog the selected axis toward lower")
        print("  stop          stop jogging; any command also stops motion")
        print("  lower         mark lower; upper and lower can be any order")
        print("  upper         mark upper; its raw position may be either sign")
        print("  done          confirm both endpoint pairs and save them")
        print("  load          load endpoints with both axes physically at lower")
        print("  help          show these commands again")
        print("  quit          cancel calibration and stop both motors")

    def _print_operation_instructions(self):
        """Print movement commands available after calibration or loading."""
        print("Operation commands:")
        print("  middle   move both axes to their middle positions")
        print("  lowerall move both axes to their lower endpoints")
        print("  upperall move both axes to their upper endpoints")
        print("  demo     run lower, upper, middle, lower on both axes")
        print("  status   show saved endpoints and current positions")
        print("  save     save the current endpoint configuration")
        print("  help     show these commands again")
        print("  quit     stop the controller")

    def _save_endpoints(self):
        """Atomically save calibrated endpoints and directions as plain text."""
        lines = ["version=1"]
        for motor in (self.rotation_motor, self.tilt_motor):
            lines.extend(
                (
                    f"{motor.name}_lower=0",
                    f"{motor.name}_upper={motor.travel_steps}",
                    f"{motor.name}_lower_direction={motor.lower_end_direction}",
                    f"{motor.name}_upper_direction={motor.upper_end_direction}",
                )
            )

        temporary_path = ENDPOINT_FILE_PATH + ".tmp"
        with open(temporary_path, "w") as endpoint_file:
            endpoint_file.write("\n".join(lines) + "\n")

        try:
            os.remove(ENDPOINT_FILE_PATH)
        except OSError:
            pass
        os.rename(temporary_path, ENDPOINT_FILE_PATH)

    def _load_endpoints(self):
        """Load endpoint travel and direction mappings from the text file.

        Raises:
            OSError: If the endpoint file cannot be read.
            ValueError: If the file contains missing or invalid settings.
        """
        values = {}
        with open(ENDPOINT_FILE_PATH, "r") as endpoint_file:
            for raw_line in endpoint_file:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue

                key, separator, value = line.partition("=")
                if not separator:
                    raise ValueError("Invalid endpoint-file line")
                values[key] = value

        if values.get("version") != "1":
            raise ValueError("Unsupported endpoint-file version")

        settings = {}
        for motor in (self.rotation_motor, self.tilt_motor):
            try:
                lower = int(values[f"{motor.name}_lower"])
                upper = int(values[f"{motor.name}_upper"])
                lower_direction = int(values[f"{motor.name}_lower_direction"])
                upper_direction = int(values[f"{motor.name}_upper_direction"])
            except KeyError as error:
                raise ValueError(f"Missing setting: {error}")
            except ValueError:
                raise ValueError("Endpoint values must be integers")

            if lower != 0 or upper <= lower:
                raise ValueError(f"Invalid saved range for {motor.name}")
            directions_are_valid = (
                lower_direction in (0, 1)
                and upper_direction == 1 - lower_direction
            )
            if not directions_are_valid:
                raise ValueError(f"Invalid saved directions for {motor.name}")

            settings[motor.name] = (
                upper - lower,
                lower_direction,
                upper_direction,
            )

        for motor in (self.rotation_motor, self.tilt_motor):
            travel_steps, lower_direction, upper_direction = settings[motor.name]
            motor.travel_steps = travel_steps
            motor.lower_end_direction = lower_direction
            motor.upper_end_direction = upper_direction
            motor.position_steps = 0
            motor.is_calibrated = True


def main():
    """Create the hardware objects and run interactive calibration."""
    tilt_motor = StepperMotor(
        name="tilt",
        step_pin_number=14,
        direction_pin_number=15,
        lower_end_direction=TILT_LOWER_END_DIRECTION,
    )
    rotation_motor = StepperMotor(
        name="rotation",
        step_pin_number=16,
        direction_pin_number=17,
        lower_end_direction=ROTATION_LOWER_END_DIRECTION,
    )
    console = ConsoleInterface()
    tracker = SolarTracker(tilt_motor, rotation_motor, console)
    tracker.run()


if __name__ == "__main__":
    main()
