# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Uses an Explicit High-Level Finite State Machine (FSM) for mission control.
   Includes PyZbar/OpenCV computer vision for barcode scanning on a background thread.
   Supports 'fly-by' waypoints that skip the 360-degree scan.
   Cross-platform: works on macOS and Windows."""

import sys
import os
import platform

# --- Webots Early Python Path Extension ---
# If you run this file from the terminal (not through Webots), it needs to know
# where the Webots controller API is located so the import doesn't fail.
if "WEBOTS_HOME" not in os.environ:
    _sys = platform.system()
    if _sys == "Darwin":
        os.environ["WEBOTS_HOME"] = "/Applications/Webots.app"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "Contents", "lib", "controller", "python")
    elif _sys == "Windows":
        os.environ["WEBOTS_HOME"] = r"C:\Program Files\Webots"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python")
    else:
        os.environ["WEBOTS_HOME"] = "/usr/local/webots"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python")
        
    if os.path.exists(_webots_py_path) and _webots_py_path not in sys.path:
        sys.path.append(_webots_py_path)

from controller import Robot
import math
import threading
from enum import Enum
import subprocess
import time
import json
import platform

# --- Cross-platform site-packages resolution ---
# Only needed on macOS where Homebrew installs packages outside the default path
if platform.system() == "Darwin":
    # manually add the Homebrew site-packages to the path
    # this ensures Python looks where 'pip3 install --break-system-packages' put them
    homebrew_site_packages = [
        '/opt/homebrew/lib/python3.11/site-packages', # Update '3.11' to your version
        '/usr/local/lib/python3.11/site-packages',
        os.path.expanduser('~/Library/Python/3.11/lib/python/site-packages')
    ]
    for path in homebrew_site_packages:
        if os.path.exists(path) and path not in sys.path:
            sys.path.append(path)

    # fix the ZBar dynamic library path for macOS
    if os.path.exists('/opt/homebrew/lib'):
        os.environ['DYLD_LIBRARY_PATH'] = '/opt/homebrew/lib'
    elif os.path.exists('/usr/local/lib'):
        os.environ['DYLD_LIBRARY_PATH'] = '/usr/local/lib'

# --- Imports ---
try:
    import numpy as np
    import cv2
    from pyzbar.pyzbar import decode, ZBarSymbol
    print("SUCCESS: All modules loaded.")
except ImportError as e:
    print(f"STILL MISSING: {e}")


def get_telemetry_path():
    """Returns a writable telemetry path next to this script on any OS."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "telemetry.json")


def get_webots_path():
    """Returns the Webots executable path for the current OS."""
    system = platform.system()
    if system == "Darwin":
        return "/Applications/Webots.app/Contents/MacOS/webots"
    elif system == "Windows":
        candidates = [
            r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
            r"C:\Program Files (x86)\Webots\msys64\mingw64\bin\webots.exe",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return None
    else:
        # Linux
        return "/usr/local/bin/webots"


class DroneState(Enum):
    IDLE       = 0
    TAKEOFF    = 1
    NAVIGATING = 2
    CAPTURING  = 3
    LANDING    = 4
    RETURNING  = 5
    CHARGING   = 6
    PAUSED     = 7
    FAILURE    = 8
    COMPLETED  = 9


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    K_VERTICAL_THRUST    = 68.5
    K_VERTICAL_OFFSET    = 0.6
    K_VERTICAL_P         = 3.0
    K_ROLL_P             = 50.0
    K_PITCH_P            = 30.0
    MAX_YAW_DISTURBANCE  = 1.5
    MAX_PITCH_DISTURBANCE = -3.0
    target_precision     = 0.4  # Default catch radius for scan waypoints
    transit_precision    = 0.15 # Tighter precision for transit corners
    SPIN_POSITION_GAIN   = 5.0

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor  = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor   = self.getDevice("rear left propeller")
        self.rear_right_motor  = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")

        self.motors = [self.front_left_motor, self.front_right_motor,
                       self.rear_left_motor,  self.rear_right_motor]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose    = 6 * [0]
        self.target_position = [0, 0, 0]
        self.target_index    = 0
        self.target_altitude = 0

        self.spin_accumulated   = 0.0
        self.last_yaw           = None
        self.spin_hold_position = None

        self.roll_disturbance  = 0
        self.pitch_disturbance = 0

        self.state = DroneState.IDLE

        self.battery_low        = False
        self.obstacle_detected  = False
        self.drone_damaged      = False
        self.temperature_unsafe = False
        self.manual_stop        = False
        self.at_base            = False

        self.scanned_codes  = set()
        self.scanned_detail = []
        self.scan_throttle  = 0
        self.is_scanning    = False

    # --- FSM ---
    def _set_state(self, new_state: DroneState):
        if new_state is None or new_state == self.state:
            return
        prev = self.state
        self.state = new_state
        print(f'[Mavic FSM] {prev.name} -> {new_state.name}', flush=True)

        if new_state == DroneState.TAKEOFF:
            self.target_altitude = 1.3  # Fly at 1.3m to see barcodes on top of pallet stacks
            self.camera_pitch_motor.setPosition(0.7)
            self.target_position[0:2] = self.current_pose[0:2]
        elif new_state == DroneState.NAVIGATING:
            self.camera_pitch_motor.setPosition(0.7)
            self.pitch_disturbance = 0
            self.roll_disturbance  = 0
        elif new_state == DroneState.CAPTURING:
            self.camera_pitch_motor.setPosition(0.7)  # Set camera to its default pitch
            self.target_altitude += 0.1  # Increase altitude by 0.1 when scanning
            self.spin_accumulated   = 0.0
            self.last_yaw           = None
            self.spin_hold_position = list(self.current_pose[0:2])
        elif new_state == DroneState.LANDING:
            self.camera_pitch_motor.setPosition(0.7)
        elif new_state == DroneState.NAVIGATING and prev == DroneState.CAPTURING:
            self.target_altitude -= 0.1  # Restore altitude after scanning

    def _check_interrupts(self):
        if self.drone_damaged:
            self._set_state(DroneState.FAILURE)
            return
        if self.state in (DroneState.NAVIGATING, DroneState.CAPTURING):
            if self.battery_low or self.temperature_unsafe:
                self._set_state(DroneState.RETURNING)
                return
            if self.obstacle_detected or self.manual_stop:
                self._set_state(DroneState.PAUSED)
                return
        elif self.state == DroneState.PAUSED:
            if not (self.obstacle_detected or self.manual_stop):
                self._set_state(DroneState.NAVIGATING)
                return
        elif self.state == DroneState.RETURNING:
            if self.at_base:
                self._set_state(DroneState.CHARGING)
                return
        elif self.state == DroneState.CHARGING:
            if not self.battery_low and not self.manual_stop:
                self._set_state(DroneState.TAKEOFF)
                return

    # --- Barcode scanning ---
    def _save_barcode(self, data, kind):
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scanned_barcodes.txt")
        with open(log_path, "a") as f:
            f.write(f"[{kind}] {data}\n")

    def _handle_barcode(self, barcode):
        """Print full barcode details: content, type, image position, and polygon."""
        data    = barcode.data.decode("utf-8")
        kind    = barcode.type
        rect    = barcode.rect
        polygon = barcode.polygon

        print(f"╔══════════════════════════════════════════════", flush=True)
        print(f"║ NEW BARCODE DETECTED",                          flush=True)
        print(f"║ Type    : {kind}",                              flush=True)
        print(f"║ Content : {data}",                              flush=True)
        print(f"║ Position: x={rect.left}, y={rect.top}, "
              f"w={rect.width}, h={rect.height}",                 flush=True)
        print(f"║ Polygon : {[(p.x, p.y) for p in polygon]}",    flush=True)
        print(f"║ Total unique scanned: {len(self.scanned_codes) + 1}", flush=True)
        print(f"╚══════════════════════════════════════════════", flush=True)

        self._save_barcode(data, kind)
        self.scanned_detail.append(data)
        return data

    def _process_image_worker(self, image_data, width, height):
        try:
            img  = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
            gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
            gray = cv2.equalizeHist(gray)
            gray = cv2.resize(gray, (width * 2, height * 2), interpolation=cv2.INTER_LINEAR)

            barcodes = decode(gray, symbols=[ZBarSymbol.CODE128])

            for barcode in barcodes:
                barcode_data = barcode.data.decode("utf-8")
                if barcode_data not in self.scanned_codes:
                    self.scanned_codes.add(barcode_data)
                    self._handle_barcode(barcode)
                else:
                    print(f"[Re-detected]: {barcode_data}", flush=True)
        except Exception as e:
            print(f"[SCANNER ERROR] {e}", flush=True)
        finally:
            self.is_scanning = False

    def _scan_barcodes(self):
        if self.is_scanning:
            return
        image = self.camera.getImage()
        if not image:
            return
        width        = self.camera.getWidth()
        height       = self.camera.getHeight()
        image_data   = bytes(image)
        self.is_scanning = True
        worker = threading.Thread(
            target=self._process_image_worker,
            args=(image_data, width, height)
        )
        worker.daemon = True
        worker.start()

    # --- Flight helpers ---
    def set_position(self, pos):
        self.current_pose = pos

    def _hold_position(self, target_xy):
        dx  = target_xy[0] - self.current_pose[0]
        dy  = target_xy[1] - self.current_pose[1]
        yaw = self.current_pose[5]
        error_forward =  dx * np.cos(yaw) + dy * np.sin(yaw)
        error_left    = -dx * np.sin(yaw) + dy * np.cos(yaw)
        self.pitch_disturbance = clamp(-error_forward * self.SPIN_POSITION_GAIN, -1, 1)
        self.roll_disturbance  = clamp( error_left    * self.SPIN_POSITION_GAIN, -1, 1)
        return 0, self.pitch_disturbance

    def _perform_spin(self, yaw_rate):
        current_yaw = self.current_pose[5]
        if self.last_yaw is None:
            self.last_yaw = current_yaw
            self._spin_steps = 0
            return False, 0, 0
        self._spin_steps = getattr(self, '_spin_steps', 0) + 1
        delta = current_yaw - self.last_yaw
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        self.spin_accumulated += abs(delta)
        self.last_yaw = current_yaw
        _, pitch_corr = self._hold_position(self.spin_hold_position)
        if self.spin_accumulated >= 2 * np.pi:
            self._spin_steps = 0
            return True, 0, 0
        yaw_error      = 1.5 - yaw_rate
        yaw_correction = clamp(yaw_error * 2.0, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
        return False, yaw_correction, pitch_corr

    def _navigate_forward(self):
        # Desired heading toward target
        desired_yaw = np.arctan2(
            self.target_position[1] - self.current_pose[1],
            self.target_position[0] - self.current_pose[0])
        self.target_position[2] = desired_yaw

        angle_left = desired_yaw - self.current_pose[5]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        xy_dist = np.sqrt((self.target_position[0] - self.current_pose[0]) ** 2 +
                          (self.target_position[1] - self.current_pose[1]) ** 2)

        # Two-phase navigation:
        # Phase 1 — FAR (>2m): only yaw if heading is very wrong (>34 deg), else fly straight
        # Phase 2 — NEAR (<2m): full yaw+pitch to home in on waypoint
        if xy_dist > 2.0:
            if abs(angle_left) > 0.6:
                # Heading badly off — correct yaw first, don't pitch yet
                yaw_disturbance   = clamp(angle_left * 0.6, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
                pitch_disturbance = 0.0
            else:
                # Heading good — fly forward with gentle yaw trim only
                yaw_disturbance   = clamp(angle_left * 0.3, -0.3, 0.3)
                pitch_disturbance = self.MAX_PITCH_DISTURBANCE * 0.6
        else:
            # Close to waypoint — full yaw + pitch control
            yaw_disturbance = clamp(angle_left, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
            if abs(angle_left) > 0.4:
                pitch_disturbance = 0.0
            else:
                pitch_disturbance = clamp(np.log10(abs(angle_left) + 0.001), self.MAX_PITCH_DISTURBANCE, 0.1)

        return yaw_disturbance, pitch_disturbance

    # --- Main loop ---
    def run(self):
        waypoints = [
            # [x, y, do_scan]  — do_scan=True: hover+scan, False: fly-through transit
            # Takeoff hover at base station
            [  -8.500,    3.000, False],
            # ── TRANSIT: fly to SW corner of south outer aisle
            [  -6.500,  -10.500, False],
            # ── AISLE S-OUTER (Y=-10.5): scan east →  (Aisle 1 front face)
            [  -5.400,  -10.500, True ],
            [  -4.200,  -10.500, True ],
            [  -3.000,  -10.500, True ],
            [  -1.800,  -10.500, True ],
            [  -0.600,  -10.500, True ],
            [   0.600,  -10.500, True ],
            [   1.800,  -10.500, True ],
            [   3.000,  -10.500, True ],
            [   4.200,  -10.500, True ],
            [   5.400,  -10.500, True ],
            # ── TRANSIT: cross north to south inner aisle corridor
            [   6.500,  -10.500, False],
            [   6.500,   -7.350, False],
            # ── AISLE S-INNER (Y=-7.35): scan west ←  (A1 back + A2 front faces)
            [   5.400,   -7.350, True ],
            [   4.200,   -7.350, True ],
            [   3.000,   -7.350, True ],
            [   1.800,   -7.350, True ],
            [   0.600,   -7.350, True ],
            [  -0.600,   -7.350, True ],
            [  -1.800,   -7.350, True ],
            [  -3.000,   -7.350, True ],
            [  -4.200,   -7.350, True ],
            [  -5.400,   -7.350, True ],
            # ── TRANSIT: cross north to middle aisle corridor
            [  -6.500,   -7.350, False],
            [  -6.500,   -4.000, False],
            # ── AISLE MIDDLE (Y=-4.0): scan east →   (A2 back + A3 front faces)
            [  -5.400,   -4.000, True ],
            [  -4.200,   -4.000, True ],
            [  -3.000,   -4.000, True ],
            [  -1.800,   -4.000, True ],
            [  -0.600,   -4.000, True ],
            [   0.600,   -4.000, True ],
            [   1.800,   -4.000, True ],
            [   3.000,   -4.000, True ],
            [   4.200,   -4.000, True ],
            [   5.400,   -4.000, True ],
            # ── TRANSIT: cross north to north inner aisle corridor
            [   6.500,   -4.000, False],
            [   6.500,   -0.650, False],
            # ── AISLE N-INNER (Y=-0.65): scan west ←  (A3 back + A4 front faces)
            [   5.400,   -0.650, True ],
            [   4.200,   -0.650, True ],
            [   3.000,   -0.650, True ],
            [   1.800,   -0.650, True ],
            [   0.600,   -0.650, True ],
            [  -0.600,   -0.650, True ],
            [  -1.800,   -0.650, True ],
            [  -3.000,   -0.650, True ],
            [  -4.200,   -0.650, True ],
            [  -5.400,   -0.650, True ],
            # ── TRANSIT: cross north to north outer aisle corridor
            [  -6.500,   -0.650, False],
            [  -6.500,    2.500, False],
            # ── AISLE N-OUTER (Y=2.5): scan east →   (A4 back face)
            [  -5.400,    2.500, True ],
            [  -4.200,    2.500, True ],
            [  -3.000,    2.500, True ],
            [  -1.800,    2.500, True ],
            [  -0.600,    2.500, True ],
            [   0.600,    2.500, True ],
            [   1.800,    2.500, True ],
            [   3.000,    2.500, True ],
            [   4.200,    2.500, True ],
            [   5.400,    2.500, True ],
            # ── Return to base station and land
            [   6.500,    2.500, False],
            [   6.500,    3.000, False],
            [  -8.500,    3.000, False],
        ]

        yaw_disturbance   = 0
        pitch_disturbance = 0

        telemetry_file = get_telemetry_path()
        print("=" * 60, flush=True)
        print(f"[MISSION] USCS Cold Storage Inventory Patrol", flush=True)
        print(f"[MISSION] Waypoints : {len(waypoints)} total", flush=True)
        print(f"[MISSION] Scan stops: {sum(1 for w in waypoints if w[2])}", flush=True)
        print(f"[MISSION] Aisles    : S-Outer, S-Inner, Middle, N-Inner, N-Outer", flush=True)
        print("=" * 60, flush=True)
        print(f"[TELEMETRY] Running on : {platform.system()}", flush=True)
        print(f"[TELEMETRY] Writing to : {telemetry_file}",   flush=True)
        print("=" * 50, flush=True)

        try:
            with open(telemetry_file, "w") as f:
                json.dump({"status": "initializing"}, f)
            print("[TELEMETRY] Test write successful!", flush=True)
        except Exception as e:
            print(f"[TELEMETRY] File write FAILED: {e}", flush=True)

        self._set_state(DroneState.TAKEOFF)
        self.target_position[0:2] = waypoints[0][0:2]

        while self.step(self.time_step) != -1:

            roll, pitch, yaw                              = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude                        = self.gps.getValues()
            roll_acceleration, pitch_acceleration, yaw_rate = self.gyro.getValues()

            if math.isnan(altitude):
                continue

            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            if self.state == DroneState.CAPTURING:
                self.scan_throttle += 1
                if self.scan_throttle % 5 == 0:
                    self._scan_barcodes()

            self._check_interrupts()

            if self.state in (DroneState.FAILURE, DroneState.COMPLETED,
                              DroneState.CHARGING, DroneState.IDLE):
                for motor in self.motors:
                    motor.setVelocity(0.0)
                if self.state == DroneState.COMPLETED:
                    print("Patrol completed. Shutting down controller.")
                    if self.scanned_codes:
                        print("\n╔══════════════════════════════════════════════")
                        print(f"║ MISSION COMPLETE — {len(self.scanned_codes)} unique barcode(s) found:")
                        for i, detail in enumerate(self.scanned_detail, 1):
                            print(f"║  {i}. {detail}")
                        print("╚══════════════════════════════════════════════\n")
                    else:
                        print("No barcodes were detected during this mission.")
                    break
                continue

            elif self.state == DroneState.PAUSED:
                yaw_disturbance, pitch_disturbance = self._hold_position(self.current_pose[0:2])

            elif self.state == DroneState.TAKEOFF:
                yaw_disturbance, pitch_disturbance = self._hold_position(self.target_position[0:2])
                if altitude > self.target_altitude - 0.05:
                    self._set_state(DroneState.NAVIGATING)

            elif self.state == DroneState.NAVIGATING:
                yaw_disturbance, pitch_disturbance = self._navigate_forward()
                self.pitch_disturbance = pitch_disturbance
                self.roll_disturbance  = 0
                xy_dist = np.sqrt((self.target_position[0] - x_pos) ** 2 +
                                  (self.target_position[1] - y_pos) ** 2)
                # Use tighter precision for transit corners, looser for scan stops
                current_wp_is_scan = waypoints[self.target_index][2]
                precision = self.target_precision if current_wp_is_scan else self.transit_precision
                if xy_dist < precision:
                    # Arrived at waypoint[target_index]
                    if self.target_index == len(waypoints) - 1:
                        # Last waypoint → land
                        self.target_position[0:2] = waypoints[-1][0:2]
                        self._set_state(DroneState.LANDING)
                    else:
                        if current_wp_is_scan:
                            # Scan stop: spin 360° here
                            self._set_state(DroneState.CAPTURING)
                        else:
                            # Transit corner reached — advance to next waypoint
                            self.target_index += 1
                            self.target_position[0:2] = waypoints[self.target_index][0:2]
                            print(f"[NAV] Transit done → waypoint {self.target_index} "
                                  f"({self.target_position[0]:.1f}, {self.target_position[1]:.1f})", flush=True)

            elif self.state == DroneState.CAPTURING:
                spin_done, yaw_disturbance, pitch_disturbance = self._perform_spin(yaw_rate)
                if spin_done:
                    self.target_index += 1
                    self.target_position[0:2] = waypoints[self.target_index][0:2]
                    self._set_state(DroneState.NAVIGATING)

            elif self.state == DroneState.LANDING:
                yaw_disturbance, pitch_disturbance = self._hold_position(self.target_position[0:2])
                if self.target_altitude > 0.05:
                    self.target_altitude -= 0.001
                if altitude <= 0.15:
                    self._set_state(DroneState.COMPLETED)
                    continue
                telemetry = {
                    "timestamp":      time.time(),
                    "scanned_codes":  list(self.scanned_codes),
                    "scanned_detail": self.scanned_detail
                }
                try:
                    with open(telemetry_file, "w") as f:
                        json.dump(telemetry, f, indent=2)
                except Exception:
                    pass

            # --- Motor mixing ---
            roll_input  = self.K_ROLL_P  * clamp(roll,  -1, 1) + roll_acceleration  + self.roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + self.pitch_disturbance
            yaw_input   = yaw_disturbance

            clamped_diff_alt = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input   = self.K_VERTICAL_P * pow(clamped_diff_alt, 3.0)

            front_left_input  = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_input   = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_input  = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_input)
            self.front_right_motor.setVelocity(-front_right_input)
            self.rear_left_motor.setVelocity(-rear_left_input)
            self.rear_right_motor.setVelocity(rear_right_input)


if __name__ == "__main__":
    simulator_dir = os.path.dirname(os.path.abspath(__file__))

    # Webots sets this env variable when it launches a controller —
    # if it's present we're already inside Webots, so skip the subprocess launch
    already_in_webots = (
        "WEBOTS_ROBOT_ID"        in os.environ or
        "WEBOTS_CONTROLLER_URL"  in os.environ
    )

    webots_process = None
    if not already_in_webots:
        webots_exe = get_webots_path()
        wbt_path   = os.path.join(simulator_dir, "world", "worlds", "uscs_cold_storage.wbt")

        if webots_exe and os.path.exists(webots_exe):
            print(f"[Launcher] Starting Webots: {webots_exe}", flush=True)
            launch_env = os.environ.copy()
            launch_env.pop("DYLD_LIBRARY_PATH", None)  # macOS-only, safe to remove on Windows
            webots_process = subprocess.Popen([webots_exe, wbt_path], env=launch_env)
        else:
            print("[Launcher] Webots not found — assuming it launched this script.", flush=True)

    try:
        robot = Mavic()
        robot.run()

        print(f"[Launcher] Saving {len(robot.scanned_codes)} scanned barcodes to database...", flush=True)
        backend_path = os.path.join(simulator_dir, "..", "backend")
        sys.path.append(backend_path)
        from app.db import insert_scan
        insert_scan(robot.scanned_detail)
        print("[Launcher] Database updated successfully!")

    except KeyboardInterrupt:
        print("\n[Launcher] Manual interrupt received.")
    finally:
        if webots_process:
            print("[Launcher] Closing Webots...", flush=True)
            webots_process.terminate()
            try:
                webots_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                webots_process.kill()
            print("[Launcher] Done.")