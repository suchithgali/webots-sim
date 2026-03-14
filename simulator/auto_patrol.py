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
   Supports 'fly-by' waypoints that skip the 360-degree scan."""

from controller import Robot
import sys
import os
import math
import threading
from enum import Enum
import subprocess
import time
import json  

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

# now attempt the imports
try:
    import numpy as np
    import cv2
    from pyzbar.pyzbar import decode
    print("SUCCESS: All modules loaded.")
except ImportError as e:
    print(f"STILL MISSING: {e}")

class DroneState(Enum):
    IDLE = 0
    TAKEOFF = 1       
    NAVIGATING = 2    
    CAPTURING = 3     
    LANDING = 4       
    RETURNING = 5     
    CHARGING = 6      
    PAUSED = 7        
    FAILURE = 8       
    COMPLETED = 9     


# Limits a value to a given range
def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


# Main robot class implementing the Mavic drone FSM controller and motor mixing.
class Mavic(Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 0.5
    SPIN_POSITION_GAIN = 5.0

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

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        self.camera_pitch_motor = self.getDevice("camera pitch")
        
        self.motors = [self.front_left_motor, self.front_right_motor,
                       self.rear_left_motor, self.rear_right_motor]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

        # Spin variables
        self.spin_accumulated = 0.0
        self.last_yaw = None
        self.spin_hold_position = None

        # Base disturbances
        self.roll_disturbance = 0
        self.pitch_disturbance = 0

        # high level FSM tracking
        self.state = DroneState.IDLE
        
        self.battery_low = False
        self.obstacle_detected = False
        self.drone_damaged = False
        self.temperature_unsafe = False
        self.manual_stop = False
        self.at_base = False

        # barcode scanning trackers
        self.scanned_codes = set()
        self.scan_throttle = 0
        self.is_scanning = False  # Track if background thread is active

    # Changes the active FSM state and executes any entry-actions for the new state
    def _set_state(self, new_state: DroneState):
        if new_state is None or new_state == self.state:
            return
        
        prev = self.state
        self.state = new_state
        print(f'[Mavic FSM] {prev.name} -> {new_state.name}', flush=True)

        # state entry actions
        if new_state == DroneState.TAKEOFF:
            self.target_altitude = 0.3
            self.camera_pitch_motor.setPosition(0.7)
            self.target_position[0:2] = self.current_pose[0:2]

        elif new_state == DroneState.NAVIGATING:
            self.camera_pitch_motor.setPosition(0.7)
            self.pitch_disturbance = 0
            self.roll_disturbance = 0

        elif new_state == DroneState.CAPTURING:
            self.camera_pitch_motor.setPosition(-0.3)
            self.spin_accumulated = 0.0
            self.last_yaw = None
            self.spin_hold_position = list(self.current_pose[0:2])

        elif new_state == DroneState.LANDING:
            self.camera_pitch_motor.setPosition(0.7)

    # Evaluates flags like faults, low battery, and obstacles to trigger emergency states
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

    # Background thread logic to decode barcodes from an image using ZBar
    def _process_image_worker(self, image_data, width, height):
        try:
            # 1) Convert Raw Webots Bytes to numpy array 
            # The BGRA format is what Webots passes natively
            img = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
            
            # 2) Drop the image down to Grayscale
            # PyZbar will do this internally anyway, but doing it in cv2 is much faster 
            gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)

            # 3) Decode Barcodes
            # Restrict PyZbar strictly to CODE128 for max optimization and false positive reduction
            from pyzbar.pyzbar import decode, ZBarSymbol
            barcodes = decode(gray, symbols=[ZBarSymbol.CODE128])

            # 4) Process Results
            for barcode in barcodes:
                barcode_data = barcode.data.decode("utf-8")

                # Only act on net-new barcodes we haven't seen in this session
                if barcode_data not in self.scanned_codes:
                    print(f"[BARCODE SCANNER] New Code Detected: {barcode_data}")
                    self.scanned_codes.add(barcode_data)
                    
                    # Log to a persistent file alongside this controller script
                    import os
                    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scanned_barcodes.txt")
                    with open(log_path, "a") as f:
                        f.write(barcode_data + "\n")
        except Exception as e:
            # Silently pass in background thread if decoding errors out to ensure thread dies cleanly 
            # without crashing the main Webots simulation loop
            pass 
        finally:
            # Always reset the lock so the drone can capture the next frame
            self.is_scanning = False
    # Takes a camera frame and dispatches the background worker thread for barcode processing
    def _scan_barcodes(self):
        if self.is_scanning:
            return  # Wait until previous frame finishes processing

        image = self.camera.getImage()
        if not image:
            return
            
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        # Copy image bytes so Webots C-backend doesn't overwrite it while thread works
        image_data = bytes(image)
        self.is_scanning = True

        # Run vision processing on a background thread so motor loop stays real-time
        worker = threading.Thread(target=self._process_image_worker, args=(image_data, width, height))
        worker.daemon = True
        worker.start()

    # Updates current target pose directly
    def set_position(self, pos):
        self.current_pose = pos

    # Computes pitch and roll adjustments to maintain hover over a specific x,y target
    def _hold_position(self, target_xy):
        dx = target_xy[0] - self.current_pose[0]
        dy = target_xy[1] - self.current_pose[1]
        yaw = self.current_pose[5]
        
        error_forward =  dx * np.cos(yaw) + dy * np.sin(yaw)
        error_left    = -dx * np.sin(yaw) + dy * np.cos(yaw)

        self.pitch_disturbance = clamp(-error_forward * self.SPIN_POSITION_GAIN, -1, 1)
        self.roll_disturbance  = clamp(error_left * self.SPIN_POSITION_GAIN, -1, 1)
        
        return 0, self.pitch_disturbance

    # Gradually spins the drone 360 degrees to scan the area, returning True when complete
    def _perform_spin(self, yaw_rate):
        current_yaw = self.current_pose[5]

        if self.last_yaw is None:
            self.last_yaw = current_yaw
            return False, 0, 0

        delta = current_yaw - self.last_yaw
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        self.spin_accumulated += abs(delta)
        self.last_yaw = current_yaw

        _, pitch_corr = self._hold_position(self.spin_hold_position)

        if self.spin_accumulated >= 2 * np.pi:
            return True, 0, 0

        target_spin_rate = 0.5 
        yaw_error = target_spin_rate - yaw_rate
        yaw_correction = clamp(yaw_error * 2.0, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)

        return False, yaw_correction, pitch_corr
# Navigates the drone towards the current waypoint using yaw alignment and forward pitch
    
    def _navigate_forward(self):
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1],
            self.target_position[0] - self.current_pose[0])

        angle_left = self.target_position[2] - self.current_pose[5]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        yaw_disturbance = clamp(angle_left, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
        
        # Only move forward if the drone is pointing somewhat towards the target
        if abs(angle_left) > 0.4:
            pitch_disturbance = 0.0
        else:
            pitch_disturbance = clamp(np.log10(abs(angle_left) + 0.001), self.MAX_PITCH_DISTURBANCE, 0.1)

        return yaw_disturbance, pitch_disturbance

    # Primary event loop: manages waypoints, processes sensors, and handles lower-level motor mixing
    def run(self):
        # Format: [X, Y, DO_SPIN]
        waypoints = [
            [3.50, 2.00, True], 
            [-0.07, 1.86, True], 
            [-3.89, 1.85, True], 
            [-7.15, 1.85, True], 
            [-8.8, 2.2, False],    # Fly-by Waypoint
            [-8.8, -1.5, False],   # Fly-by Waypoint
            [-7.25, -1.00, True], 
            [-3.72, -2.41, True], 
            [-0.19, -1.75, True],
            [3.31, -1.51, True], 
            [7.41166, 2.71991, False] 
        ]

        yaw_disturbance = 0
        pitch_disturbance = 0

        # Setup telemetry file path - use fixed location
        telemetry_file = "/Users/suchithgali/C++ Files/CSE120/S26-CSE-303/simulator/telemetry.json"
        print("="*50, flush=True)
        print(f"[TELEMETRY] Controller started!", flush=True)
        print(f"[TELEMETRY] Writing to: {telemetry_file}", flush=True)
        print("="*50, flush=True)
        
        # Test write to verify file permissions
        try:
            with open(telemetry_file, "w") as f:
                json.dump({"status": "initializing"}, f)
            print("[TELEMETRY] Test file write successful!", flush=True)
        except Exception as e:
            print(f"[TELEMETRY] File write FAILED: {e}", flush=True)

        self._set_state(DroneState.TAKEOFF)
        self.target_position[0:2] = waypoints[0][0:2]

        while self.step(self.time_step) != -1:
            
            # sensor update
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, yaw_rate = self.gyro.getValues()
            
            if math.isnan(altitude):
                continue

            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            # barcode scanning
            if self.state == DroneState.CAPTURING:
                self.scan_throttle += 1
                if self.scan_throttle % 5 == 0:
                    self._scan_barcodes()

            # evaluate interrupts
            self._check_interrupts()

            # execute the FSM behavior
            if self.state in (DroneState.FAILURE, DroneState.COMPLETED, DroneState.CHARGING, DroneState.IDLE):
                for motor in self.motors:
                    motor.setVelocity(0.0)
                if self.state == DroneState.COMPLETED:
                    print("Patrol completed. Shutting down controller.")
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
                self.roll_disturbance = 0

                xy_dist = np.sqrt((self.target_position[0] - x_pos)**2 + (self.target_position[1] - y_pos)**2)
                if xy_dist < self.target_precision:
                    if self.target_index == len(waypoints) - 1:
                        self.target_position[0:2] = waypoints[-1][0:2]
                        self._set_state(DroneState.LANDING)
                    else:
                        do_spin = waypoints[self.target_index][2]
                        if do_spin:
                            self._set_state(DroneState.CAPTURING)
                        else:
                            # Skip the spin and go straight to the next waypoint
                            self.target_index += 1
                            self.target_position[0:2] = waypoints[self.target_index][0:2]

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
                
                if altitude <= 0.15: # Raised threshold to ensure it detects landing
                    self._set_state(DroneState.COMPLETED)
                    continue

                # Write telemetry data to file
                telemetry = {
                    "timestamp": time.time(),
                    #"state": self.state.name,
                    #"x": round(self.current_pose[0], 3),
                    #"y": round(self.current_pose[1], 3),
                    #"z": round(self.current_pose[2], 3),
                    #"roll": round(self.current_pose[3], 3),
                    #"pitch": round(self.current_pose[4], 3),
                    #"yaw": round(self.current_pose[5], 3),
                    #"altitude": round(altitude, 3),
                    #"battery_low": self.battery_low,
                    #"obstacle_detected": self.obstacle_detected,
                    #"temperature_unsafe": self.temperature_unsafe,
                    #"waypoint_index": self.target_index,
                    "scanned_codes": list(self.scanned_codes)
                }
                try:
                    with open(telemetry_file, "w") as f:
                        json.dump(telemetry, f)
                except Exception as e:
                    pass  # Silently ignore file write errors to avoid crashing simulation

            # low level motor fixing
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + self.roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + self.pitch_disturbance
            yaw_input = yaw_disturbance
            
            clamped_diff_alt = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_diff_alt, 3.0)

            front_left_input  = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_input   = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_input  = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_input)
            self.front_right_motor.setVelocity(-front_right_input)
            self.rear_left_motor.setVelocity(-rear_left_input)
            self.rear_right_motor.setVelocity(rear_right_input)


if __name__ == "__main__":
    # Calculate path to the world file
    simulator_dir = os.path.dirname(os.path.abspath(__file__))
    wbt_path = os.path.join(simulator_dir, "world", "worlds", "wharehouse.wbt")
    
    print("[Launcher] Starting Webots automatically...", flush=True)
    # Start webots as a background process
    # Clear DYLD_LIBRARY_PATH just for the webots subprocess so it doesn't conflict with Homebrew ODE
    webots_env = os.environ.copy()
    if 'DYLD_LIBRARY_PATH' in webots_env:
        del webots_env['DYLD_LIBRARY_PATH']
        
    webots_process = subprocess.Popen(
        ["/Applications/Webots.app/Contents/MacOS/webots", wbt_path],
        env=webots_env
    )
    
    try:
        # To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
        # with a linear and angular damping both of 0.5
        robot = Mavic()
        robot.run()
        
        # Save the scanned barcodes to the database once completed
        print(f"[Launcher] Saving {len(robot.scanned_codes)} scanned barcodes to database...", flush=True)
        
        # Add the backend to sys.path to import the get_connection code
        backend_path = os.path.join(simulator_dir, "..", "backend")
        sys.path.append(backend_path)
        from app.db import insert_scan
        
        insert_scan(list(robot.scanned_codes))
        print("[Launcher] Database updated successfully!")

    except KeyboardInterrupt:
        print("\n[Launcher] Manual interrupt received.")
    finally:
        print("[Launcher] Closing Webots...", flush=True)
        webots_process.terminate()
        try:
            webots_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            webots_process.kill()
        print("[Launcher] Done.")