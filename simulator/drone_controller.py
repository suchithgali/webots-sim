import webots_setup
import sys
import os
import platform
import math
import threading
import time
import json
import numpy as np
import cv2
from pyzbar.pyzbar import decode, ZBarSymbol
from controller import Robot

from drone_state import DroneState
from utils import get_telemetry_path, clamp
from waypoint_planner import WaypointPlanner 


class Mavic(Robot):
    K_VERTICAL_THRUST     = 68.5
    K_VERTICAL_OFFSET     = 0.6
    K_VERTICAL_P          = 3.0
    K_ROLL_P              = 50.0
    K_PITCH_P             = 30.0
    MAX_YAW_DISTURBANCE   = 3.0
    MAX_PITCH_DISTURBANCE = -6.0
    target_precision      = 0.3   # capture radius for scan waypoints
    transit_precision     = 0.1   # VERY tighter precision for transit corners to avoid clipping racks
    SPIN_POSITION_GAIN    = 5.0

    def __init__(self):
        """
        Initializes the drone, connects to Webots sensors, and prepares the quadcopter motors. 
        Also sets up state tracking and threads.
        """
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

        # Range finder for real-time obstacle detection.
        # Falls back gracefully if the sensor isn't present in the world file.
        try:
            self.range_finder = self.getDevice("range-finder")
            self.range_finder.enable(self.time_step)
            self._has_range_finder = True
            print("[INIT] Range-finder enabled for obstacle avoidance.", flush=True)
        except Exception:
            self.range_finder = None
            self._has_range_finder = False
            print("[INIT] No range-finder found — obstacle detection disabled.", flush=True)

        self.OBSTACLE_THRESHOLD_M = 1.2  # metres — trip avoidance manoeuvre
        self.SIDESTEP_DISTANCE    = 1.5  # metres — how far to sidestep

        self.front_left_motor   = self.getDevice("front left propeller")
        self.front_right_motor  = self.getDevice("front right propeller")
        self.rear_left_motor    = self.getDevice("rear left propeller")
        self.rear_right_motor   = self.getDevice("rear right propeller")
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

        # Dynamic replanning: avoidance waypoints injected ahead of current index
        self._replan_queue = []
        self._avoiding     = False

        self.scanned_codes  = set()
        self.scanned_detail = []
        self.scan_throttle  = 0
        self.is_scanning    = False
        self._scan_lock     = threading.Lock() 

    def _set_state(self, new_state: DroneState):
        """
        Handles Finite State Machine (FSM) transitions (e.g. TAKEOFF -> NAVIGATING).
        Manages initial configuration for camera pitch and target setups when entering a state.
        """
        if new_state is None or new_state == self.state:
            return
        prev = self.state
        self.state = new_state
        print(f'[Mavic FSM] {prev.name} -> {new_state.name}', flush=True)

        if new_state == DroneState.TAKEOFF:
            self.target_altitude = 1.3
            self.camera_pitch_motor.setPosition(0.0)
            self.target_position[0:2] = self.current_pose[0:2]
        elif new_state == DroneState.CAPTURING:
            self.camera_pitch_motor.setPosition(0.0)
            self.spin_accumulated   = 0.0
            self.last_yaw           = None
            self.spin_hold_position = list(self.current_pose[0:2])
        elif new_state == DroneState.NAVIGATING:
            self.camera_pitch_motor.setPosition(0.0)
            self.pitch_disturbance = 0
            self.roll_disturbance  = 0
        elif new_state == DroneState.LANDING:
            self.camera_pitch_motor.setPosition(0.0)

    def _check_interrupts(self):
        """
        Interrupt handler running every physics tick. Monitors health and sensors.
        If the range-finder detects an obstacle within OBSTACLE_THRESHOLD_M, it aborts the path and sidesteps.
        """
        if self.drone_damaged:
            self._set_state(DroneState.FAILURE)
            return
        if self.state in (DroneState.NAVIGATING, DroneState.CAPTURING):
            if self.battery_low or self.temperature_unsafe:
                self._set_state(DroneState.RETURNING)
                return

            # Real range-finder obstacle check
            if self._has_range_finder and not self._avoiding:
                try:
                    ranges = self.range_finder.getRangeImage()
                    if ranges:
                        min_range = min(r for r in ranges if not math.isnan(r) and r > 0)
                        if min_range < self.OBSTACLE_THRESHOLD_M:
                            print(f"[AVOID] Obstacle at {min_range:.2f}m — replanning sidestep.", flush=True)
                            self._inject_sidestep_waypoints()
                            self.obstacle_detected = True
                except Exception:
                    pass  # sensor read failure is non-fatal

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

    def _inject_sidestep_waypoints(self):
        """
        Insert two avoidance waypoints immediately ahead of the current target:
          1. Sidestep perpendicular to current heading (left by SIDESTEP_DISTANCE m).
          2. Resume toward the original next waypoint from the clear position.
        """
        if self._avoiding:
            return
        self._avoiding = True

        yaw = self.current_pose[5]
        cx  = self.current_pose[0]
        cy  = self.current_pose[1]

        sidestep_x = cx - math.sin(yaw) * self.SIDESTEP_DISTANCE
        sidestep_y = cy + math.cos(yaw) * self.SIDESTEP_DISTANCE

        orig_target = list(self.target_position[0:3]) + [False]
        self._replan_queue = [
            [sidestep_x, sidestep_y, self.target_altitude, False],
            orig_target,
        ]
        self.target_position[0] = sidestep_x
        self.target_position[1] = sidestep_y
        print(f"[AVOID] Sidestep → ({sidestep_x:.2f}, {sidestep_y:.2f}), "
              f"then resume → ({orig_target[0]:.2f}, {orig_target[1]:.2f})", flush=True)

    def _save_barcode(self, data, kind):
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scanned_barcodes.txt")
        with open(log_path, "a") as f:
            f.write(f"[{kind}] {data}\n")

    def _handle_barcode(self, barcode):
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
        print(f"║ Total unique scanned: {len(self.scanned_codes)}", flush=True)
        print(f"╚══════════════════════════════════════════════", flush=True)

        self._save_barcode(data, kind)
        scan_record = {
            "barcode": data,
            "sim_x": self.current_pose[0],
            "sim_y": self.current_pose[1],
            "sim_z": self.current_pose[2]
        }
        self.scanned_detail.append(scan_record)
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
                with self._scan_lock:
                    if barcode_data in self.scanned_codes:
                        print(f"[Re-detected]: {barcode_data}", flush=True)
                        continue
                    self.scanned_codes.add(barcode_data)
                self._handle_barcode(barcode)
        except Exception as e:
            print(f"[SCANNER ERROR] {e}", flush=True)
        finally:
            self.is_scanning = False

    def _scan_barcodes(self):
        """
        Triggers the camera. Offloads the heavy OpenCV image processing to a background thread
        so that barcode scanning doesn't block the physics engine from ticking.
        """
        if self.is_scanning:
            return
        image = self.camera.getImage()
        if not image:
            return
        width      = self.camera.getWidth()
        height     = self.camera.getHeight()
        image_data = bytes(image)
        self.is_scanning = True
        worker = threading.Thread(
            target=self._process_image_worker,
            args=(image_data, width, height)
        )
        worker.daemon = True
        worker.start()

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
        
        # Complete the sweep if we have spun 360 degrees
        if self.spin_accumulated >= 2 * np.pi:
            self._spin_steps = 0
            return True, 0, 0
            
        # Rotate slowly to give the camera thread plenty of time to scan
        yaw_error      = 1.5 - yaw_rate
        yaw_correction = clamp(yaw_error * 2.0, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
        
        return False, yaw_correction, pitch_corr

    def _navigate_forward(self):
        """
        Calculates the required yaw (rotation) and pitch (forward tilt) to move strictly towards the target.
        Applies braking math as the drone nears its destination coordinate.
        """
        # Calculates the X and Y difference between where the drone is and where it wants to be
        dx = self.target_position[0] - self.current_pose[0]
        dy = self.target_position[1] - self.current_pose[1]

        # Computes the 2D Euclidean distance (straight line) to the target
        xy_dist = np.sqrt(dx ** 2 + dy ** 2)

        # If the drone is more than 10cm away from the target horizontally
        if xy_dist > 0.1:
            # Use arctangent to figure out the absolute angle (yaw) it needs to face
            desired_yaw = np.arctan2(dy, dx)
        else:
            # If the drone is already within 10cm, it shouldn't try to change its heading (prevents spinning wildly at the target)
            desired_yaw = self.current_pose[5]
            # It sets its target yaw to the current yaw to lock it in
            self.target_position[2] = desired_yaw
            # Calculates minor pitch adjustments just to stay hovering perfectly in place
            _, pitch_corr = self._hold_position(self.target_position[0:2])
            # Returns 0 for yaw correction, and the hovering pitch correction
            return 0.0, pitch_corr

        # Saves the calculated desired heading
        self.target_position[2] = desired_yaw

        # Calculates how many radians the drone needs to rotate to face the target
        angle_left = desired_yaw - self.current_pose[5]

        # Normalizes the angle to be strictly between 0 and 2*Pi
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)

        # Converts the angle to be between -Pi and Pi (so the drone turns left for negative, right for positive, taking the shortest path)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        # If the drone is facing relatively close to the correct direction
        if abs(angle_left) < 0.3:
            yaw = self.current_pose[5]
            # Calculate how far sideways the drone has drifted off the straight line path due to wind or momentum
            error_left = -dx * np.sin(yaw) + dy * np.cos(yaw)
            # Apply a correction to stay perfectly on the invisible track line
            self.roll_disturbance = clamp(error_left * self.SPIN_POSITION_GAIN, -0.5, 0.5)
        else:
            # Temporarily disable lateral corrections if the drone is busy doing a hard turn
            self.roll_disturbance = 0.0

        # If the drone is more than 2 meters away
        if xy_dist > 2.0:
            # And it's facing the wrong way 
            if abs(angle_left) > 0.6:
                # Turn aggressively, but don't pitch forward yet 
                yaw_disturbance   = clamp(angle_left * 0.6, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
                pitch_disturbance = 0.0
            else:
                # Otherwise it means it iS facing the right way: and use a smaller yaw correction and pitch forward aggressively to go fast
                yaw_disturbance   = clamp(angle_left * 0.3, -0.3, 0.3)
                pitch_disturbance = self.MAX_PITCH_DISTURBANCE * 0.6
                
        # If the drone is closer between 0.1m and 2.0m
        elif xy_dist > 0.1:
            # Always try to turn toward the target
            yaw_disturbance = clamp(angle_left, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
            # If the drone is pointing more than 0.4 radians away from the target, level out (0 pitch) to do an in-place turn
            if abs(angle_left) > 0.4:
                pitch_disturbance = 0.0
            else:
                # Creates a multiplier that drops from 1.0 to 0.2 as the drone gets closer to the 0.1m mark
                dist_factor = clamp(xy_dist / 1.5, 0.2, 1.0)
                # Scales the forward pitch based on distance (braking) and how straight the drone is facing
                pitch_disturbance = clamp(np.log10(abs(angle_left) + 0.001), self.MAX_PITCH_DISTURBANCE, 0.1) * dist_factor
                
        # If the drone is practically on top of the target
        else:
            # Just correct heading, do not push forward or backward at all
            yaw_disturbance = clamp(angle_left, -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
            pitch_disturbance = 0.0

        # Return the computed steering inputs to the main loop
        return yaw_disturbance, pitch_disturbance

    def run(self):
        """
        The core simulator loop.
        Gets waypoints, reads sensors every tick, runs the State Machine logic, 
        and mixes X/Y/Z adjustments into quadcopter motor velocities.
        """
        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "backend", "app", "configs", "warehouses", "default.json"
        )

        try:
            with open(config_path, 'r') as f:
                warehouse_config = json.load(f)
            planner   = WaypointPlanner(warehouse_config)
            waypoints = planner.build()
        except FileNotFoundError:
            print(f"[MISSION] FATAL: Config not found at {config_path}", flush=True)
            self._set_state(DroneState.FAILURE)
            return
        except (ValueError, KeyError) as e:
            print(f"[MISSION] FATAL: Config is invalid — {e}", flush=True)
            self._set_state(DroneState.FAILURE)
            return
        except Exception as e:
            print(f"[MISSION] FATAL: Unexpected error loading config — {e}", flush=True)
            self._set_state(DroneState.FAILURE)
            return

        yaw_disturbance   = 0
        pitch_disturbance = 0

        telemetry_file = get_telemetry_path()
        print("=" * 60, flush=True)
        print(f"[MISSION] USCS Cold Storage Inventory Patrol", flush=True)
        print(f"[MISSION] Waypoints : {len(waypoints)} total", flush=True)
        print(f"[MISSION] Scan stops: {sum(1 for w in waypoints if w[3])}", flush=True)
        print("=" * 60, flush=True)
        print(f"[TELEMETRY] Running on : {platform.system()}", flush=True)
        print(f"[TELEMETRY] Writing to : {telemetry_file}", flush=True)
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

            roll, pitch, yaw                               = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude                         = self.gps.getValues()
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
                    with self._scan_lock:
                        codes  = list(self.scanned_codes)
                        detail = list(self.scanned_detail)
                    if codes:
                        print("\n╔══════════════════════════════════════════════")
                        print(f"║ MISSION COMPLETE — {len(codes)} unique barcode(s) found:")
                        for i, d in enumerate(detail, 1):
                            print(f"║  {i}. {d}")
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
                # roll_disturbance is intentionally left alone, _navigate_forward explicitly sets it to 0 or actively corrects!
                xy_dist = np.sqrt((self.target_position[0] - x_pos) ** 2 +
                                  (self.target_position[1] - y_pos) ** 2)
                z_dist = abs(self.target_altitude - altitude)
                
                current_wp_is_scan = waypoints[self.target_index][3]
                precision = self.target_precision if current_wp_is_scan else self.transit_precision

                # Must reach the target 3D coordinate (XY and Z) before marking as done
                if xy_dist < precision and z_dist < 0.2:
                    if self._replan_queue:
                        next_avoid = self._replan_queue.pop(0)
                        self.target_position[0] = next_avoid[0]
                        self.target_position[1] = next_avoid[1]
                        print(f"[AVOID] Replan step → ({next_avoid[0]:.2f}, {next_avoid[1]:.2f})", flush=True)
                        if not self._replan_queue:
                            self._avoiding         = False
                            self.obstacle_detected  = False
                            print("[AVOID] Avoidance complete — resuming mission.", flush=True)
                    elif self.target_index == len(waypoints) - 1:
                        self.target_position[0:2] = waypoints[-1][0:2]
                        self._set_state(DroneState.LANDING)
                    else:
                        if current_wp_is_scan:
                            self._set_state(DroneState.CAPTURING)
                        else:
                            self.target_index += 1
                            self.target_position[0:3] = waypoints[self.target_index][0:3]
                            self.target_altitude = self.target_position[2]  # Update 3D altitude!
                            print(f"[NAV] Transit done → waypoint {self.target_index} "
                                  f"({self.target_position[0]:.1f}, {self.target_position[1]:.1f}, z={self.target_altitude:.1f})", flush=True)

            elif self.state == DroneState.CAPTURING:
                spin_done, yaw_disturbance, pitch_disturbance = self._perform_spin(yaw_rate)
                if spin_done:
                    self.target_index += 1
                    self.target_position[0:3] = waypoints[self.target_index][0:3]
                    self.target_altitude = self.target_position[2]  # Update 3D altitude!
                    self._set_state(DroneState.NAVIGATING)

            elif self.state == DroneState.LANDING:
                yaw_disturbance, pitch_disturbance = self._hold_position(self.target_position[0:2])
                if self.target_altitude > 0.05:
                    self.target_altitude -= 0.001
                if altitude <= 0.15:
                    self._set_state(DroneState.COMPLETED)
                    continue
                with self._scan_lock:
                    telemetry = {
                        "timestamp":      time.time(),
                        "scanned_codes":  list(self.scanned_codes),
                        "scanned_detail": list(self.scanned_detail),
                    }
                try:
                    with open(telemetry_file, "w") as f:
                        json.dump(telemetry, f, indent=2)
                except Exception:
                    pass

            # Motor mixing
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