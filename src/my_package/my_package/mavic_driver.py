import math 
from enum import Enum


class DroneState(Enum):
    IDLE = 0
    SCANNING = 1
    FAILURE = 2
    PAUSED = 3
    RETURNING = 4
    CHARGING = 5
    COMPLETED = 6

# These are constants that control how the drone flies

K_THRUST = 68.5
K_ALT_P = 3.0
K_ALT_D = 2.5
K_ROLL_P  = 50.0
K_PITCH_P = 30.0
K_YAW_P = 2.0
K_VX_P = 1.5
K_VY_P = 1.5

CRUISE_ALT = 3.0
ALT_RAMP = 0.02
LAND_RAMP = 0.002
KP_XY = 0.3
HOVER_KP = 0.5
MAX_SPEED = 0.7
HOVER_SPEED = 0.15
MAX_TILT_CMD = 0.2
ARRIVE_R = 1.2
HOME_R = 1.5
WARMUP_STEPS = 100
LOG_EVERY = 50
LOG_HOVER = 50
STOP_STEPS = 40
FINAL_WP_SPEED = 0.2


# This function keeps a value between two limits
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

    # This class controls the drone's behavior
class MavicDriver:
    # Set up the robot and initialize sensors and motors
    def init(self, webots_node, properties):
        robot = webots_node.robot
        self._ts = int(robot.getBasicTimeStep())
        self._dt = self._ts * 1e-3

        self._imu  = robot.getDevice('inertial unit'); self._imu.enable(self._ts)
        self._gyro = robot.getDevice('gyro'); self._gyro.enable(self._ts)
        self._gps  = robot.getDevice('gps'); self._gps.enable(self._ts)

        names = ['front right propeller', 'front left propeller',
                 'rear right propeller', 'rear left propeller']
        self._motors = [robot.getDevice(n) for n in names]
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        self._prev_pos = None
        self._filt_v = [0.0, 0.0, 0.0]
        self._alpha = 0.3
        self._state = 'WARMUP'
        self._step = 0
        self._wp = 0
        self._home = [0.0, 0.0]
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_alt = 0.5
        self._ramp_alt = 0.5
        self._land_alt = CRUISE_ALT
        self._hover_count = 0
        self._stop_count  = 0

        # High-level mission state (finite state machine)
        self.state = DroneState.IDLE
        self._last_state = None

        # Simple interrupt flags (can be updated externally)
        self.battery_low = False
        self.obstacle_detected = False
        self.drone_damaged = False
        self.temperature_unsafe = False
        self.manual_stop = False
        self.at_base = False
        self.mission_finished = False

        # This is the list of waypoints (x, y) the drone will visit in order
        # The last waypoint is usually the home/landing spot
        self._wps = [
            (3.50, 1.00),  
            (-0.07, 0.86),
            (-3.89, 0.85),
            (-7.15, 0.85),
            (-8.75, -1.85),
            (-7.25, -2.00),
            (-3.72, -1.69),
            (-0.19, -1.75),
            (3.31, -1.51),
            (7.41166, 2.71991),  
        ]
        print('[Mavic] Init done — warming up sensors')

    def _set_state(self, new_state: DroneState):
        if new_state is None or new_state == self.state:
            return
        prev = self.state
        self.state = new_state
        print(f'[Mavic] FSM {prev.name} -> {new_state.name}', flush=True)

    def _check_interrupts(self):
        """Evaluate interrupt conditions and update the high-level FSM state."""
        # ANY STATE → FAILURE (critical error)
        if self.drone_damaged:
            if self.state != DroneState.FAILURE:
                self._set_state(DroneState.FAILURE)
            return

        # State-specific transitions
        if self.state == DroneState.SCANNING:
            # SCANNING → RETURNING (battery low or temperature unsafe)
            if self.battery_low or self.temperature_unsafe:
                self._set_state(DroneState.RETURNING)
                return
            # SCANNING → PAUSED (obstacle detected or manual stop)
            if self.obstacle_detected or self.manual_stop:
                self._set_state(DroneState.PAUSED)
                return

        elif self.state == DroneState.PAUSED:
            # PAUSED → SCANNING (resume when obstacle cleared and manual stop released)
            if not (self.obstacle_detected or self.manual_stop):
                self._set_state(DroneState.SCANNING)
                return

        elif self.state == DroneState.RETURNING:
            # RETURNING → CHARGING (at base)
            if self.at_base:
                self._set_state(DroneState.CHARGING)
                return

        elif self.state == DroneState.CHARGING:
            # CHARGING → SCANNING (resume mission, here modeled as battery recovered)
            if not self.battery_low and not self.manual_stop:
                self._set_state(DroneState.SCANNING)
                return

        elif self.state == DroneState.COMPLETED:
            # COMPLETED → IDLE
            self._set_state(DroneState.IDLE)
            return

        # IDLE → SCANNING (start mission automatically on first run)
        if self.state == DroneState.IDLE:
            self._set_state(DroneState.SCANNING)

    # This function is called every time step to update the drone
    def step(self):
        # Get the current orientation and position from sensors
        roll, pitch, yaw = self._imu.getRollPitchYaw()
        gx, gy, gz = self._gps.getValues()
        rv, pv, yv = self._gyro.getValues()

        # roll, pitch, yaw: orientation angles of the drone
        # gx, gy, gz: current position (x, y, z)
        # rv, pv, yv: angular velocities (how fast the drone is rotating)

        # If the GPS is not ready, skip this step
        if math.isnan(gz):
            return

        self._step += 1  # Count the step

        # Estimate velocity by looking at position change
        if self._prev_pos is not None:
            for i, (cur, prev) in enumerate(zip([gx, gy, gz], self._prev_pos)):
                raw = (cur - prev) / self._dt
                self._filt_v[i] = self._alpha * raw + (1 - self._alpha) * self._filt_v[i]
        self._prev_pos = [gx, gy, gz]
        vx_w, vy_w, vz_w = self._filt_v

        # vx_w, vy_w, vz_w: estimated velocity in world coordinates

        # Update high-level mission state based on interrupt conditions
        self._check_interrupts()

        # Wrap existing mission logic inside the high-level FSM
        if self.state == DroneState.IDLE:
            # Stay idle, keep motors stopped
            for m in self._motors:
                m.setVelocity(0.0)
            return
        elif self.state == DroneState.SCANNING:
            # Normal mission execution
            self._navigate(gx, gy, gz)
        elif self.state == DroneState.RETURNING:
            # For now, reuse the same navigation logic; RETURNING is a semantic label
            self._navigate(gx, gy, gz)
        elif self.state == DroneState.PAUSED:
            # Hold position: no new navigation commands, maintain current altitude
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
        elif self.state in (DroneState.CHARGING, DroneState.COMPLETED, DroneState.FAILURE):
            # These states keep the drone grounded with motors stopped
            for m in self._motors:
                m.setVelocity(0.0)
            return

        # Detect mission completion from existing navigation state
        if self._state == 'DONE' and self.state not in (DroneState.COMPLETED, DroneState.FAILURE):
            self.mission_finished = True
            self._set_state(DroneState.COMPLETED)
            for m in self._motors:
                m.setVelocity(0.0)
            return

        # Print info every so often
        log_interval = LOG_HOVER if self._state == 'LAND' else LOG_EVERY
        if self._step % log_interval == 0:
            extra = ''
            if self._state == 'NAVIGATE' and self._wp < len(self._wps):
                tx, ty = self._wps[self._wp]
                d = math.hypot(tx - gx, ty - gy)
                extra = f'  wp={self._wp}/{len(self._wps)-1}  target=({tx:.2f},{ty:.2f})  d={d:.2f}m'
                if self._stop_count > 0:
                    extra += f' [BRAKING {self._stop_count}]'
            elif self._state == 'RETURN_HOME':
                extra = f' hd={math.hypot(self._home[0]-gx, self._home[1]-gy):.2f}m'
            elif self._state == 'LAND':
                spd  = math.hypot(self._filt_v[0], self._filt_v[1])
                dist = math.hypot(self._home[0]-gx, self._home[1]-gy)
                extra = f'  hd={dist:.2f}m  spd={spd:.2f}'
            print(f'[Mavic] {self._state}  pos=({gx:.2f},{gy:.2f},{gz:.2f})'
                  f'  vz={vz_w:.2f}  alt_cmd={self._cmd_alt:.2f}{extra}', flush=True)

        # Change the command from world frame to body frame
        cy, sy = math.cos(yaw), math.sin(yaw)
        cmd_bx =  cy * self._cmd_vx + sy * self._cmd_vy
        cmd_by = -sy * self._cmd_vx + cy * self._cmd_vy
        meas_bx =  cy * vx_w + sy * vy_w
        meas_by = -sy * vx_w + cy * vy_w

        # cmd_bx/cmd_by: desired speed in drone's forward/right direction
        # meas_bx/meas_by: actual speed in drone's forward/right direction

        # Calculate how much to tilt to move the right way
        pitch_ref = clamp(-K_VX_P * (cmd_bx - meas_bx), -MAX_TILT_CMD, MAX_TILT_CMD)
        roll_ref = clamp( K_VY_P * (cmd_by - meas_by), -MAX_TILT_CMD, MAX_TILT_CMD)

        # pitch_ref: how much to tilt forward/backward
        # roll_ref: how much to tilt left/right

        # Calculate how much to go up or down
        alt_err = max(self._cmd_alt, 0.3) - gz
        vertical_in = clamp(K_ALT_P * alt_err - K_ALT_D * vz_w, -3.0, 3.0)

        # alt_err: difference between target and current altitude
        # vertical_in: how much to go up or down

        # Calculate motor commands for roll, pitch, and yaw
        roll_in = K_ROLL_P  * clamp(roll,  -1, 1) + rv + roll_ref
        pitch_in = K_PITCH_P * clamp(pitch, -1, 1) + pv + pitch_ref
        yaw_in = K_YAW_P   * (0.0 - yv)

        # roll_in, pitch_in, yaw_in: control signals for each axis

        # Mix all commands to get each motor's speed
        m1 = K_THRUST + vertical_in + yaw_in + pitch_in + roll_in
        m2 = K_THRUST + vertical_in - yaw_in + pitch_in - roll_in
        m3 = K_THRUST + vertical_in - yaw_in - pitch_in + roll_in
        m4 = K_THRUST + vertical_in + yaw_in - pitch_in - roll_in

        # m1, m2, m3, m4: motor speeds for each propeller

        # Set the motor speeds
        self._motors[0].setVelocity(-m1)
        self._motors[1].setVelocity( m2)
        self._motors[2].setVelocity( m3)
        self._motors[3].setVelocity(-m4)

    # This function decides what the drone should do based on its state
    def _navigate(self, gx, gy, gz):
        if self._state == 'WARMUP':
            # WARMUP: waiting for sensors to be ready before takeoff
            self._cmd_vx = self._cmd_vy = 0.0
            self._cmd_alt = 0.5
            if self._step >= WARMUP_STEPS:
                self._home = [gx, gy]
                self._ramp_alt = gz + 0.3
                self._state = 'TAKEOFF'
                print(f'[Mavic] Taking off from ({gx:.5f}, {gy:.5f})', flush=True)

        elif self._state == 'TAKEOFF':
            # TAKEOFF: rising to the cruise altitude
            self._cmd_vx = self._cmd_vy = 0.0
            self._ramp_alt = min(self._ramp_alt + ALT_RAMP, CRUISE_ALT)
            self._cmd_alt = self._ramp_alt
            if gz >= CRUISE_ALT - 0.5:
                self._wp = 0
                self._stop_count = 0
                self._state = 'NAVIGATE'
                print(f'[Mavic] Cruise alt reached ({gz:.2f}m) — navigating', flush=True)

        elif self._state == 'NAVIGATE':
            # NAVIGATE: flying to each waypoint in the list
            if self._stop_count > 0:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_alt = CRUISE_ALT
                self._stop_count -= 1
                return

            tx, ty = self._wps[self._wp]
            is_last = (self._wp == len(self._wps) - 1)

            if is_last:
                self._fly_to(tx, ty, CRUISE_ALT, gx, gy, max_spd=FINAL_WP_SPEED)
            else:
                self._fly_to(tx, ty, CRUISE_ALT, gx, gy)

            if math.hypot(tx - gx, ty - gy) < ARRIVE_R:
                print(f'[Mavic] >>> WP {self._wp} reached at ({gx:.2f}, {gy:.2f}, {gz:.2f})',
                      flush=True)
                self._wp += 1
                if self._wp >= len(self._wps):
                    self._state = 'LAND'
                    self._land_alt = gz
                    self._hover_count = 0
                    print('[Mavic] Home reached — descending', flush=True)
                else:
                    self._stop_count = STOP_STEPS
                    nx, ny = self._wps[self._wp]
                    print(f'[Mavic]     → WP {self._wp}: ({nx:.2f}, {ny:.2f})', flush=True)

        elif self._state == 'LAND':
            # LAND: descending and slowing down to land at home
            dist = math.hypot(self._home[0] - gx, self._home[1] - gy)
            spd  = math.hypot(self._filt_v[0], self._filt_v[1])
            land_spd = min(0.15, dist * 0.4)
            self._fly_to(self._home[0], self._home[1], self._land_alt, gx, gy,
                         max_spd=land_spd, kp=HOVER_KP)
            if dist < 0.3 and spd < 0.05:
                self._land_alt = max(self._land_alt - LAND_RAMP, 0.3)
            if gz < 0.4:
                self._state = 'DONE'
                for m in self._motors:
                    m.setVelocity(0.0)
                hdist = math.hypot(self._home[0] - gx, self._home[1] - gy)
                print(f'[Mavic] Landed at ({gx:.5f}, {gy:.5f}) — {hdist:.3f}m from home',
                      flush=True)

        elif self._state == 'DONE':
            # DONE: mission complete, motors stopped
            for m in self._motors:
                m.setVelocity(0.0)

    # This function sets the speed and direction to fly to a target (tx, ty)
    def _fly_to(self, tx, ty, alt, gx, gy, max_spd=None, kp=None):
        # tx, ty: target waypoint position
        # gx, gy: current position
        # max_spd: maximum allowed speed
        # kp: proportional gain for position control
        k = kp if kp is not None else KP_XY
        vx = k * (tx - gx)
        vy = k * (ty - gy)
        dist = math.hypot(tx - gx, ty - gy)

        limit = max_spd if max_spd is not None else MAX_SPEED

        if dist < 8.0:
            limit = min(limit, max(0.15, dist * 0.15))

        spd = math.hypot(vx, vy)
        if spd > limit:
            s = limit / spd
            vx *= s
            vy *= s

        self._cmd_vx  = vx
        self._cmd_vy  = vy
        self._cmd_alt = alt