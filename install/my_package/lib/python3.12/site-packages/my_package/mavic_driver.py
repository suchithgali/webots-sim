"""
mavic_driver.py — Webots Python plugin for Mavic 2 Pro waypoint mission.

State machine:  WARMUP → TAKEOFF → NAVIGATE → RETURN_HOME → LAND → DONE

Reads sensors and commands motors directly via the Webots API.
No external ROS2 nodes or cmd_vel topics needed.

KEY FIX vs previous version: PD altitude control (vertical velocity
damping) prevents the altitude oscillation that caused repeated crashes.
"""

import math

# ── Gains (Cyberbotics reference + tuning) ─────────────────────────────────
K_THRUST  = 68.5      # base motor speed for hover
K_ALT_P   = 3.0       # altitude P
K_ALT_D   = 2.5       # altitude D — damps vertical velocity to prevent overshoot
K_ROLL_P  = 50.0
K_PITCH_P = 30.0
K_YAW_P   = 2.0
K_VX_P    = 1.5       # forward velocity → pitch reference
K_VY_P    = 1.5       # lateral velocity → roll reference

CRUISE_ALT   = 3.0    # metres (lower = more stable)
ALT_RAMP     = 0.02   # altitude increment per step during takeoff (m)
LAND_RAMP    = 0.002  # altitude decrement per step during landing (m) — very slow
KP_XY        = 0.3    # position → velocity (m/s per m error)
HOVER_KP     = 0.5    # higher gain during HOVER/LAND for tight convergence
MAX_SPEED    = 1.0    # horizontal speed cap (m/s) — conservative for stability
HOVER_SPEED  = 0.15   # slow approach near home — limits overshoot to ~10 cm
MAX_TILT_CMD = 0.2    # max attitude reference (rad ~11°) — limits tilt & lift loss
ARRIVE_R     = 2.0    # waypoint capture radius (m) — wider to prevent near-miss overshoot
HOME_R       = 1.5    # radius at which RETURN_HOME → LAND
WARMUP_STEPS = 100    # sensor warmup (~0.8 s at 8 ms timestep)
LOG_EVERY    = 50     # log every N steps (~0.4 s)
LOG_HOVER    = 50     # log every N steps during HOVER/LAND (~0.4 s)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MavicDriver:
    """Webots Python plugin loaded by mavic_webots.urdf."""

    def init(self, webots_node, properties):
        robot = webots_node.robot
        self._ts = int(robot.getBasicTimeStep())
        self._dt = self._ts * 1e-3

        # Sensors
        self._imu  = robot.getDevice('inertial unit'); self._imu.enable(self._ts)
        self._gyro = robot.getDevice('gyro');          self._gyro.enable(self._ts)
        self._gps  = robot.getDevice('gps');           self._gps.enable(self._ts)

        # Motors [FR, FL, RR, RL]
        names = ['front right propeller', 'front left propeller',
                 'rear right propeller', 'rear left propeller']
        self._motors = [robot.getDevice(n) for n in names]
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # Filtered velocity from GPS diffs (x, y, z)
        self._prev_pos = None
        self._filt_v   = [0.0, 0.0, 0.0]
        self._alpha    = 0.3          # low-pass smoothing

        # Navigation
        self._state    = 'WARMUP'
        self._step     = 0
        self._wp       = 0
        self._home     = [0.0, 0.0]
        self._cmd_vx   = 0.0
        self._cmd_vy   = 0.0
        self._cmd_alt  = 0.5
        self._ramp_alt = 0.5
        self._land_alt = CRUISE_ALT  # will be set properly when LAND starts
        self._hover_count = 0        # steps spent in LAND before alt ramp starts

        # Waypoints [x, y] — do NOT include home as last WP (RETURN_HOME handles that)
        self._wps = [
            ( 3.50,  1.00 ),   
            ( -0.07,  0.86 ),  
            ( -3.89,  0.85 ),
            ( -7.15,  0.85 ),
            ( -8.75,  -1.85 ),  
            ( -7.25,  -2.00 ), 
            ( -3.72,  -1.69 ),
            ( -0.19,  -1.75 ),
            ( 3.31,  -1.51 ),
            ( 6.29,  -1.52 ),
            ( 3.35,  -8.45 ),
            ( -0.07,  -8.43 ),
            ( -3.37,  -8.47 ),
            ( -6.79,  -8.36 ),
            ( -8.63,  -10.26 ),
            ( -6.59,  -10.59 ),
            ( -3.27,  -10.59 ),
            ( -0.07,  -10.59 ),
            ( 3.35,  -10.59 ),
            ( 7.12,  -10.59 ), 
            ( 7.41166, 2.71991),
        ]
        print('[Mavic] Init done — warming up sensors')

    # ── Main step ─────────────────────────────────────────────────────────
    def step(self):
        roll, pitch, yaw = self._imu.getRollPitchYaw()
        gx, gy, gz       = self._gps.getValues()
        rv, pv, yv       = self._gyro.getValues()
        if math.isnan(gz):
            return

        self._step += 1

        # ── Velocity estimation (low-pass filtered GPS diff) ─────────────
        if self._prev_pos is not None:
            for i, (cur, prev) in enumerate(zip([gx, gy, gz], self._prev_pos)):
                raw = (cur - prev) / self._dt
                self._filt_v[i] = self._alpha * raw + (1 - self._alpha) * self._filt_v[i]
        self._prev_pos = [gx, gy, gz]
        vx_w, vy_w, vz_w = self._filt_v

        # ── Navigation state machine (sets _cmd_vx/vy/alt) ──────────────
        self._navigate(gx, gy, gz)

        # If motors are off (DONE), skip the rest of the control loop
        if self._state == 'DONE':
            return

        # ── Logging ──────────────────────────────────────────────────────
        log_interval = LOG_HOVER if self._state == 'LAND' else LOG_EVERY
        if self._step % log_interval == 0:
            extra = ''
            if self._state == 'NAVIGATE' and self._wp < len(self._wps):
                tx, ty = self._wps[self._wp]
                extra = f'  wp={self._wp} d={math.hypot(tx - gx, ty - gy):.2f}m'
            elif self._state == 'RETURN_HOME':
                extra = f'  hd={math.hypot(self._home[0] - gx, self._home[1] - gy):.2f}m'
            elif self._state == 'LAND':
                spd = math.hypot(self._filt_v[0], self._filt_v[1])
                dist = math.hypot(self._home[0] - gx, self._home[1] - gy)
                extra = f'  hd={dist:.2f}m  spd={spd:.2f}'
            print(f'[Mavic] {self._state}  pos=({gx:.2f},{gy:.2f},{gz:.2f})'
                  f'  vz={vz_w:.2f}  alt_cmd={self._cmd_alt:.2f}{extra}')

        # ── Rotate velocity & commands to body frame ─────────────────────
        cy, sy = math.cos(yaw), math.sin(yaw)
        cmd_bx  =  cy * self._cmd_vx + sy * self._cmd_vy
        cmd_by  = -sy * self._cmd_vx + cy * self._cmd_vy
        meas_bx =  cy * vx_w + sy * vy_w
        meas_by = -sy * vx_w + cy * vy_w

        # ── Velocity P → attitude reference (tightly clamped) ────────────
        pitch_ref = clamp(-K_VX_P * (cmd_bx - meas_bx), -MAX_TILT_CMD, MAX_TILT_CMD)
        roll_ref  = clamp( K_VY_P * (cmd_by - meas_by), -MAX_TILT_CMD, MAX_TILT_CMD)

        # ── Altitude PD — the D term prevents the oscillation ────────────
        alt_err     = max(self._cmd_alt, 0.3) - gz
        vertical_in = clamp(K_ALT_P * alt_err - K_ALT_D * vz_w, -3.0, 3.0)

        # ── Attitude PD ──────────────────────────────────────────────────
        roll_in  = K_ROLL_P  * clamp(roll,  -1, 1) + rv + roll_ref
        pitch_in = K_PITCH_P * clamp(pitch, -1, 1) + pv + pitch_ref
        yaw_in   = K_YAW_P   * (0.0 - yv)

        # ── Motor mixing (Cyberbotics reference) ─────────────────────────
        m1 = K_THRUST + vertical_in + yaw_in + pitch_in + roll_in
        m2 = K_THRUST + vertical_in - yaw_in + pitch_in - roll_in
        m3 = K_THRUST + vertical_in - yaw_in - pitch_in + roll_in
        m4 = K_THRUST + vertical_in + yaw_in - pitch_in - roll_in

        self._motors[0].setVelocity(-m1)   # FR  (CW)
        self._motors[1].setVelocity( m2)   # FL  (CCW)
        self._motors[2].setVelocity( m3)   # RR  (CCW)
        self._motors[3].setVelocity(-m4)   # RL  (CW)

    # ── Navigation state machine ──────────────────────────────────────────
    def _navigate(self, gx, gy, gz):

        if self._state == 'WARMUP':
            self._cmd_vx = self._cmd_vy = 0.0
            self._cmd_alt = 0.5
            if self._step >= WARMUP_STEPS:
                self._home = [gx, gy]
                self._ramp_alt = gz + 0.3
                self._state = 'TAKEOFF'
                print(f'[Mavic] Taking off from ({gx:.5f}, {gy:.5f})', flush=True)

        elif self._state == 'TAKEOFF':
            self._cmd_vx = self._cmd_vy = 0.0
            self._ramp_alt = min(self._ramp_alt + ALT_RAMP, CRUISE_ALT)
            self._cmd_alt = self._ramp_alt
            if gz >= CRUISE_ALT - 0.5:
                self._wp = 0
                self._state = 'NAVIGATE'
                print(f'[Mavic] Cruise alt reached ({gz:.2f}m) — navigating')

        elif self._state == 'NAVIGATE':
            tx, ty = self._wps[self._wp]
            self._fly_to(tx, ty, CRUISE_ALT, gx, gy)
            if math.hypot(tx - gx, ty - gy) < ARRIVE_R:
                print(f'[Mavic] >>> WP {self._wp} reached at ({gx:.2f}, {gy:.2f}, {gz:.2f})')
                self._wp += 1
                if self._wp >= len(self._wps):
                    self._state = 'RETURN_HOME'
                    print(f'[Mavic] All waypoints done — returning home')

        elif self._state == 'RETURN_HOME':
            dist = math.hypot(self._home[0] - gx, self._home[1] - gy)
            # Ramp speed: 0.3 far away, linearly down to HOVER_SPEED at ~3.75m
            approach_spd = max(HOVER_SPEED, min(0.3, dist * 0.04))
            self._fly_to(self._home[0], self._home[1], CRUISE_ALT, gx, gy, max_spd=approach_spd)
            if dist < HOME_R:
                self._land_alt = gz
                self._hover_count = 0
                self._state = 'LAND'
                print(f'[Mavic] Over home (hd={dist:.2f}m) — descending while centering')

        elif self._state == 'LAND':
            # Correct position toward home — distance-proportional speed
            dist = math.hypot(self._home[0] - gx, self._home[1] - gy)
            spd = math.hypot(self._filt_v[0], self._filt_v[1])
            land_spd = min(0.15, dist * 0.4)    # at 0.3m → 0.12, at 0.1m → 0.04
            self._fly_to(self._home[0], self._home[1], self._land_alt, gx, gy, max_spd=land_spd, kp=HOVER_KP)
            # Only descend once actually centered and stopped
            if dist < 0.3 and spd < 0.05:
                self._land_alt = max(self._land_alt - LAND_RAMP, 0.3)
            if gz < 0.4:
                self._state = 'DONE'
                # Kill motors immediately so drone doesn't bounce/flip
                for m in self._motors:
                    m.setVelocity(0.0)
                hdist = math.hypot(self._home[0] - gx, self._home[1] - gy)
                print(f'[Mavic] Landed at ({gx:.5f}, {gy:.5f}) — {hdist:.3f}m from home — motors off', flush=True)

        elif self._state == 'DONE':
            # Motors already off — do nothing
            for m in self._motors:
                m.setVelocity(0.0)
            return

    def _fly_to(self, tx, ty, alt, gx, gy, max_spd=None, kp=None):
        k = kp if kp is not None else KP_XY
        vx = k * (tx - gx)
        vy = k * (ty - gy)
        limit = max_spd if max_spd is not None else MAX_SPEED
        # Decelerate when close to target to prevent overshoot & wall crashes
        dist = math.hypot(tx - gx, ty - gy)
        if dist < 6.0:
            limit = min(limit, max(0.2, dist * 0.15))
        spd = math.hypot(vx, vy)
        if spd > limit:
            s = limit / spd
            vx *= s
            vy *= s
        self._cmd_vx  = vx
        self._cmd_vy  = vy
        self._cmd_alt = alt

