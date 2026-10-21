"""FireHawk Mk1 wildfire-spotting quadcopter controller.

Mission state machine:
  TAKEOFF -> PATROL -> INVESTIGATE -> ALERT -> RTB -> LAND -> DONE

The drone climbs to patrol altitude, flies a lawnmower search pattern over
the forest, scans its down-tilted camera for the colour signature of fire,
steers towards a detection to confirm it, hovers over the fire while
broadcasting its estimated GPS position (alert LED flashing), then returns
to the landing pad and lands.

Press M in the 3D window to toggle manual keyboard flight:
  arrows = forward/back/strafe, A/D = yaw, W/S = climb/descend
"""

import math
from controller import Robot, Keyboard

# ---------------- flight tuning constants ----------------
K_VERTICAL_THRUST = 72.0   # base propeller speed [rad/s] (hover ~72)
K_VERTICAL_OFFSET = 0.6    # altitude error offset for the cubic controller
K_VERTICAL_P = 3.0         # vertical PID gain
K_ROLL_P = 50.0            # roll stabilisation gain
K_PITCH_P = 30.0           # pitch stabilisation gain
K_YAW_P = 2.0              # yaw-to-heading gain
MAX_YAW_DISTURBANCE = 1.3
MAX_PITCH_DISTURBANCE = 2.2
MAX_ROLL_DISTURBANCE = 1.0
K_FWD = 0.45               # waypoint approach gain (forward)
K_FWD_D = 0.55             # forward velocity damping
K_LAT = 0.35               # lateral correction gain
K_LAT_D = 0.45             # lateral velocity damping

PATROL_ALTITUDE = 8.0      # [m]
WAYPOINT_RADIUS = 1.5      # [m] arrival threshold
LANDING_DESCENT_RATE = 0.7  # [m/s]

# lawnmower search pattern over the forest block east of the pad
PATROL_WAYPOINTS = [
    (18.0, 4.0),
    (18.0, -4.0),
    (6.0, -4.0),
    (6.0, -12.0),
    (18.0, -12.0),
    (22.0, -6.0),
    (10.0, 2.0),
]

# camera geometry (must match the .wbt file)
CAM_PITCH = 0.7854         # camera tilt below horizontal [rad]
CAM_FOV_H = 1.0            # horizontal field of view [rad]

# fire detection thresholds
FIRE_MIN_FRACTION = 0.0015  # fraction of sampled pixels that must look like fire
FIRE_CONFIRM_FRAMES = 3     # consecutive positive frames required


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class FireHawk(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # motors (front-right and rear-left spin clockwise -> negative command)
        self.motor_fl = self.getDevice("m_fl")
        self.motor_fr = self.getDevice("m_fr")
        self.motor_rl = self.getDevice("m_rl")
        self.motor_rr = self.getDevice("m_rr")
        for motor in (self.motor_fl, self.motor_fr, self.motor_rl, self.motor_rr):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        # sensors
        self.imu = self.getDevice("imu")
        self.imu.enable(self.timestep)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.compass = self.getDevice("compass")
        self.compass.enable(self.timestep)
        self.range_down = self.getDevice("range_down")
        self.range_down.enable(self.timestep)
        self.camera = self.getDevice("camera")
        self.camera.enable(4 * self.timestep)
        self.led = self.getDevice("alert_led")
        self.radio = self.getDevice("radio")  # telemetry downlink to base

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

        # mission state
        self.state = "TAKEOFF"
        self.manual = False
        self.waypoint_index = 0
        self.target_position = (0.0, 0.0)
        self.target_altitude = PATROL_ALTITUDE
        self.fire_streak = 0
        self.fire_estimate = None
        self.alert_started = None
        self.last_alert_print = -10.0
        self.last_position = None
        self.velocity = (0.0, 0.0)
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw_rate = 0.0
        self.last_status_print = 0.0
        self.last_telemetry = -1.0

    # ---------------- telemetry downlink ----------------
    def transmit(self, message):
        """Send a packet over the radio downlink to the ground station."""
        if self.radio is not None:
            self.radio.send(message.encode())

    # ---------------- perception ----------------
    def detect_fire(self):
        """Scan the camera image for fire-coloured pixels.

        Returns (fraction, cx_norm, cy_norm) where cx/cy are the detection
        centroid in normalised image coordinates (-0.5..0.5), or None.
        """
        image = self.camera.getImage()
        if image is None:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        step = 3
        count = 0
        sum_x = 0
        sum_y = 0
        for y in range(0, height, step):
            row = 4 * y * width
            for x in range(0, width, step):
                i = row + 4 * x
                b = image[i]
                g = image[i + 1]
                r = image[i + 2]
                if r > 160 and g < r - 50 and b < 110 and b < g + 40:
                    count += 1
                    sum_x += x
                    sum_y += y
        sampled = (width // step + 1) * (height // step + 1)
        fraction = count / float(sampled)
        if count == 0:
            return None
        cx_norm = (sum_x / count) / width - 0.5
        cy_norm = (sum_y / count) / height - 0.5
        return (fraction, cx_norm, cy_norm)

    def estimate_fire_position(self, cx_norm, cy_norm, x, y, altitude, yaw):
        """Project the detection centroid onto the ground plane,
        compensating for the drone's own pitch tilt."""
        fov_v = CAM_FOV_H * self.camera.getHeight() / self.camera.getWidth()
        depression = CAM_PITCH + cy_norm * fov_v + self.current_pitch
        if depression < 0.08:  # nearly horizontal: unreliable
            return None
        ground_distance = altitude / math.tan(depression)
        bearing = yaw - cx_norm * CAM_FOV_H
        fire_x = x + ground_distance * math.cos(bearing)
        fire_y = y + ground_distance * math.sin(bearing)
        return (fire_x, fire_y)

    # ---------------- guidance ----------------
    def navigation_disturbances(self, x, y, yaw):
        """P-D guidance towards self.target_position.

        Returns (yaw_disturbance, pitch_disturbance, roll_disturbance,
                 distance_to_target).
        """
        dx = self.target_position[0] - x
        dy = self.target_position[1] - y
        distance = math.hypot(dx, dy)

        # errors in the body frame (x forward, y left)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_error = cos_yaw * dx + sin_yaw * dy
        lateral_error = -sin_yaw * dx + cos_yaw * dy
        forward_velocity = cos_yaw * self.velocity[0] + sin_yaw * self.velocity[1]
        lateral_velocity = -sin_yaw * self.velocity[0] + cos_yaw * self.velocity[1]

        if distance > 1.0:
            target_heading = math.atan2(dy, dx)
            yaw_error = wrap_angle(target_heading - yaw)
        else:
            yaw_error = 0.0
        yaw_disturbance = clamp(K_YAW_P * yaw_error,
                                -MAX_YAW_DISTURBANCE, MAX_YAW_DISTURBANCE)

        # only push forward when roughly facing the target
        if abs(yaw_error) < 0.5:
            pitch_disturbance = clamp(-K_FWD * forward_error + K_FWD_D * forward_velocity,
                                      -MAX_PITCH_DISTURBANCE, MAX_PITCH_DISTURBANCE)
        else:
            pitch_disturbance = clamp(K_FWD_D * forward_velocity,
                                      -MAX_PITCH_DISTURBANCE, MAX_PITCH_DISTURBANCE)
        roll_disturbance = clamp(K_LAT * lateral_error - K_LAT_D * lateral_velocity,
                                 -MAX_ROLL_DISTURBANCE, MAX_ROLL_DISTURBANCE)
        return yaw_disturbance, pitch_disturbance, roll_disturbance, distance

    # ---------------- mission logic ----------------
    def update_mission(self, x, y, altitude, yaw, now):
        yaw_d = pitch_d = roll_d = 0.0

        # only scan for fire in level, steady flight: estimates taken while
        # banking through a turn project onto the wrong ground position
        stable = (abs(self.current_roll) < 0.15
                  and abs(self.current_yaw_rate) < 0.3)
        detection = None
        if self.state in ("PATROL", "INVESTIGATE") and stable:
            detection = self.detect_fire()
            if detection and detection[0] >= FIRE_MIN_FRACTION:
                self.fire_streak += 1
                estimate = self.estimate_fire_position(
                    detection[1], detection[2], x, y, altitude, yaw)
                if estimate:
                    self.fire_estimate = estimate
            else:
                self.fire_streak = max(0, self.fire_streak - 1)

        if self.state == "TAKEOFF":
            self.target_position = (x, y) if self.last_position is None else self.target_position
            if altitude > PATROL_ALTITUDE - 0.4:
                self.state = "PATROL"
                self.target_position = PATROL_WAYPOINTS[0]
                print("[FireHawk] Takeoff complete, starting patrol at "
                      f"{PATROL_ALTITUDE:.0f} m")

        elif self.state == "PATROL":
            yaw_d, pitch_d, roll_d, distance = self.navigation_disturbances(x, y, yaw)
            if distance < WAYPOINT_RADIUS:
                self.waypoint_index = (self.waypoint_index + 1) % len(PATROL_WAYPOINTS)
                self.target_position = PATROL_WAYPOINTS[self.waypoint_index]
                print(f"[FireHawk] Waypoint reached, heading to "
                      f"{self.target_position}")
            if self.fire_streak >= FIRE_CONFIRM_FRAMES and self.fire_estimate:
                self.state = "INVESTIGATE"
                self.target_position = self.fire_estimate
                print(f"[FireHawk] FIRE SIGNATURE DETECTED near "
                      f"({self.fire_estimate[0]:.1f}, {self.fire_estimate[1]:.1f})"
                      " - investigating")

        elif self.state == "INVESTIGATE":
            if self.fire_estimate:
                self.target_position = self.fire_estimate
            yaw_d, pitch_d, roll_d, distance = self.navigation_disturbances(x, y, yaw)
            if distance < 2.0:
                self.state = "ALERT"
                self.alert_started = now
                print("[FireHawk] FIRE CONFIRMED - holding position and alerting")

        elif self.state == "ALERT":
            yaw_d, pitch_d, roll_d, _ = self.navigation_disturbances(x, y, yaw)
            self.led.set(1 if int(now * 4) % 2 == 0 else 0)
            if now - self.last_alert_print >= 1.0 and self.fire_estimate:
                self.last_alert_print = now
                print(f"[FireHawk] *** WILDFIRE ALERT *** estimated position "
                      f"x={self.fire_estimate[0]:.1f} m, y={self.fire_estimate[1]:.1f} m "
                      f"(drone GPS {x:.1f}, {y:.1f}, alt {altitude:.1f} m)")
                self.transmit(f"ALERT|{self.fire_estimate[0]:.1f}"
                              f"|{self.fire_estimate[1]:.1f}"
                              f"|{x:.1f}|{y:.1f}|{altitude:.1f}")
            if now - self.alert_started > 8.0:
                self.led.set(0)
                self.state = "RTB"
                self.target_position = (0.0, 0.0)
                print("[FireHawk] Alert relayed, returning to base")

        elif self.state == "RTB":
            yaw_d, pitch_d, roll_d, distance = self.navigation_disturbances(x, y, yaw)
            if distance < 0.8:
                self.state = "LAND"
                print("[FireHawk] Over landing pad, descending")

        elif self.state == "LAND":
            yaw_d, pitch_d, roll_d, _ = self.navigation_disturbances(x, y, yaw)
            # let the target sink below ground level so the steady-state
            # altitude offset cannot leave the drone hovering above the pad
            self.target_altitude = max(-3.0,
                                       self.target_altitude - LANDING_DESCENT_RATE
                                       * self.timestep / 1000.0)
            range_reading = self.range_down.getValue()
            if altitude < 0.18 or (0.0 < range_reading < 0.12):
                self.state = "DONE"
                print("[FireHawk] Touchdown. MISSION_COMPLETE")

        return yaw_d, pitch_d, roll_d

    # ---------------- manual override ----------------
    def keyboard_disturbances(self):
        yaw_d = pitch_d = roll_d = 0.0
        key = self.keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                pitch_d = -2.0
            elif key == Keyboard.DOWN:
                pitch_d = 2.0
            elif key == Keyboard.RIGHT:
                roll_d = -1.0
            elif key == Keyboard.LEFT:
                roll_d = 1.0
            elif key in (ord("A"), ord("a")):
                yaw_d = 1.3
            elif key in (ord("D"), ord("d")):
                yaw_d = -1.3
            elif key in (ord("W"), ord("w")):
                self.target_altitude += 0.05
            elif key in (ord("S"), ord("s")):
                self.target_altitude = max(0.5, self.target_altitude - 0.05)
            elif key in (ord("M"), ord("m")):
                self.manual = not self.manual
                mode = "MANUAL" if self.manual else "AUTONOMOUS"
                print(f"[FireHawk] Switched to {mode} mode")
            key = self.keyboard.getKey()
        return yaw_d, pitch_d, roll_d

    # ---------------- main loop ----------------
    def run(self):
        print("[FireHawk] Controller started. Press M for manual keyboard mode "
              "(arrows, A/D yaw, W/S altitude).")
        dt = self.timestep / 1000.0
        while self.step(self.timestep) != -1:
            now = self.getTime()
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            self.current_pitch = pitch
            self.current_roll = roll
            x, y, altitude = self.gps.getValues()
            roll_rate, pitch_rate, yaw_rate = self.gyro.getValues()
            self.current_yaw_rate = yaw_rate

            if now - self.last_status_print >= 5.0:
                self.last_status_print = now
                print(f"[FireHawk] t={now:5.1f}s state={self.state:11s} "
                      f"pos=({x:5.1f},{y:5.1f}) alt={altitude:4.1f} m")

            # stream GPS telemetry to the ground station at 4 Hz
            if now - self.last_telemetry >= 0.25:
                self.last_telemetry = now
                self.transmit(f"TLM|{now:.1f}|{self.state}"
                              f"|{x:.1f}|{y:.1f}|{altitude:.1f}")

            if self.last_position is not None:
                self.velocity = ((x - self.last_position[0]) / dt,
                                 (y - self.last_position[1]) / dt)
            self.last_position = (x, y)

            manual_input = self.keyboard_disturbances()
            if self.manual:
                yaw_d, pitch_d, roll_d = manual_input
            elif self.state == "DONE":
                for motor in (self.motor_fl, self.motor_fr,
                              self.motor_rl, self.motor_rr):
                    motor.setVelocity(0.0)
                continue
            else:
                yaw_d, pitch_d, roll_d = self.update_mission(x, y, altitude, yaw, now)

            # ---- attitude stabilisation (PID, Mavic-style mixing) ----
            roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_rate + roll_d
            pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_rate + pitch_d
            altitude_error = clamp(self.target_altitude - altitude + K_VERTICAL_OFFSET,
                                   -1.0, 1.0)
            vertical_input = K_VERTICAL_P * altitude_error ** 3

            base = K_VERTICAL_THRUST + vertical_input
            motor_fl_speed = base - roll_input + pitch_input - yaw_d
            motor_fr_speed = base + roll_input + pitch_input + yaw_d
            motor_rl_speed = base - roll_input - pitch_input + yaw_d
            motor_rr_speed = base + roll_input - pitch_input - yaw_d

            self.motor_fl.setVelocity(clamp(motor_fl_speed, 0.0, 500.0))
            self.motor_fr.setVelocity(-clamp(motor_fr_speed, 0.0, 500.0))
            self.motor_rl.setVelocity(-clamp(motor_rl_speed, 0.0, 500.0))
            self.motor_rr.setVelocity(clamp(motor_rr_speed, 0.0, 500.0))


if __name__ == "__main__":
    FireHawk().run()
