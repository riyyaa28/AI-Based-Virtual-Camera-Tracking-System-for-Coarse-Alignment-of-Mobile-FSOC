"""Pointing-control helpers shared by the desktop and browser simulators."""


def compute_delta(tracked_pos, frame_center, gain=0.3):
    """Original proportional pixel controller retained for the PyQt client."""
    tx, ty = tracked_pos
    cx, cy = frame_center
    dx = (tx - cx) * gain
    dy = (ty - cy) * gain
    return dx, dy


class PIDPanTiltController:
    """PID controller with integral clamping for gimbal angle commands.

    Errors and output are in degrees.  The physical gimbal is modelled by the
    simulation separately, which means this controller only determines the
    commanded angle and cannot make the camera jump instantaneously.
    """

    def __init__(self, kp=1.45, ki=0.06, kd=0.20, integral_limit=18.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self._integral = [0.0, 0.0]
        self._previous_error = [0.0, 0.0]

    def reset(self):
        self._integral = [0.0, 0.0]
        self._previous_error = [0.0, 0.0]

    def update(self, pan_error, tilt_error, dt):
        dt = max(float(dt), 1e-4)
        errors = (float(pan_error), float(tilt_error))
        output = []
        for index, error in enumerate(errors):
            self._integral[index] = max(
                -self.integral_limit,
                min(self.integral_limit, self._integral[index] + error * dt),
            )
            derivative = (error - self._previous_error[index]) / dt
            output.append(
                self.kp * error
                + self.ki * self._integral[index]
                + self.kd * derivative
            )
            self._previous_error[index] = error
        return tuple(output)
