
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
  def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
          max_steer_angle, vehicle_mass, wheel_radius):
    self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    self.lin_vel_tol = 0.9;
    self.vehicle_mass = vehicle_mass
    self.wheel_radius = wheel_radius
    self.accel_pid_controller = PID(0.11, 0.0, -0.01, -1, 1)
    self.accel_low_pass_filter = LowPassFilter(10, 1)

    self.steer_factor = 1.0
    full_throttle_velocity = 8.94
    self.throttle_scale = full_throttle_velocity * full_throttle_velocity

    # initialize to zero for smooth initial accel from stop
    self.accel_low_pass_filter.filt(0)


  def get_accel(self, prop_lin_vel, cur_lin_vel, time):
    throttle = 0
    brake = 0

    accel_raw = self.accel_pid_controller.step(prop_lin_vel - cur_lin_vel, time)
    accel = self.accel_low_pass_filter.filt(accel_raw)
    if accel > 0:
      throttle = min(0.999, accel +
          (prop_lin_vel*prop_lin_vel/self.throttle_scale))
    else:
      brake = accel * -9.81 * self.vehicle_mass * self.wheel_radius

    return throttle, brake

  def control(self, prop_lin_vel, prop_ang_vel, cur_lin_vel, dbw_status, time):
    steer = self.steer_factor * self.yaw_controller.get_steering(prop_lin_vel, prop_ang_vel, cur_lin_vel)
    throttle, brake = self.get_accel(prop_lin_vel, cur_lin_vel, time)

    return throttle, brake, steer

