
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
  def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
          max_steer_angle, vehicle_mass, wheel_radius):
    self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    self.lin_vel_tol = 0.5;
    self.vehicle_mass = vehicle_mass
    self.wheel_radius = wheel_radius

  def control(self, prop_lin_vel, prop_ang_vel, cur_lin_vel, dbw_status):
    throttle = 0
    brake = 0
    steer = self.yaw_controller.get_steering(prop_lin_vel, prop_ang_vel, cur_lin_vel)

    # TODO: probably better to use a PID controller for throttle
    if cur_lin_vel < (prop_lin_vel - self.lin_vel_tol):
      throttle = 0.5
    elif cur_lin_vel > (prop_lin_vel + self.lin_vel_tol):
      brake = 0.3 * 9.81 * self.vehicle_mass * self.wheel_radius

    return throttle, brake, steer

