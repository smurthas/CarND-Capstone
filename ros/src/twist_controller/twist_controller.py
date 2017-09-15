
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
  def __init__(self, *args, **kwargs):
    # TODO: Implement
    self.lin_vel_tol = 0.5;
    pass

  def control(self, prop_lin_vel, prop_ang_vel, cur_lin_vel, dbw_status):
    throttle = 0
    brake = 0
    steer = 0
    if cur_lin_vel < (prop_lin_vel - self.lin_vel_tol):
      throttle = 0.5
    elif cur_lin_vel > (prop_lin_vel + self.lin_vel_tol):
      brake = 0.5
    return throttle, brake, steer
