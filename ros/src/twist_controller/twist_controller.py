
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
FULL_BRAKE_SPEED = 0.1 


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius, decel_limit, brake_deadband):
        
	self.wheel_radius = wheel_radius
	self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit


	self.yaw_controller = YawController(wheel_base, steer_ratio, 1, max_lat_accel, max_steer_angle)
	self.acc_low_pass_filter = LowPassFilter(4., 1.)
        self.acc_pid_controller = PID(1.5, 0.001, 0.,decel_limit, 1.0)
     

    def get_accel(self, prop_lin_vel, cur_lin_vel, time):
	
	accel_raw = self.acc_pid_controller.step(prop_lin_vel - cur_lin_vel, time)
        accel = self.acc_low_pass_filter.filt(accel_raw)
   
	# Keep full brake if target velocity is almost 0
        if prop_lin_vel < FULL_BRAKE_SPEED:
            throttle = 0.0
            brake = abs(self.decel_limit) * self.vehicle_mass * self.wheel_radius
        else:
            if accel > 0.0:
                throttle = accel
                brake = 0.0
            else:
                throttle = 0.0
                deceleration = -accel

                # Do not brake if too small deceleration
                if deceleration < self.brake_deadband:
                    deceleration = 0.0

                # Compute brake torque, in Nm
                brake = deceleration * self.vehicle_mass * self.wheel_radius

	return throttle, brake



    def control(self, prop_lin_vel, prop_ang_vel, cur_lin_vel, time):
       
	steer = self.yaw_controller.get_steering(prop_lin_vel, prop_ang_vel, cur_lin_vel)

        throttle, brake = self.get_accel(prop_lin_vel, cur_lin_vel, time)
   
        return throttle, brake, steer

