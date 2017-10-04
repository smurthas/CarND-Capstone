
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius):
        
	self.yaw_controller = YawController(wheel_base, steer_ratio, 0.2, max_lat_accel, max_steer_angle)

	self.vehicle_mass = vehicle_mass
	self.wheel_radius = wheel_radius
	self.acc_pid_controller = PID(1.0, 0.0, 0.0,-1.0, 1.0)
	self.acc_low_pass_filter = LowPassFilter(10.0, 1.0)
        
     

    def get_accel(self, prop_lin_vel, cur_lin_vel, time):
	throttle = 0
	brake = 0

	accel_raw = self.acc_pid_controller.step(prop_lin_vel - cur_lin_vel, time)
        accel = self.acc_low_pass_filter.filt(accel_raw)
   
	if prop_lin_vel < .1: 
	    brake = self.vehicle_mass * self.wheel_radius
	elif accel > 0.0:
	    throttle = accel
	else:
	    brake = abs(accel) * self.vehicle_mass * self.wheel_radius

	return throttle, brake



    def control(self, prop_lin_vel, prop_ang_vel, cur_lin_vel, time):
       
	steer = self.yaw_controller.get_steering(prop_lin_vel, prop_ang_vel, cur_lin_vel)

        throttle, brake = self.get_accel(prop_lin_vel, cur_lin_vel, time)
   
        return throttle, brake, steer

