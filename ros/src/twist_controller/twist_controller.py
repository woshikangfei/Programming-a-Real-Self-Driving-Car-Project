
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class TwistController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # Define and initialize each params will be used in control
        self.wheel_base = kwargs['wheel_base']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        steer_ratio = kwargs['steer_ratio']
        min_speed = 0.
        max_lat_accel = kwargs['max_lat_accel']
        accel_limit = kwargs['accel_limit']
        decel_limit = kwargs['decel_limit']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.brake_deadband = kwargs['brake_deadband']
        self.max_throttle_percentage = kwargs['max_throttle_percentage']
        self.max_braking_percentage = kwargs['max_braking_percentage']

        self.yaw_controller = YawController(
            self.wheel_base, steer_ratio, min_speed, max_lat_accel, self.max_steer_angle)

        self.current_linear_velocity_lowpassfilter = LowPassFilter(150, 75)
        self.steer_lowpassfilter = LowPassFilter(150, 75)
        self.throttle_pid = PID(0.1, 0.007, 0)

        self.throttle = 0.
        self.brake = 0.
        self.steer = 0.

    def control(self, proposed_twist, current_twist, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        proposed_linear_velocity = proposed_twist.linear.x
        proposed_angular_velocity = proposed_twist.angular.z
        current_linear_velocity = current_twist.linear.x
        current_angular_velocity = current_twist.angular.z

        delta_linear_velocity = proposed_linear_velocity - current_linear_velocity

        if dbw_enabled:
            if proposed_linear_velocity > current_linear_velocity:
                if delta_linear_velocity / proposed_linear_velocity > 0.3:
                    self.throttle = self.max_throttle_percentage
                else:
                    self.throttle = max((delta_linear_velocity / proposed_linear_velocity) /
                                   0.3 * self.max_throttle_percentage, self.max_throttle_percentage)
            elif current_linear_velocity > 1:
                self.brake = 3250 * self.max_braking_percentage * -1
            else:
                self.brake = 3250 * 0.01

            self.steer = self.yaw_controller.get_steering(
                proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
            self.steer = max(-self.max_steer_angle, min(self.steer, self.max_steer_angle))


        self.steer = self.lowpassfilter.filt(self.steer)
        rospy.loginfo("throttle:" + str(self.throttle) + "brake:" +
                      str(self.brake) + "steer" + str(self.steer))

        # Return throttle, brake, steer
        return self.throttle, self.brake, self.steer
