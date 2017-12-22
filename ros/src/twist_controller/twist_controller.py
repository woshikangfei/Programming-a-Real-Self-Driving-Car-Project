
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class TwistController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
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
        self.lowpassfilter = LowPassFilter(0.5, 0.1)

    def control(self, proposed_twist, current_twist, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        proposed_linear_velocity = proposed_twist.linear.x
        proposed_angular_velocity = proposed_twist.angular.z
        current_linear_velocity = current_twist.linear.x
        current_angular_velocity = current_twist.angular.z

        delta_linear_velocity = proposed_linear_velocity - current_linear_velocity

        brake = 0.
        throttle = 0.
        if dbw_enabled:
            if proposed_linear_velocity > current_linear_velocity:
                if delta_linear_velocity / proposed_linear_velocity > 0.3:
                    throttle = self.max_throttle_percentage
                else:
                    throttle = max((delta_linear_velocity / proposed_linear_velocity) /
                                   0.3 * self.max_throttle_percentage, self.max_throttle_percentage)
            elif current_linear_velocity > 1:
                brake = 3250 * self.max_braking_percentage * -1
            else:
                brake = 3250 * 0.01

            steer = self.yaw_controller.get_steering(
                proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
            steer = max(-self.max_steer_angle, min(steer, self.max_steer_angle))

        else:
            throttle = 0.
            brake = 0.
            steer = 0.
        steer = self.lowpassfilter.filt(steer)
        rospy.loginfo("throttle:" + str(throttle) + "brake:" +
                      str(brake) + "steer" + str(steer))
        return throttle, brake, steer
