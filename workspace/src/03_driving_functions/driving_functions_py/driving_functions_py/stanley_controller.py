#
# MIT License
#
# Copyright (c) 2018-2022 Wisconsin Autonomous
#
# See https://wa.wisc.edu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

from wagrandprix_interfaces.wagrandprix_control_msg.msg import VehicleCommand


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        # Create publisher handles
        self.publisher_handles = {}

        self.publisher_handles["actuation_control"] = self.create_publisher(
            String, "actuation_control", 10
        )

        # Create subscriber handles
        self.subscriber_handles = {}

        self.subscriber_handles["vehicle_state"] = self.create_subscription(
            String, "vehicle_state", self.vehicle_state_callback, 10
        )

        # Periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def StanleyLateralController(self):
        """Advance the state of the controller by step

        Args:
            step (float): step size to update the controller by
        """
        pos = self._vehicle.get_pos()
        _, _, yaw = self._vehicle.get_rot().to_euler()

        self._sentinel = wa.WAVector(
            [
                self._dist * np.cos(yaw) + pos.x,
                self._dist * np.sin(yaw) + pos.y,
                0,
            ]
        )

        self._target = self._path.calc_closest_point(self._sentinel)
        print(self._target)

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = self._target - self._sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self._calc_sign(pos)

        # Calculate current error (magnitude)
        err = sign * err_vec.length

        # Estimate error derivative (backward FD approximation).
        self._errd = (err - self._err) / step

        # Calculate current error integral (trapezoidal rule).
        self._erri += (err + self._err) * step / 2

        # Cache new error
        self._err = err

        # Calculate the yaw error
        yawerror = math.atan(err/self._dist)

        # Calculate the steering value based of the Stanley Controller algorithm
        steering = yawerror + math.atan(((self._Kp) * (self._err)) / self._vehicle.get_pos_dt().length)

        # Clip the steering value to follow steering value bounds (max .436 radians/25 degrees)
        steering = np.clip(steering, -.436, .436)

        # Scale the steering value to -1.0 and 1.0 for steering value bounds
        self.steering = steering / 0.436

    def StanleyLongitudinalController(self):
        self._speed = self._vehicle.get_pos_dt().length

        # Calculate current error
        err = self._target_speed - self._speed

        # Estimate error derivative (backward FD approximation)
        self._errd = (err - self._err) / step

        # Calculate current error integral (trapezoidal rule).
        self._erri += (err + self._err) * step / 2

        # Cache new error
        self._err = err

        # Return PID output (steering value)
        throttle = np.clip(
            self._Kp * self._err + self._Ki * self._erri + self._Kd * self._errd, -1.0, 1.0
        )

        if throttle > 0:
            # Vehicle moving too slow
            self.braking = 0
            self.throttle = throttle
        elif self.throttle > self._throttle_threshold:
            # Vehicle moving too fast: reduce throttle
            self.braking = 0
            self.throttle += throttle
        else:
            # Vehicle moving too fast: apply brakes
            self.braking = -throttle
            self.throttle = 0


    def timer_callback(self):
        """
        Basic method for publishing. Get rid of or modify this method,
        but please include a description of the method as is done here.
        """

        # get the position, yaw, and target point from the subscriber and update that on self
        # self.pos ....
        


        self.StanleyLateralController

        self.StanleyLongitudinalController



        # publish the values found at self.steering, self.braking, self.throttle

        msg = VehicleCommand()

        msg.steering = self.steering
        msg.throttle = self.throttle
        msg.braking = self.braking


        # publish the message ---- not sure how to


        # msg = String()
        # msg.data = "Hello World: %d" % self.i

        # self.publisher_handles["actuation_control"].publish(msg)

        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1

    def vehicle_state_callback(self, msg):
        """
        Callback for the vehicle_state topic.
        """
        self.get_logger().info(f"Received {msg} on topic vehicle_state")


def main(args=None):
    rclpy.init(args=args)

    pure_pursuit = PurePursuit()

    rclpy.spin(pure_pursuit)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
