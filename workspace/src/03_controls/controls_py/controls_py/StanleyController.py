# Import the WASimulator
import wa_simulator as wa

# Other imports
import numpy as np

# New import
import math


class StanleyController(wa.WAController):
    """Stanley Controller that contains a lateral. PID is used for the longitudinal controller

    Uses the lateral controller for steering and longitudinal controller throttle/braking

    Args:
        system (WASystem): The system used to manage the simulation
        vehicle (WAVehicle): The vehicle to grab the state from
        vehicle_inputs (WAVehicleInputs): The vehicle inputs
        path (WAPath): The path to follow
        lat_controller (StanleyLateralController, optional): Lateral controller for steering. Defaults to None. Will create one if not passed.
        long_controller (StanleyLongitudinalController, optional): Longitudinal controller for throttle/braking. Defaults to None. Will create one if not passed.
    """

    def __init__(self, system, VehicleState, target_point, vehicle_inputs: wa.WAVehicleInputs, lat_controller: 'StanleyLateralController' = None, long_controller: 'StanleyLongitudinalController' = None):
        super().__init__(system, vehicle_inputs)
        self.VehicleState = VehicleState
        self.target_point = target_point

        # Lateral controller (steering)
        if lat_controller is None:
            lat_controller = StanleyLateralController(VehicleState, target_point, vehicle_inputs)
            lat_controller.set_gains(Kp=0.4, Ki=0, Kd=0)
            lat_controller.set_lookahead_distance(dist=5)
        self._lat_controller = lat_controller

        if long_controller is None:
            # Longitudinal controller (throttle and braking)
            long_controller = StanleyLongitudinalController(VehicleState, target_point, vehicle_inputs)
            long_controller.set_gains(Kp=0.4, Ki=0, Kd=0)
            long_controller.set_target_speed(speed=7.0)
        self._long_controller = long_controller

        self._target_steering = 0
        self._target_throttle = 0
        self._target_braking = 0

        self._steering_delta = 1.0 / 50
        self._throttle_delta = 1.0 / 50
        self._braking_delta = 1.0 / 50

        self._steering_gain = 4.0
        self._throttle_gain = 0.25
        self._braking_gain = 4.0

    def set_delta(self, steering_delta: float, throttle_delta: float, braking_delta: float):
        """Set the delta values

        In this controller, delta values are the maximum change in steering/throttle/braking that can occur in a single
        update. For example, if the vehicle is not moving and the underyling longitudinal controller wanted to have the vehicle
        go pedal-to-the-metal, instead of going from 0% throttle to 100%, it will go from 0% to (1. / throttle_delta / 100)%. This
        means if the throttle_delta was 0.01, then the throttle would be placed at 10%. Same principle is true for braking and 
        steering values during the update.

        Args:
            steering_delta (float): max steering delta
            throttle_delta (float): max throttle delta
            braking_delta (float): max braking delta
        """
        self._steering_delta = steering_delta
        self._throttle_delta = throttle_delta
        self._braking_delta = braking_delta

    def set_gains(self, steering_gain: float, throttle_gain: float, braking_gain: float):
        """Set the gain values

        The gains are multiplied to each "derivative" calculated during each time step. This controller works
        by using the target input value provided by the underlying lateral and longitudinal controllers and
        "stepping" towards that value in increments. See :meth:`~set_delta` to understand the `delta` values.
        On each step, the difference between the target and current values are multiplied by the gain. This is
        because the difference may be very small or very large, so a high/low gain can offset this and provide
        better results.

        Args:
            steering_gain (float): steering gain
            throttle_gain (float): throttle gain
            braking_gain (float): braking gain
        """
        self._steering_gain = steering_gain
        self._throttle_gain = throttle_gain
        self._braking_gain = braking_gain

    def get_long_controller(self) -> 'StanleyLongitudinalController':
        """Get the underyling longitudinal controller

        Returns:
            StanleyLongitudinalController: The underyling longitudinal (speed/throttle) controller
        """
        return self._long_controller

    def synchronize(self, time: float):
        """Synchronize the lateral and longitudinal controllers at the specified time

        Args:
            time (float): the time at which the controller should synchronize all modules
        """

        self._lat_controller.synchronize(time)
        self._long_controller.synchronize(time)

    def advance(self, step: float):
        """Advance the state of the lateral and longitudinal controllers

        Args:
            step (float): the time step at which the controller should be advanced
        """
        self._lat_controller.target_point = self.target_point
        self._long_controller.target_point = self.target_point

        self._lat_controller.advance()
        self._long_controller.advance(step)

        self._target_steering = self._lat_controller.steering
        self._target_throttle = self._long_controller.throttle
        self._target_braking = self._long_controller.braking

        # Integrate dynamics, taking as many steps as required to reach the value 'step'
        t = 0.0
        while t < step:    # this is used by the simulation and is usually like 1 ms
            # h = min(self._system.step_size, step - t)
            h = 0.01

            steering_deriv = self._steering_gain * (self._target_steering - self.steering)  # noqa
            throttle_deriv = self._throttle_gain * (self._target_throttle - self.throttle)  # noqa
            braking_deriv = self._braking_gain * (self._target_braking - self.braking)  # noqa

            self.steering += min(h * steering_deriv, self._steering_delta, key=abs)  # noqa
            self.throttle += min(h * throttle_deriv, self._throttle_delta, key=abs)  # noqa
            self.braking += min(h * braking_deriv, self._braking_delta, key=abs)  # noqa

            t += h

    def get_target_pos(self) -> wa.WAVector:

        """Get the position of the target point of the lateral controller

        Returns:
            WAVector: The position of the target point
        """
        return self._lat_controller._target

    def get_sentinel_pos(self) -> wa.WAVector:
        """Get the position of the sentinel point of the lateral controller

        Returns:
            WAVector: The position of the sentinel point
        """
        return self._lat_controller._sentinel


class StanleyLateralController():
    """Lateral (steering) controller which minimizes error using the Stanley Controller algorithm

    Args:
        system (WASystem): The system used to manage the simulation
        vehicle (WAVehicle): the vehicle who has dynamics
        vehicle_inputs (WAVehicleInputs): The vehicle inputs
        path (WAPath): the path the vehicle is attempting to follow
    """

    def __init__(self, vehicle_state, target_point, vehicle_inputs):
        self._Kp = 0
        self._Ki = 0
        self._Kd = 0

        self._dist = 0
        self._target = wa.WAVector([0, 0, 0])
        self._sentinel = wa.WAVector([0, 0, 0])

        self._err = 0
        self._errd = 0
        self._erri = 0

        self.VehicleState = vehicle_state
        self.target_point = target_point

    def set_gains(self, Kp: float, Ki: float, Kd: float):
        """Set the gains

        Args:
            Kp (float): new proportional gain
            Ki (float): new integral gain
            Kd (float): new derivative gain
        """
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd

    def set_lookahead_distance(self, dist: float):
        """Set the lookahead distance

        To track the path, this controller aims for a lookahead point at the specified distance
        directly infront of the vehicle. That way, the tracking will be much smoother.

        Args:
            dist (float): new lookahead distance
        """
        self._dist = dist

    def synchronize(self, time: float):
        """Synchronize the lateral and longitudinal controllers at the specified time

        Args:
            time (float): the time at which the controller should synchronize all modules
        """

        pass

    def advance(self):
        """Advance the state of the controller by step

        Args:
            step (float): step size to update the controller by
        """
        pos = wa.WAVector([self.VehicleState.pose.position.x, self.VehicleState.pose.position.y, self.VehicleState.pose.position.z])
        # print(type(pos))
        temp = [self.VehicleState.pose.orientation.x, self.VehicleState.pose.orientation.y, self.VehicleState.pose.orientation.z, self.VehicleState.pose.orientation.w]
        # print(temp)
        _, _, yaw = wa.WAQuaternion(temp).to_euler()

        self._sentinel = wa.WAVector(
            [
                self._dist * np.cos(yaw) + pos.x,
                self._dist * np.sin(yaw) + pos.y,
                0,
            ]
        )

        self._target = self.target_point    #NOT RIGHT place holder

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = self._target - self._sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self._calc_sign(pos)

        # print("sign", sign)
        # Calculate current error (magnitude)
        err = sign * err_vec.length

        # # Estimate error derivative (backward FD approximation).
        # self._errd = (err - self._err) / step

        # # Calculate current error integral (trapezoidal rule).
        # self._erri += (err + self._err) * step / 2

        # Cache new error
        self._err = err

        # print("err", err)

        # Calculate the yaw error
        yawerror = math.atan(err/self._dist)

        # print(yawerror)

        # Calculate the steering value based of the Stanley Controller algorithm
        velocity = wa.WAVector([self.VehicleState.twist.linear.x, self.VehicleState.twist.linear.y, self.VehicleState.twist.linear.z])
        steering = yawerror + math.atan(((self._Kp) * (self._err)) / velocity.length)

        # Clip the steering value to follow steering value bounds (max .436 radians/25 degrees)
        steering = np.clip(steering, -.436, .436)

        # Scale the steering value to -1.0 and 1.0 for steering value bounds
        self.steering = steering / 0.436


    def _calc_sign(self, pos: wa.WAVector) -> int:
        """Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).

        Args:
            pos (WAVector): Vehicle position

        Returns:
            int: the sign indicating direction of the state and the sentinel on the path (-1 for left or 1 for right)
        """
        sentinel_vec = self._sentinel - pos
        target_vec = self._target - pos

        temp = np.dot(np.cross(sentinel_vec, target_vec),
                      wa.WAVector([0, 0, 1]))


        return int(temp > 0) - int(temp < 0)


class StanleyLongitudinalController():
    """Longitudinal (throttle, braking) controller which minimizes error using a PID

    Args:
        system (WASystem): The system used to manage the simulation
        vehicle_inputs (WAVehicleInputs): The vehicle inputs
        vehicle (WAVehicle): the vehicle who has dynamics
    """

    def __init__(self, vehicle_state, target_point, vehicle_inputs):

        self._Kp = 0
        self._Ki = 0
        self._Kd = 0

        self._err = 0
        self._errd = 0
        self._erri = 0

        self._speed = 0
        self._target_speed = 0

        # self.braking = 0
        # self.throttle = 0

        self._throttle_threshold = 0.2

        self.VehicleState = vehicle_state
        self.target_point = target_point


    def set_gains(self, Kp: float, Ki: float, Kd: float):
        """Set the gains

        Args:
            Kp (float): new proportional gain
            Ki (float): new integral gain
            Kd (float): new derivative gain
        """
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd

    def set_target_speed(self, speed: float):
        """Set the target speed for the controller

        Args:
            speed (float): the new target speed
        """
        self._target_speed = speed

    def synchronize(self, time: float):
        """Synchronize the lateral and longitudinal controllers at the specified time

        Args:
            time (float): the time at which the controller should synchronize all modules
        """

        pass

    def advance(self, step):
        """Advance the state of the controller by step

        Args:
            step (float): step size to update the controller by
        """
        velocity = wa.WAVector([self.VehicleState.twist.linear.x, self.VehicleState.twist.linear.y, self.VehicleState.twist.linear.z])

        self._speed = np.linalg.norm(velocity)
        

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