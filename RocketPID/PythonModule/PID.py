class PID:
    def __init__(
        self,
        set_point,
        dt,
        kp,
        ki,
        kd,
        lower_limit,
        upper_limit,
        differential_on_measurement=True,
    ):
        """Initializes the PID controller with the set point and the PID constants.

        Args:
            set_point (float): The desired value for the process variable.
            dt (float): The time step between two updates of the controller.
            kp (float): The proportional gain.
            ki (float): The integral gain.
            kd (float): The derivative gain.
            lower_limit (float): The lower limit of the output in degrees.
            upper_limit (float): The upper limit of the output in degrees.
            differential_on_measurement (bool, optional): Whether to use the
            derivative on measurement or on error.
        """
        self.set_point = set_point
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lower_limit = lower_limit * 3.1416 / 180  # Convert to radians
        self.upper_limit = upper_limit * 3.1416 / 180  # Convert to radians
        self.differential_on_measurement = differential_on_measurement
        self.last_input = 0
        self.last_error = 0
        self.proportional = 0
        self.integrative = 0
        self.derivative = 0

    def __call__(self, input):
        """Computes the PID output based on the input."""
        error = self.set_point - input
        d_input = input - self.last_input
        d_error = error - self.last_error

        # Compute the proportional term
        self.proportional = self.kp * error

        # Compute integrative and derivative terms
        self.integrative += self.ki * error * self.dt
        self.integrative = self.anti_wind_up(
            self.integrative,
            self.lower_limit * 180 / 3.1416,  # Convert to degrees
            self.upper_limit * 180 / 3.1416,  # Convert to degrees
        )  # Avoid integrative windup (-lowerInput, +upperInput)

        if self.differential_on_measurement:
            self.derivative = -self.kd * d_input / self.dt
        else:
            self.derivative = self.kd * d_error / self.dt

        # Compute final output
        output = self.proportional + self.integrative + self.derivative
        output = self.clamp(output)

        # Keep track of state
        self.last_input = input
        self.last_error = error
        return output

    def clamp(self, value):
        """Clamps the value between the lower and upper limits."""
        if value > self.upper_limit:
            return self.upper_limit
        elif value < self.lower_limit:
            return self.lower_limit
        return value

    def anti_wind_up(self, integrative, lower_limit, upper_limit):
        """Clamps the integrative term between the lower and upper limits."""
        if integrative > upper_limit:
            return upper_limit
        if integrative < lower_limit:
            return lower_limit
        return integrative
