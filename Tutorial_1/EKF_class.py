"""Extended Kalman Filter for tracking a vehicle in the tutorial dataset."""

import numpy as np


class EKF:
    """Track vehicle position, velocity, and angular velocity.

    The state vector follows this order:
        [x, y, vx, vy, omega]

    Position measurements are expected in the form ``[x, y]``.
    """

    def __init__(self, state, covariance):
        self.state = np.array(state, dtype=float)
        self.covariance = np.array(covariance, dtype=float)

        # Process noise: x acceleration, y acceleration, and angular acceleration.
        self.Q = np.diag([0.15**2, 0.15**2, 0.30**2])

        # Measurement noise for the observed x and y positions.
        self.R = np.diag([0.15**2, 0.15**2])

        # The sensor observes position only; velocity and angular velocity are hidden.
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0]], dtype=float)

    def state_prediction(self, delta_t):
        """Predict the next state using the constant-turn-rate model."""
        x, y, vx, vy, omega = self.state
        dt = delta_t
        self.state = np.array([x + vx*dt, y + vy*dt,
                               vx - omega*vy*dt,
                               vy + omega*vx*dt, omega])

    def compute_Fp(self, delta_t):
        """Return the state-transition Jacobian for the prediction step."""
        _, _, vx, vy, omega = self.state
        dt = delta_t
        return np.array([
            [1, 0, dt, 0, 0],
            [0, 1, 0, dt, 0],
            [0, 0, 1, -omega*dt, -vy*dt],
            [0, 0, omega*dt, 1, vx*dt],
            [0, 0, 0, 0, 1]
        ], dtype=float)

    def compute_Fdelta(self, delta_t):
        """Return the process-noise Jacobian for the prediction step."""
        _, _, vx, vy, _ = self.state
        dt = delta_t
        return np.array([
            [0.5*dt**2, 0, 0],
            [0, 0.5*dt**2, 0],
            [dt, 0, -0.5*vy*dt**2],
            [0, dt, 0.5*vx*dt**2],
            [0, 0, dt]
        ], dtype=float)

    def covariance_prediction(self, delta_t):
        """Propagate covariance and process uncertainty to the next step."""
        Fp = self.compute_Fp(delta_t)
        Fdelta = self.compute_Fdelta(delta_t)
        self.covariance = Fp @ self.covariance @ Fp.T + Fdelta @ self.Q @ Fdelta.T

        # Round-off can introduce tiny asymmetries into a covariance matrix.
        self.covariance = (self.covariance + self.covariance.T) / 2

    def correction(self, measurement):
        """Update the predicted state using a measured x-y position."""
        z = np.asarray(measurement, dtype=float).reshape(2)

        # Innovation: the difference between the measurement and prediction.
        innovation = z - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R

        # Solve for the Kalman gain without explicitly inverting S.
        K = np.linalg.solve(S, self.H @ self.covariance).T
        self.state = self.state + K @ innovation

        # Joseph form: numerically stable covariance update.
        I = np.eye(5)
        A = I - K @ self.H
        self.covariance = A @ self.covariance @ A.T + K @ self.R @ K.T

        # Keep the covariance symmetric after floating-point operations.
        self.covariance = (self.covariance + self.covariance.T) / 2
