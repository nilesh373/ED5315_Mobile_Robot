import numpy as np

class EKF:
    def __init__(self, state, covariance):
        self.state = np.array(state, dtype=float)
        self.covariance = np.array(covariance, dtype=float)
        self.Q = np.diag([0.15**2, 0.15**2, 0.30**2])
        self.R = np.diag([0.15**2, 0.15**2])
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0]], dtype=float)

    def state_prediction(self, delta_t):
        x, y, vx, vy, omega = self.state
        dt = delta_t
        self.state = np.array([x + vx*dt, y + vy*dt,
                               vx - omega*vy*dt,
                               vy + omega*vx*dt, omega])

    def compute_Fp(self, delta_t):
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
        Fp = self.compute_Fp(delta_t)
        Fdelta = self.compute_Fdelta(delta_t)
        self.covariance = Fp @ self.covariance @ Fp.T + Fdelta @ self.Q @ Fdelta.T
        self.covariance = (self.covariance + self.covariance.T) / 2

    def correction(self, measurement):
        z = np.asarray(measurement, dtype=float).reshape(2)
        innovation = z - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R
        K = np.linalg.solve(S, self.H @ self.covariance).T
        self.state = self.state + K @ innovation
        I = np.eye(5)
        A = I - K @ self.H
        self.covariance = A @ self.covariance @ A.T + K @ self.R @ K.T
        self.covariance = (self.covariance + self.covariance.T) / 2
