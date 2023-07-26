class KalmanFilter:
    def __init__(self, initial_state, initial_uncertainty, measurement_uncertainty, process_noise):
        self.state = initial_state
        self.uncertainty = initial_uncertainty
        self.measurement_uncertainty = measurement_uncertainty
        self.process_noise = process_noise

    def predict(self):
        self.uncertainty += self.process_noise

    def update(self, measurement):
        kalman_gain = self.uncertainty / (self.uncertainty + self.measurement_uncertainty)
        self.state += kalman_gain * (measurement - self.state)
        self.uncertainty *= (1 - kalman_gain)
