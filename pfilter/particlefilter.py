import numpy as np
import matplotlib.pyplot as plt

#############
# Constants #
#############
NUM_PARTICLES = 100
LANDMARKS = np.array([[5, 10], [10, 5], [15, 15]])
DT = 1.0
RANGE_NOISE = 0.5
BEARING_NOISE = np.deg2rad(5)
MOTION_NOISE = np.diag([0.2, 0.2, np.deg2rad(5)]) ** 2

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Motion model
def motion_model(state, control):
    x, y, theta = state
    v, omega = control
    theta += omega * DT + np.random.normal(0, np.sqrt(MOTION_NOISE[2, 2]))
    x += v * np.cos(theta) * DT + np.random.normal(0, np.sqrt(MOTION_NOISE[0, 0]))
    y += v * np.sin(theta) * DT + np.random.normal(0, np.sqrt(MOTION_NOISE[1, 1]))
    return np.array([x, y, normalize_angle(theta)])

# Sensor model: range and bearing
def sense(state, landmark):
    dx, dy = landmark - state[:2]
    r = np.hypot(dx, dy) + np.random.normal(0, RANGE_NOISE)
    b = normalize_angle(np.arctan2(dy, dx) - state[2] + np.random.normal(0, BEARING_NOISE))
    return np.array([r, b])

class LandmarkEKF:
    def __init__(self, mean, cov):
        self.mean = mean
        self.cov = cov

    def update(self, z, pose):
        x, y, theta = pose
        dx, dy = self.mean[0] - x, self.mean[1] - y
        q = dx**2 + dy**2
        z_hat = np.array([np.sqrt(q), normalize_angle(np.arctan2(dy, dx) - theta)])

        H = np.array([
            [dx / np.sqrt(q), dy / np.sqrt(q)],
            [-dy / q, dx / q]
        ])
        R = np.diag([RANGE_NOISE ** 2, BEARING_NOISE ** 2])
        S = H @ self.cov @ H.T + R
        K = self.cov @ H.T @ np.linalg.inv(S)
        innovation = z - z_hat
        innovation[1] = normalize_angle(innovation[1])

        self.mean += K @ innovation
        self.cov = (np.eye(2) - K @ H) @ self.cov

class Particle:
    def __init__(self, pose):
        self.pose = pose
        self.weight = 1.0
        self.landmarks = [None] * len(LANDMARKS)

    def motion_update(self, control):
        self.pose = motion_model(self.pose, control)

    def sensor_update(self, measurements):
        for i, z in enumerate(measurements):
            if self.landmarks[i] is None:
                # Initialize landmark using inverse measurement model
                r, b = z
                lx = self.pose[0] + r * np.cos(b + self.pose[2])
                ly = self.pose[1] + r * np.sin(b + self.pose[2])
                self.landmarks[i] = LandmarkEKF(np.array([lx, ly]), np.eye(2) * 1.0)
            else:
                self.landmarks[i].update(z, self.pose)

        # Update weight based on likelihood (simplified)
        self.weight *= self.calculate_likelihood(measurements)

    def calculate_likelihood(self, measurements):
        likelihood = 1.0
        for i, z in enumerate(measurements):
            if self.landmarks[i] is None:
                continue
            landmark = self.landmarks[i]
            dx, dy = landmark.mean - self.pose[:2]
            q = dx**2 + dy**2
            z_hat = np.array([np.sqrt(q), normalize_angle(np.arctan2(dy, dx) - self.pose[2])])
            diff = z - z_hat
            diff[1] = normalize_angle(diff[1])
            cov = landmark.cov + np.diag([RANGE_NOISE ** 2, BEARING_NOISE ** 2])
            det = np.linalg.det(cov)
            if det <= 0:
                continue
            exponent = diff.T @ np.linalg.inv(cov) @ diff
            likelihood *= np.exp(-0.5 * exponent) / np.sqrt((2 * np.pi)**2 * det)
        return likelihood

def resample(particles):
    weights = np.array([p.weight for p in particles])
    weights /= np.sum(weights)
    indices = np.random.choice(len(particles), len(particles), p=weights)
    new_particles = []
    for i in indices:
        p = particles[i]
        new_p = Particle(p.pose.copy())
        new_p.landmarks = [None if lm is None else LandmarkEKF(lm.mean.copy(), lm.cov.copy()) for lm in p.landmarks]
        new_particles.append(new_p)
    for p in new_particles:
        p.weight = 1.0
    return new_particles

def get_best_estimate(particles):
    poses = np.array([p.pose for p in particles])
    mean_pose = np.mean(poses, axis=0)
    return mean_pose
