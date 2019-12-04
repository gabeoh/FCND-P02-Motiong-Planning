import numpy as np

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.vstack([p1, p2, p3])
    det = np.linalg.det(m)
    return abs(det) < epsilon
