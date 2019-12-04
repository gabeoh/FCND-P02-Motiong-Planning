import numpy as np

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))
