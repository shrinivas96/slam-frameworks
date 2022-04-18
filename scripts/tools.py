import numpy as np

def t2v(A):
    v = np.zeros((3, 1))
    v[0:2, 0] = A[0:2, 2]
    v[2, 0] = np.arctan2(A[1, 0], A[0, 0])

    return v


def v2t(v):
    c = np.cos(v[2])
    s = np.sin(v[2])
    A = np.array([[c,  -s,  v[0]],
                  [s,   c,  v[1]],
                  [0,   0,  1]])

    return A


def str2double(S):
    if isinstance(S, list):
        X = np.array([], dtype=np.float64)
        for item in S:
            try:
                X = np.append(X, float(item))
            except Exception as e:
                X = np.append(X, np.nan)
    else:
        try:
            X = float(S)
        except:
            X = np.nan
    
    return X
