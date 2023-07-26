import numpy as np

def kabsch_umeyama(A, B):
    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = ((A - EA).T @ (B - EB)) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(VT))
    S = np.diag([1] * (m - 1) + [d])

    R = U @ S @ VT
    c = VarA / np.trace(np.diag(D) @ S)
    t = EA - c * R @ EB

    return R, c, t


if __name__ == "__main__": 
	map_points = np.array([[-10.15, -3.71],
			[-4.55, 10.74],
			[9.91, 4.79],
			[4.6, -10.6]])

	true_points = np.array([[15, 0],
			[0, 15],
			[-15, 0],
			[0, -15]])

	R, c, t = kabsch_umeyama(map_points, true_points)
	np.save('R_ku.npy', R)
	np.save('T_ku.npy', t)
	print(R, t)