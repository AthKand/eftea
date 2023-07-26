# -*- coding: utf-8 -*-
# @Author: ath
# @Date:   2023-07-18 16:14:48
# @Last Modified by:   ath
# @Last Modified time: 2023-07-20 10:25:40


import numpy as np
'''
adopted from: 
https://stackoverflow.com/questions/60877274/optimal-rotation-in-3d-with-kabsch-algorithm

'''

map_points = [[-10.15, -3.71, 5],
			[-4.55, 10.74, 5],
			[9.91, 4.79, 5],
			[4.6, -10.6, 5]]

true_points = [[15, 0, 0],
			[0, 15, 0],
			[-15, 0, 0],
			[0, -15, 0]]

mapped_centroid = np.average(map_points, axis=0)
true_centroid = np.average(true_points, axis=0)

map_points -= mapped_centroid
true_points -= true_centroid

h = map_points.T @ true_points
u, s, vt = np.linalg.svd(h)
v = vt.T

d = np.linalg.det(v @ u.T)
e = np.array([[1, 0, 0], [0, 1, 0], [0, 0, d]])

r = v @ e @ u.T

tt = true_centroid - np.matmul(r, mapped_centroid)
print(r, tt)
# np.save('Rt.npy', r)
# np.save('Tt.npy', tt)