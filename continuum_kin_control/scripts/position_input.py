import numpy as np
from inverse_function import pose
from jacobian_function import jacobian
import time

l = 18
r = 21
n = 8
theta = np.pi/10
phi = 0
psi = [theta, phi]

error_p = np.transpose([1, 1, 1])
error_c = np.transpose([1000000, 1000000, 1000000])

def closest_position(xdes, ydes, zdes):
	global error_p
	global error_c
	global theta
	global phi
	global psi

	while 1:
		

		x = pose(l, r, n, theta, phi)
		J = jacobian(l, n, theta, phi)

		error_p = error_c
		error_c = [xdes - x.item(0), ydes - x.item(1), zdes - x.item(2)]

		error_norm = np.absolute(np.linalg.norm(error_c) - np.linalg.norm(error_p))/np.linalg.norm(error_p)
		# print(error_norm)
		if error_norm < 0.001:
			break

		xdot = np.dot(error_c,0.1)
		A = np.linalg.inv(np.dot(np.transpose(J), J))
		B = np.transpose(J)
		# print(A)
		# print(B)
		psidot = np.dot(np.dot(A, B), xdot)
		# print(psidot)
		psi = psi + psidot
		theta = psi.item(0)
		phi = psi.item(1)
		# time.sleep(0.1)

	return psi