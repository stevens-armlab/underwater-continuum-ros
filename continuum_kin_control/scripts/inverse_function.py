import numpy as np
# from scipy.spatial.transform import Rotation

def inverse(l, r, n, theta, phi):
	theta_link = theta/(n-1)

	L = np.transpose(np.matrix([0,0,0]))
	J = np.transpose(np.matrix([0,0,l]))
	P1 = np.transpose(np.matrix([r,0,0]))
	P2 = np.transpose(np.matrix([0,r,0]))
	P3 = np.transpose(np.matrix([-r,0,0]))
	P4 = np.transpose(np.matrix([0,-r,0]))
	Rot = np.matrix([[1,0,0],[0,1,0],[0,0,1]])

	R1 = np.matrix([[np.cos(theta_link), 0, np.sin(theta_link)], [0, 1, 0], [-1*np.sin(theta_link), 0, np.cos(theta_link)]])
	R2 = np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
	R = np.dot(R2, np.dot(R1, np.linalg.inv(R2)))

	for i in range(1,n):
		Rot = np.dot(Rot, R)
		Lnew = np.add(J[:,i-1], np.dot(Rot, np.transpose(np.matrix([0,0,l]))))
		L = np.append(L, Lnew, axis=1)
		Jnew = np.add(L[:,i], np.dot(Rot, np.transpose(np.matrix([0,0,l]))))
		J = np.append(J, Jnew, axis=1)
		P1new = np.add(L[:,i], np.dot(Rot, np.transpose(np.matrix([r,0,0]))))
		P1 = np.append(P1, P1new, axis=1)
		P2new = np.add(L[:,i], np.dot(Rot, np.transpose(np.matrix([0,r,0]))))
		P2 = np.append(P2, P2new, axis=1)
		P3new = np.add(L[:,i], np.dot(Rot, np.transpose(np.matrix([-r,0,0]))))
		P3 = np.append(P3, P3new, axis=1)
		P4new = np.add(L[:,i], np.dot(Rot, np.transpose(np.matrix([0,-r,0]))))
		P4 = np.append(P4, P4new, axis=1)

	d_neutral = (n-1)*2*l

	D1 = 0
	D2 = 0
	D3 = 0
	D4 = 0

	for i in range(0, n-1):
		d1 = 0
		d2 = 0
		d3 = 0
		d4 = 0

		for j in range(0, 3):
			d1 = d1 + (P1[j,i+1] - P1[j,i])**2
			d2 = d2 + (P2[j,i+1] - P2[j,i])**2
			d3 = d3 + (P3[j,i+1] - P3[j,i])**2
			d4 = d4 + (P4[j,i+1] - P4[j,i])**2

		D1 = D1 + np.sqrt(d1)
		D2 = D2 + np.sqrt(d2)
		D3 = D3 + np.sqrt(d3)
		D4 = D4 + np.sqrt(d4)
	
	Q1 = D1 - d_neutral
	Q2 = D2 - d_neutral
	Q3 = D3 - d_neutral
	Q4 = D4 - d_neutral

	Q = [Q1, Q2, Q3, Q4]
	D = [D1, D2, D3, D4]

	return Q

def pose(l, r, n, theta, phi):
	theta_link = theta/(n-1)

	L = np.transpose(np.matrix([0,0,0]))
	J = np.transpose(np.matrix([0,0,l]))
	R1 = np.matrix([[np.cos(theta_link), 0, np.sin(theta_link)], [0, 1, 0], [-1*np.sin(theta_link), 0, np.cos(theta_link)]])
	R2 = np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
	R = np.dot(R2, np.dot(R1, np.linalg.inv(R2)))

	Rot = np.zeros((3,3,n))
	quat = np.zeros((4,n))
	Rot[:,:,0] = np.identity(3)
	# quat[:, 0] = Rotation.from_matrix(Rot[:,:,0]).as_quat()

	for i in range(1,n):
		Rot[:,:,i] = np.dot(Rot[:,:,i-1], R)
		Lnew = np.add(J[:,i-1], np.dot(Rot[:,:,i], np.transpose(np.matrix([0,0,l]))))
		L = np.append(L, Lnew, axis=1)
		Jnew = np.add(L[:,i], np.dot(Rot[:,:,i], np.transpose(np.matrix([0,0,l]))))
		J = np.append(J, Jnew, axis=1)
		
	return L[:,n-1]
# print(inverse(18,21,8,np.pi/2,np.pi/3))