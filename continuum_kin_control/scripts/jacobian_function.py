import numpy as np
def jacobian(l, n, theta, phi):

	Csum = float(0)
	Dsum = float(0)
	Esum = float(0)
	# print(type(Csum))
	for k in range(1, n+1):
		idx = float(k)
		Csum = Csum + 2*l*(idx-1)/(n-1)*np.cos((idx-1)/(n-1)*float(theta))
		Dsum = Dsum + 2*l*np.sin((idx-1)/(n-1)*theta)
		Esum = Esum - 2*l*(idx-1)/(n-1)*np.sin((idx-1)/(n-1)*theta)

	C = Csum - l*np.cos(theta)
	D = Dsum - l*np.sin(theta)
	E = Esum + l*np.sin(theta)

	J = [[np.cos(phi)*C, -1*np.sin(phi)*D], [np.sin(phi)*C, np.cos(phi)*D], [E, 0]]

	return J