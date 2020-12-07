import numpy as np
import matplotlib.pyplot as plt


class test():
    _nombreStep = 0
    _debutVirage = 0
    framerate = 100

    def virage(self, v, L, alpha):
        #https://math.stackexchange.com/questions/3055263/path-of-a-simple-turning-car
        #calculer par rapport à T
        t = (self._nombreStep-self._debutVirage)/self.framerate
        omega = v/L*np.tan(alpha)
        
        vitesseAngulaire = v/omega if omega != 0.0 else 0

        position_BackWheel = vitesseAngulaire * np.array([np.sin(omega*t), 1-np.cos(omega*t)])
        #position_FrontWheel = position_BackWheel + L * np.array([np.cos(omega*t), np.sin(omega*t)])

        return position_BackWheel, omega, t

    def update(self):
        self._nombreStep += 1


    #matrice de rotation
    def rotation(self, vecteur, angle):
        theta = np.radians(angle)
        matrice_rotation = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        matrice_rotation = np.round(matrice_rotation, decimals=5)
        return vecteur @ matrice_rotation



testobj = test()

v = 10
N = 2000

result = []
resultFW = []
angleDepart = 90
for n in range(N):
    bw, omega, t = testobj.virage(0.10, 0.15, np.radians(45))
    result.append(bw)
    testobj.update()
    resultFW.append(result[-1] + 0.15*np.array([np.cos(omega*t), np.sin(omega*t)]))



plt.figure(1)
result = np.array(result)
resultFW = np.array(resultFW)
plt.plot(result[0][0], result[1][0], "xr")
plt.plot(result[:,0], result[:,1], "b")
plt.plot(resultFW[:,0], resultFW[:,1], "r")

result = testobj.rotation(result, 90)
resultFW = testobj.rotation(resultFW, 90)
plt.plot(result[0][0], result[1][0], "xg")
plt.plot(result[:,0], result[:,1], "g")
plt.plot(resultFW[:,0], resultFW[:,1])

angle = np.arctan((resultFW[:,1]-result[:,1])/(resultFW[:,0]-result[:,0]))
plt.figure("angle")
plt.plot(angle)

plt.show()