import numpy as np

class Infrasonic():

    longueur = 2
    champsVisionRadar = np.radians(15)

    aires = [   round(((2*longueur*np.sin(champsVisionRadar)) * longueur * np.cos(champsVisionRadar))/2, 6),
                round(((2*longueur*np.sin(champsVisionRadar)) * (longueur - (longueur*np.cos(champsVisionRadar))))/2, 6)
            ]

    #Crée deux triangle pour simuler la position du capteur
    def __init__(self, positionSonar, milieuVehicule):
        #déterminer dans quel cadrant du cercle on est
        droite = False
        haut = False
        angleOffset = 0
        if(milieuVehicule[0] <= positionSonar[0]):
            droite = True
        if(milieuVehicule[1] <= positionSonar[1]):
            haut = True

        if(droite and haut):
            angleOffset = 0
        elif(not droite and haut):
            angleOffset = np.pi/2
        elif(not droite and not haut):
            angleOffset = np.pi
        else:
            angleOffset = 3*np.pi/2
        

        pentes = abs(positionSonar - milieuVehicule)
        rotationActuelle = 0
        try:
            rotationActuelle = np.arctan(pentes[1]/pentes[0])
        except Exception as e:
            rotationActuelle = pi/2

        rotationActuelle += angleOffset
        self.verts = np.array([
            np.array(positionSonar),
            np.array(positionSonar+[self.longueur * np.cos(rotationActuelle-self.champsVisionRadar), self.longueur * np.sin(rotationActuelle-self.champsVisionRadar), 0]),
            np.array(positionSonar+[self.longueur * np.cos(rotationActuelle+self.champsVisionRadar), self.longueur * np.sin(rotationActuelle+self.champsVisionRadar), 0]),
            np.array(positionSonar+[self.longueur * np.cos(rotationActuelle), self.longueur * np.sin(rotationActuelle), 0])
        ])

        print(self.verts)
    def calculNorme(self, position1, position2):
        longueur = position2-position1
        return np.sqrt(longueur[0]**2 + longueur[1]**2)
    
    #formule de héron
    def calculAireTriangle(self, a,b,c):
        p = (a+b+c)/2
        return np.sqrt(p*(p-a)*(p-b)*(p-c))

    def estDansOnde(self, position):
        faces = [[0,1,2], [1,2,3]]
        for face in faces:
            a = self.calculNorme(self.verts[face[0]], position)
            b = self.calculNorme(self.verts[face[1]], position)
            c = self.calculNorme(self.verts[face[0]], self.verts[face[1]])
            aireTotal = self.calculAireTriangle(a, b, c)
            a = self.calculNorme(self.verts[face[1]], position)
            b = self.calculNorme(self.verts[face[2]], position)
            c = self.calculNorme(self.verts[face[1]], self.verts[face[2]])
            aireTotal += self.calculAireTriangle(a, b, c)
            a = self.calculNorme(self.verts[face[0]], position)
            b = self.calculNorme(self.verts[face[2]], position)
            c = self.calculNorme(self.verts[face[0]], self.verts[face[2]])
            aireTotal += self.calculAireTriangle(a, b, c)
        
            aireTotal = round(aireTotal, 6)
            if aireTotal in self.aires:
                return self.calculNorme(position, self.verts[0])
        
        return -1



def test():
    import matplotlib.pyplot as plt
    positionOrigine = np.array([-3, 3, 0])
    positionCapteur = np.array([-3.5, 3.5, 0])

    sonar = Infrasonic(positionCapteur, positionOrigine)

    plt.figure(1)
    plt.plot(positionOrigine[0], positionOrigine[1], "rx")
    plt.plot(positionCapteur[0], positionCapteur[1], "bx")

    plt.plot(sonar.verts[:,0], sonar.verts[:,1])

    for coord in [[-4.5, 4.5, 0], [0, 0, 0], [-4.6, 4.9, 0], [-4.9, 4.9, 0]]:
        print(f"coord: [{coord}] est {True if sonar.estDansOnde(coord) != -1 else False}")
        plt.plot(coord[0], coord[1], "gx")

    plt.show()

#test()

