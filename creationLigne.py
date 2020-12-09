import bpy
import numpy as np
import time
class Ligne():
    

    #attention les x et y sont inversé
    #scene: l'objet blender où ajouter le mesh
    #fonction: f(x) qui donne un y
    def __init__(self, nom, fonction, L, rotationZ=0):
        self._largeur = 0.018/2
        self.verts = []
        edges = []
        self.faces = []
        self.aires = []
        self.interval = 0.2

        mesh = bpy.data.meshes.new(nom)
        obj = bpy.data.objects.new(mesh.name, mesh)
        col = bpy.data.collections.get("Collection")
        col.objects.link(obj)
        bpy.context.view_layer.objects.active = obj

        X_array = np.linspace(0, L, num=int(L*100)+1)
        y = fonction(X_array[0])
        self.fonction = fonction
        self.verts.append(np.array([X_array[0], y-self._largeur, 0.0]))
        self.verts.append(np.array([X_array[0], y+self._largeur, 0.0]))
        for x in X_array[1:]:
            y = fonction(x)
            self.verts.append(np.array([x, y-self._largeur, 0.0]))
            self.verts.append(np.array([x, y+self._largeur, 0.0]))
            taille = len(self.verts)
            self.faces.append([taille-4, taille-3, taille-2])
            a = self.calculNorme(self.verts[self.faces[-1][0]], self.verts[self.faces[-1][2]])
            b = self.calculNorme(self.verts[self.faces[-1][1]], self.verts[self.faces[-1][2]])
            c = self.calculNorme(self.verts[self.faces[-1][0]], self.verts[self.faces[-1][1]])
            self.aires.append(round(self.calculAireTriangle(a, b, c), 6))

            self.faces.append([taille-1, taille-2, taille-3])
            a = self.calculNorme(self.verts[self.faces[-1][0]], self.verts[self.faces[-1][2]])
            b = self.calculNorme(self.verts[self.faces[-1][1]], self.verts[self.faces[-1][2]])
            c = self.calculNorme(self.verts[self.faces[-1][0]], self.verts[self.faces[-1][1]])
            self.aires.append(round(self.calculAireTriangle(a, b, c), 6))

        self.verts = self.matriceRotation(self.verts, np.radians(rotationZ))
        mesh.from_pydata(tuple(map(tuple, self.verts)), edges, tuple(map(tuple, self.faces)))
        
        self.faceDict = self.classifieFaces(self.faces, 0.1, L, 0)
        for key in self.faceDict:
            self.faceDict[key] = self.classifieFaces(self.faceDict[key], 0.1, L, 1)
        #transformation en numpy arraay

    #arrondie vers la bas au dixième près
    def round_up(self, n):
        if(n < 0):
            return (int(n*10)-1)/10
        else:
            return (int(n*10)+1)/10
    
    def round_down(self, n):
        if(n < 0):
            return (int(n*10)-1)/10
        else:
            return (int(n*10))/10

    #Cette fonction va classer les faces dans un dictionnaire pour permettre de retrouver rapidement un index X, Y donnée (sans passez au travers tout les vertex possible.)
    def classifieFaces(self, faces, interval, L, pos):
        facesDict = {}
        min_value = min(self.verts[:,pos])
        min_value = self.round_down(min_value)
        max_value = max(self.verts[:,pos])
        max_value = self.round_up(max_value)
        N = np.linspace(min_value, max_value-0.1, num=(max_value-min_value)/interval)
        for n in N:
            key = round(n, 1)
            if(key not in facesDict):
                    facesDict[key] = []
            for face in faces:
                valeur = abs(self.verts[face[0]][pos])
                #2 fois l'interval pour augmenter la porté du dictionnaire à la valeur n
                if(valeur < (abs(n)+2*interval) and valeur >= (abs(n)-2*interval)):
                    facesDict[key].append(face)
        return facesDict



    #détermine la longueur des lignes du triangle
    def determineTriangle(self, P0, P1, P2):
        a = self.calculNorme(P0, P2)
        b = self.calculNorme(P1, P2)
        c = self.calculNorme(P0, P1)
        return a, b, c

    #formule de héron
    def calculAireTriangle(self, a,b,c):
        p = (a+b+c)/2
        return np.sqrt(p*(p-a)*(p-b)*(p-c))

    #calcul une norme entre un point A et B
    def calculNorme(self, position1, position2):
        longueur = position2[:2]-position1[:2]
        return np.sqrt(longueur[0]**2 + longueur[1]**2)
    
    #calcul de ligne en se fiant sur la fonction (pas fiable)
    def estDansLigne2(self, position):
        y = self.fonction(position[0])
        borne1 = y-self._largeur
        borne2 = y+self._largeur
        if(borne1 <= position[1] and position[1] <= borne2):
            return 1
        return 0


    #Calcul l'air de chaque face identifié contenant potentielement la position. 
    def estDansLigne(self, position):
        position = np.array(position)
        keyX = self.round_down(position[0])
        keyY = self.round_down(position[1])
        try:
            for face in self.faceDict[keyX][keyY]:
                #print(f"{self.verts[face[0]]} et {self.verts[face[1]]} et {self.verts[face[2]]}")
                a, b, c = self.determineTriangle(self.verts[face[0]], self.verts[face[1]], self.verts[face[2]])
                aire = self.calculAireTriangle(a, b, c)
                a, b, c = self.determineTriangle(self.verts[face[0]], self.verts[face[1]], position)
                aireTotal = self.calculAireTriangle(a, b, c)
                a, b, c = self.determineTriangle(self.verts[face[2]], self.verts[face[1]], position)
                aireTotal += self.calculAireTriangle(a, b, c)
                a, b, c = self.determineTriangle(self.verts[face[0]], self.verts[face[2]], position)
                aireTotal += self.calculAireTriangle(a, b, c)
                aireTotal = round(aireTotal, 6)
                if (aireTotal == round(aire, 6)):
                    return 1
        except KeyError as e:
            pass        
        return 0

    #Permet de rotationner une matrice de matrice
    def matriceRotation(self, vecteur, angle):
        #matrice de rotation
        matrice_rotation = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        matrice_rotation = np.round(matrice_rotation, decimals=5)

        #pour info, @ = produit de matrice
        vecteur = np.asarray(vecteur)
        vecteur[:,:2] = vecteur[:,:2] @ matrice_rotation

        return vecteur

#forme un crochet
def crochet(x):
    y = 0
    if(x <= 2):
        y = 0
    else:
        y = np.sin(x-2)
    return y

def test(x, y, ligne):
    print(f"[{x}, {y}] est: {True if ligne.estDansLigne2([x, y]) == 1 else False}")
    start = time.time()
    print(f"[{x}, {y}] est: {True if ligne.estDansLigne([x, y]) == 1 else False}")    
    stop = time.time()
    print(f"time took {stop-start}")


def tests():
    ligne = Ligne("test", crochet, 10, 35)

    test(0.005, 0.005, ligne)
    test(-0.0105, 0.01, ligne)
    test(-0.005, 0.005, ligne)
    test(0.005, -0.005, ligne)
    test(-0.005, -0.005, ligne)
    test(0.0, 0.0, ligne)
    test(0.011, 0.011, ligne)
    test(0.003, 0.005, ligne)
    test(0.012, 0.005, ligne)
    test(0.012, 0.012, ligne)
    test(0.0, 0.015, ligne)
    test(1, -0.699018, ligne)
    test(3.00988, -1.03765, ligne)
    test(2.94736, -1.13303, ligne)
    test(8.75612, -4.91907, ligne)
    test(3.93757, -1.81637, ligne)

tests()