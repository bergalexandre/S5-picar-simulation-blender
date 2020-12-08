import bpy
import numpy as np

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

        mesh = bpy.data.meshes.new(nom)
        obj = bpy.data.objects.new(mesh.name, mesh)
        col = bpy.data.collections.get("Collection")
        col.objects.link(obj)
        bpy.context.view_layer.objects.active = obj

        X_array = np.linspace(0, L, num=int(L*100)+1)
        y = fonction(X_array[0])
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
        
        #transformation en numpy arraay


    #formule de héron
    def calculAireTriangle(self, a,b,c):
        p = (a+b+c)/2
        return np.sqrt(p*(p-a)*(p-b)*(p-c))

    def calculNorme(self, position1, position2):
        longueur = position2[:2]-position1[:2]
        return np.sqrt(longueur[0]**2 + longueur[1]**2)
    
    def estDansLigne(self, position):
        position = np.array(position)
        for face in self.faces:
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
                return 1
        
        return 0

    def matriceRotation(self, vecteur, angle):
        #matrice de rotation
        matrice_rotation = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        matrice_rotation = np.round(matrice_rotation, decimals=5)

        #pour info, @ = produit de matrice
        vecteur = np.asarray(vecteur)
        vecteur[:,:2] = vecteur[:,:2] @ matrice_rotation

        return vecteur

def crochet(x):
    y = 0
    if(x <= 2):
        y = 0
    else:
        y = np.sin(x-2)
    return y


def test(x, y, ligne):
    print(f"[{x}, {y}] est: {True if ligne.estDansLigne([x, y]) == 1 else False}")    


def tests():
    ligne = Ligne("test", crochet, 5, 35)

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

#tests()