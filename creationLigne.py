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
        faces = []

        mesh = bpy.data.meshes.new(nom)
        obj = bpy.data.objects.new(mesh.name, mesh)
        col = bpy.data.collections.get("Collection")
        col.objects.link(obj)
        bpy.context.view_layer.objects.active = obj

        X_array = np.linspace(0, L, num=int(L*100)+1)
        y = fonction(X_array[0])
        self.verts.append((y-self._largeur, X_array[0], 0.0))
        self.verts.append((y+self._largeur, X_array[0], 0.0))
        for x in X_array[1:]:
            y = fonction(x)
            self.verts.append((y-self._largeur, x, 0.0))
            self.verts.append((y+self._largeur, x, 0.0))
            taille = len(self.verts)
            faces.append([taille-4, taille-3, taille-1, taille-2])

        mesh.from_pydata(self.verts, edges, faces)
        obj.rotation_euler[2] = np.radians(rotationZ)
        if(rotationZ!=0):
            raise Exception("Rotation non supporté, reste 0 stp sinon faudra faire une matrice de rotation comme a l'app 3")

    def estDansLigne(self, position):
        x = position[0]
        y = position[1]
        taille = len(self.verts)-3
        for i in range(0, taille, 2):
            longeur = self.verts[i+1][0] - self.verts[i][0]
            largeur = self.verts[i+3][1] - self.verts[i][1]
            aireTotal = largeur*longeur
            triangles = [   [self.verts[i+1][0] - self.verts[i][0],   position[1] - self.verts[i+1][1]], 
                            [self.verts[i+3][1] - self.verts[i][1],   position[0] - self.verts[i+3][0]], 
                            [self.verts[i+2][1] - self.verts[i+1][1], position[0] - self.verts[i+2][0]], 
                            [self.verts[i+3][0] - self.verts[i+2][0], position[1] - self.verts[i+3][1]]]
            aireSomme = 0
            for triangle in triangles:
                aireSomme += abs(triangle[0]*triangle[1]/2)
            
            if(aireSomme == aireTotal):
                return 1
                
        return 0

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
    ligne = Ligne("test", crochet, 0.02)

    test(0.005, 0.005, ligne)
    test(0.01, 0.01, ligne)
    test(-0.005, 0.005, ligne)
    test(0.005, -0.005, ligne)
    test(0.0, 0.0, ligne)
    test(0.011, 0.011, ligne)
    test(0.003, 0.005, ligne)
    test(0.012, 0.005, ligne)
    test(0.012, 0.012, ligne)
    test(0.0, 0.015, ligne)

#tests