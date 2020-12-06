import numpy as np
#import matplotlib.image as mpimg
#import matplotlib.pyplot as plt
import time
import bpy
import os
import time
from pathlib import Path
from threading import Thread

import sys
#sinon il trouvera pas les modules custom si on change pas son path d'import
sys.path.insert(0, r"C:\Users\Alexandre.Bergeron\OneDrive - USherbrooke\university\projet\S5_projet_simulation")
from bille import BilleMath
import creationLigne


#### Tout ce qui a trait à blender ####

class blenderObject():
    scale = 1
    radius = 2
    name = "undefined" #name of the blender object
    name_2 = ""
    frameNb = 0
    framerate = 100 #besoin de savoir ça?

    def __init__(self, x, y, z, name="undefined", scene=None):
        self.position = np.array([x*self.scale, y*self.scale, z*self.scale], dtype=np.float)
        self.rotation = np.array([0, 0, 0])
        self.name = name
        self.scene = scene
        if scene is not None:
            i = 1
            while(self.name + self.name_2 in bpy.data.objects):
                self.name_2 = "." + str(i).zfill(3)
                i += 1
            
            path = Path(os.getcwd()) / "blender" / f"{self.name}.dae"
            bpy.ops.wm.collada_import(filepath=str(path), import_units=False, keep_bind_info=False)
            self.blenderObj = bpy.data.objects[self.name+self.name_2]
            self.blenderObj.animation_data_clear()
        
            #place l'objet à son point de départ
            self.scene.frame_set(self.frameNb)
            self.blenderObj.location = tuple(self.position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'
            mat = bpy.data.materials.new(name=self.name+self.name_2)
            mat.diffuse_color = (1,1,1, 1)
            self.blenderObj.data.materials.append(mat)
            bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)
    
    def mouvementLocal(self, deltaPosition):
        if self.scene is not None:
            self.frameNb += 1
            self.scene.frame_set(self.frameNb)
            self.position += deltaPosition
            self.blenderObj.location = tuple(self.position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'
    
    def ajouteOffset(self, offset):
        if self.scene is not None:
            position = self.position + offset
            self.blenderObj.location = tuple(position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'

    def show(self, fig, ax):
        ax.plot(self.position[0], self.position[1], "xr")

    
    def couleurRouge(self):
        bpy.data.materials.get(self.name+self.name_2).diffuse_color = (1, 0, 0, 1)
        bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)

    def couleurVert(self):
        bpy.data.materials.get(self.name+self.name_2).diffuse_color = (0, 1, 0, 1)
        bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)



class vehicule(blenderObject):
    def __init__(self, x,y,z,scene):
        super().__init__(x,y,z, "vehicule", scene)
        self.bille = bille(x, y-0.08, z+0.020, scene)
        self.sonar = sonar(x, y-0.14, z+0.05, scene)
        self.suiveurligne = CapteurLigne(x, y-0.14, z, scene)
        #ajout du sonar à l'avant
    
    def mouvementLocal(self, deltaPosition):
        super().mouvementLocal(deltaPosition)
        self.sonar.mouvementLocal(deltaPosition)
        self.suiveurligne.mouvementLocal(deltaPosition)

        #déterminer accélération x et y pis shooter ça à bille
        #je pense que deltaposition serait une accélération en fait
        self.bille.bougeBille(deltaPosition)
    
    def detection(self, listeObj):

        

class bille(blenderObject):

    def __init__(self, x, y, z, scene):
        super().__init__(x, y, z, "bille", scene)
        self._vielleVitesse = np.array([0,0])
        self.billeMath = BilleMath(scene.render.fps)
        #qu'est-ce qu'on a besoin de savoir? Accélération en x et y? angle en z? faire le z ou pas?

    def bougeBille(self, deltaPosition):
        #vérifier si la vitesse à changer:
        vitesseCourante = deltaPosition
        if(vitesseCourante[0] != self._vielleVitesse[0]):
            self.billeMath.appliqueAcceleration(X_vitesse=vitesseCourante[0]*self.scene.render.fps)

        if(vitesseCourante[1] != self._vielleVitesse[1]):
            self.billeMath.appliqueAcceleration(Y_vitesse=vitesseCourante[1]*self.scene.render.fps)
        
        positionBille = self.billeMath.updatePosition()
        self.mouvementLocal(deltaPosition)
        self.ajouteOffset(positionBille)

class DetecteurLigne(blenderObject):

    def __init__(self, x, y, z, scene):
        super().__init__(x, y, z, "undefined", scene)

    def configureLigne(self, ligne):
        self.ligne = ligne 

    def detection(self):
        detect = self.ligne.estDansLigne(self.position)
        if(detect == 1):
            self.couleurVert()
        else:
            self.couleurRouge()
        return detect

#représente le module avec les 5 détecteurs
class CapteurLigne(blenderObject):
    def __init__(self, x, y, z, scene):
        super().__init__(x, y, z, "undefined", scene)
        self.detecteurs = []
        self.detecteurs.append(DetecteurLigne(x-0.08, y, z, scene))
        self.detecteurs.append(DetecteurLigne(x-0.04, y, z, scene))
        self.detecteurs.append(DetecteurLigne(x, y, z, scene))
        self.detecteurs.append(DetecteurLigne(x+0.04, y, z, scene))
        self.detecteurs.append(DetecteurLigne(x+0.08, y, z, scene))
    
    def mouvementLocal(self, deltaPosition):
        super().mouvementLocal(deltaPosition)
        for detecteur in self.detecteurs:
            detecteur.mouvementLocal(deltaPosition)

    def detection(self):
        resultat = []
        for detecteur in self.detecteurs:
            resultat.append(detecteur.detection())
        return resultat

    def configureLigne(self, ligne):
        resultat = []
        for detecteur in self.detecteurs:
            detecteur.configureLigne(ligne)


class sonar(blenderObject): 
    def __init__(self, x, y, z, scene):
        super().__init__(x,y,z, "undefined", scene)
        self.max_range = 4.5*self.scale #mètre
        self.angle = 30 # angle en degrées
        self.precision = 0.01*self.scale

    def Check(self, blenderObjectList):
        raise Exception("Sonar pas implémenté")
        return -1 #dans la librairie, -1 est retourné s'il y a rien        

    def show(self, fig, ax):
        ax.set_title('sonar')
        ax.imshow(self.onde)


#gere la connection blender au script

class blenderManager(Thread):
    _foward_speed = 0
    _rotationServo = 0
    _distanceSonar = 0

    #constant
    _circonference_roue = 0.04*2*np.pi
    framerate = 100
    _step = 1/framerate #100 serait 100fps, donc 1 seconde.
    _rpsMax = 4 # rotation par seconde à confirmer

    #liste d'état
    _avance = False
    _stop = True
    _recule = False


    #l'argument secondes est la durée de la simulation
    def __init__(self, secondes, nomDeLigne):
        super().__init__()
        self._tempsDeSimulation = secondes
        #delete tout les trucs
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=True, confirm=False)
        #load blender scene
        scene = bpy.context.scene
        if scene is not None:
            scene.render.fps = self.framerate
        self.vehicule = vehicule(0, 0, 0, scene)
        bpy.context.scene.frame_end = int(secondes*self.framerate)

        #crée la ligne
        ligne = creationLigne.Ligne(nomDeLigne, getattr(creationLigne, nomDeLigne), 0.6)
        self.vehicule.suiveurligne.configureLigne(ligne)


    def read_digital(self):
        return self.vehicule.suiveurligne.detection()


    def run(self):
        nombreStepAvantLaFin = self.framerate*self._tempsDeSimulation
        nombreStep = 0
    
        while(nombreStep < nombreStepAvantLaFin):
            start = time.time()
            if self._avance:
                distance = (self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._recule:
                distance = -(self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._stop:
                distance = 0
            else:
                raise Exception("Aucun mode active pour la classe blenderManager")
            


            #pas de rotation pour le moment à prendre en compte, donc je vais juste mettre la vitesse dans le y
            position = np.array([0, distance, 0])
            self.vehicule.mouvementLocal(position)

            nombreStep += 1
            tempsEcoule = time.time() - start
            if(tempsEcoule > 1/self.framerate):
                #c'est peut-être juste un breakpoint aussi, donc pas d'exception on continue
                print("La simulation n'est pas assez performante, risque d'erreur")
                print(f"etape {nombreStep}")
            else:
                time.sleep((1/self.framerate)-tempsEcoule)

            #maintenant qu'on a la distance, le convertir en x, y et z



    def forward(self):
        self._avance = True
        self._stop = False
        self._recule = False
    
    def backward(self):
        self._avance = False
        self._stop = False
        self._recule = True

    def stop(self):
        self._avance = False
        self._stop = True
        self._recule = False

    #vitesse de 0 à 100
    def set_speed(self, speed):
        if(speed < 0 or speed > 100):
            raise Exception(f"Vitesse invalide dans set_speed({speed})")
        self._foward_speed = speed/100


#L = capteur_sonar.Check(objlist)
#print(f"obstacle detecte a {L}m")

blender = blenderManager(10, "crochet")
blender.start()
blender.forward()
blender.set_speed(20)
for i in range(10):
    print(blender.read_digital())
    time.sleep(1)
blender.join()
print("fini")