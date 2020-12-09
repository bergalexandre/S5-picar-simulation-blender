import numpy as np
#import matplotlib.image as mpimg
#import matplotlib.pyplot as plt
import time
import bpy
import os
import time
import copy
from pathlib import Path
from threading import Thread, Lock

import sys
#sinon il trouvera pas les modules custom si on change pas son path d'import
sys.path.insert(0, r"C:\Users\Alexandre.Bergeron\OneDrive - USherbrooke\university\projet\S5_projet_simulation")
from bille import BilleMath
import creationLigne
import infrasonic

#### Tout ce qui a trait à blender ####
blenderMutex = Lock()
frameNb = 0

class blenderObject():
    scale = 1
    radius = 2
    name = "undefined" #name of the blender object
    name_2 = ""
    framerate = 100 #besoin de savoir ça?
    angle = 0
    _dernierePosition = np.array([0,0,0])

    def __init__(self, x, y, z, name="undefined", scene=None, parent=None):
        self.position = np.array([x*self.scale, y*self.scale, z*self.scale], dtype=np.float)
        self.name = name
        self.scene = scene
        self.parent = parent
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
            global frameNb
            self.scene.frame_set(frameNb)
            self.blenderObj.location = tuple(self.position/self.scale)
            if(parent is not None):
                self.blenderObj.parent = parent
                self.blenderObj.matrix_parent_inverse = parent.matrix_world.inverted()
            else:
                self.blenderObj.location = tuple(self.position/self.scale)
                self.blenderObj.keyframe_insert(data_path="location", index=-1)
                self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'

            #initialise la rotation à 0
            self.rotation(0)

            mat = bpy.data.materials.new(name=self.name+self.name_2)
            mat.diffuse_color = (1,1,1, 1)
            self.blenderObj.data.materials.append(mat)
            bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)
    
    #pour un mouvement local, on assume que on veut avancer sur la position X par exemple mais celle "local". 
    #en gros, elle fait toujours face à l'origine avec un certain angle. (voir blender local versus global)
    def matriceRotation(self, deltaPosition, angle):
        #matrice de rotation
        matrice_rotation = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        matrice_rotation = np.round(matrice_rotation, decimals=5)

        #pour info, @ = produit de matrice
        deltaPosition[:2] = deltaPosition[:2] @ matrice_rotation

        return deltaPosition 

    def mouvementLocal(self, deltaPosition):
        if self.scene is not None:
            blenderMutex.acquire()
            global frameNb
            self.scene.frame_set(frameNb)
            self._dernierePosition = self.position
            self.position += deltaPosition
            self.blenderObj.location = tuple(self.position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'
            blenderMutex.release()
    
    def ajouteOffset(self, offset):
        if self.scene is not None:
            blenderMutex.acquire()
            offset = offset
            position = self.position + offset
            self.blenderObj.location = tuple(position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'
            blenderMutex.release()

    def show(self, fig, ax):
        ax.plot(self.position[0], self.position[1], "xr")

    def rotation(self, angle):
        if self.scene is not None:
            blenderMutex.acquire()
            self.blenderObj.rotation_euler[2] = angle
            self.blenderObj.keyframe_insert(data_path="rotation_euler", index=-1)
            blenderMutex.release()

    def enregistreRotation(self):
        if self.scene is not None:
            blenderMutex.acquire()
            self.blenderObj.keyframe_insert(data_path="rotation_euler", index=-1)
            blenderMutex.release()

    def couleurRouge(self):
        blenderMutex.acquire()
        global frameNb
        self.scene.frame_set(frameNb)
        bpy.data.materials.get(self.name+self.name_2).diffuse_color = (1, 0, 0, 1)
        bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)
        blenderMutex.release()

    def couleurVert(self):
        blenderMutex.acquire()
        global frameNb
        self.scene.frame_set(frameNb)
        bpy.data.materials.get(self.name+self.name_2).diffuse_color = (0, 1, 0, 1)
        bpy.data.materials.get(self.name+self.name_2).keyframe_insert(data_path="diffuse_color", index=-1)
        blenderMutex.release()
    
    #compare avec la dernière position connu
    def determineAngle(self, A, B):

        if (B[0]-A[0] == 0 and B[1]-A[1] == 0):
            return self.angle
        droite = False
        haut = False
        angleOffset = 0
        if(A[0] <= B[0]):
            droite = True
        if(A[1] <= B[1]):
            haut = True

        if(droite and haut):
            angleOffset = 0
        elif(not droite and haut):
            angleOffset = np.pi/2
        elif(not droite and not haut):
            angleOffset = np.pi
        else:
            angleOffset = 3*np.pi/2
        
        longueur = B - A
        rotationActuelle = np.arctan(longueur[1]/longueur[0])
        if np.isnan(rotationActuelle): #pour la division par 0
            rotationActuelle = 0.0

        rotationActuelle += angleOffset
        return rotationActuelle


class vehicule(blenderObject):
    length = 0.15
    
    def __init__(self, x,y,z,scene):
        super().__init__(x,y,z, "vehicule", scene)
        global frameNb
        frameNb = 0
        self.angleVirage = 0
        self.bille = bille(x, y, z+0.035, scene, self.blenderObj)
        self.sonar = sonar(x+0.065, y, z+0.05, scene, self.blenderObj)
        self.suiveurligne = CapteurLigne(x+0.065, y, z, scene, self.blenderObj)
        #ajout du sonar à l'avant

    def mouvementLocal(self, deltaPosition, omega=0, t=0, rot=True):
        #deltaPosition est l'équivalent du backwheel pour le véhicule
        if(rot is True):
            deltaPosition = self.matriceRotation(deltaPosition, self.angle)
        super().mouvementLocal(deltaPosition)
        self.mouvementFrontwheel(omega, t)
        #déterminer accélération x et y pis shooter ça à bille
        #je pense que deltaposition serait une accélération en fait
        self.bille.bougeBille(deltaPosition)
        self.enregistreRotation()
    
    def mouvementFrontwheel(self, omega, t):
        fw_position = self.length*np.array([np.cos(omega*t), np.sin(omega*t), 0])
        self.ajouteOffset(self.matriceRotation(fw_position, self.angle)) #le point d'origine du vehicule blender est le milieu des frontwheels

    def virage(self, v, alpha, t):
        #https://math.stackexchange.com/questions/3055263/path-of-a-simple-turning-car
        #calculer par rapport à T
        omega = v/self.length*np.tan(alpha)        
        vitesseAngulaire = v/omega if omega != 0.0 else 0
        position_BackWheel = vitesseAngulaire * np.array([np.sin(omega*t), 1-np.cos(omega*t), 0])
        return self.matriceRotation(position_BackWheel, self.angle), omega

    def avance(self, vitesse, angleRoue, t, T0):
        if(angleRoue == 0.0):
            self.angleVirage = 0
            self.mouvementLocal([vitesse, 0, 0])
        else:
            position1 = self.position[:] #[:] pour forcer une copie des valeurs (sinon la valeur de la ref va changer)
            position_BackWheel, omega = self.virage(vitesse*self.framerate, angleRoue, t)
            position2 = (position_BackWheel+T0)-self.position
            self.mouvementLocal(position2, omega=omega, t=t, rot=False)
            self.angleVirage = omega*t
            self.rotation(-self.angle + self.angleVirage)

    def detection(self, listeObj):
        pass

        

class bille(blenderObject):

    def __init__(self, x, y, z, scene, parent):
        super().__init__(x, y, z, "bille", scene, parent)
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
        
        self._vielleVitesse = vitesseCourante
        positionBille = self.billeMath.updatePosition()
        self.ajouteOffset(positionBille)

class DetecteurLigne(blenderObject):
    def configureLigne(self, ligne):
        self.ligne = ligne 

    def detection(self):
        blenderMutex.acquire()
        position = np.asarray(self.parent.location)+np.asarray(self.blenderObj.location)
        blenderMutex.release()
        detect = self.ligne.estDansLigne(position)
        if(detect == 1):
            self.couleurVert()
        else:
            self.couleurRouge()
        return detect

#représente le module avec les 5 détecteurs
class CapteurLigne(blenderObject):
    def __init__(self, x, y, z, scene, parent):
        #super().__init__(x, y, z, "undefined", scene, parent)
        self.detecteurs = []
        self.detecteurs.append(DetecteurLigne(x, y-0.06, z, scene=scene, parent=parent))
        self.detecteurs.append(DetecteurLigne(x, y-0.03, z, scene=scene, parent=parent))
        self.detecteurs.append(DetecteurLigne(x, y, z, scene=scene, parent=parent))
        self.detecteurs.append(DetecteurLigne(x, y+0.03, z, scene=scene, parent=parent))
        self.detecteurs.append(DetecteurLigne(x, y+0.06, z, scene=scene, parent=parent))
    
    def mouvementLocal(self, deltaPosition):
        super().mouvementLocal(deltaPosition)
        for detecteur in self.detecteurs:
            detecteur.mouvementLocal(deltaPosition)

    def rotation(self, angle):
        super().rotation(angle)
        for detecteur in self.detecteurs:
            detecteur.rotation(angle)

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
    def __init__(self, x, y, z, scene, parent):
        super().__init__(x,y,z, "undefined", scene, parent)
        self.max_range = 4.5*self.scale #mètre
        self.angle = 30 # angle en degrées
        self.precision = 0.01*self.scale       

    def detection(self, listeObj):
        blenderMutex.acquire()
        positionRobot = np.asarray(self.parent.location)
        positionSelf = np.asarray(self.blenderObj.location) + positionRobot
        capteur = infrasonic.Infrasonic(positionRobot, positionSelf)
        distanceList = []
        for obj in listeObj:
            D = capteur.estDansOnde(obj.position)
            if(D != -1):
                distanceList.append(D)
        blenderMutex.release()
        return min(distanceList) if len(distanceList)>0 else -1

#gere la connection blender au script

class blenderManager(Thread):
    _foward_speed = 0
    _rotationServo = 0
    _distanceSonar = 0

    #constant
    _circonference_roue = 0.04*2*np.pi
    framerate = 100
    _step = 1/framerate #100 serait 100fps, donc 1 seconde.
    _rpsMax = 0.20/_circonference_roue # rotation par seconde à confirmer

    #liste d'état
    _avance = False
    _stop = True
    _recule = False

    #pour la sim
    turning_max = 135
    
    ## getter
    @property
    def speed(self):
        return self._foward_speed

    #Fonction(setter) pour ajuster la vitesse
    @speed.setter
    def speed(self, value):
        print(f"{frameNb} set_speed({value})")
        if(value < 0 or value > 100):
            raise Exception(f"Vitesse invalide dans set_speed({value})")
        self._foward_speed = value/100

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
            for material in bpy.data.materials:
                material.user_clear()
                bpy.data.materials.remove(material)
            
            for obj in bpy.data.objects:
                obj.user_clear()
                bpy.data.objects.remove(obj)

        self.vehicule = vehicule(0, 0, 0, scene)
        self.T0 = self.vehicule.position[:]
        bpy.context.scene.frame_end = int(secondes*self.framerate)
        
        self.listeObj = []
        self.listeObj.append(blenderObject(1, 0, 0, name="obstacle", scene=scene))
        self.listeObj.append(blenderObject(3, 3, 0, name="obstacle", scene=scene))
        self.listeObj.append(blenderObject(3, -3, 0, name="obstacle", scene=scene))

        self._nombreStep = 0
        self.turn(90)

        #crée la ligne
        ligne = creationLigne.Ligne(nomDeLigne, getattr(creationLigne, nomDeLigne), 10)
        self.vehicule.suiveurligne.configureLigne(ligne)

    #lecture du capteur de ligne
    def read_digital(self):
        lecture = self.vehicule.suiveurligne.detection()
        print(f"{frameNb} {lecture} = read_digital()")
        return lecture

    #Thread qui roule poiur la durée de la simulation. Son but est de comminiquer les informations aux classes simulés.
    def run(self):
        nombreStepAvantLaFin = self.framerate*self._tempsDeSimulation
    
        while(self._nombreStep < nombreStepAvantLaFin):
            start = time.time()
            if self._avance:
                vitesse = (self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._recule:
                vitesse = -(self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._stop:
                vitesse = 0
            else:
                raise Exception("Aucun mode actif pour la classe blenderManager")
            

            t = (self._nombreStep - self._debutVirage)/self.framerate
            self.vehicule.avance(vitesse, self._angleRoue, t, self.T0)
            global frameNb
            frameNb += 1
            self._nombreStep += 1
            tempsEcoule = time.time() - start
            if(tempsEcoule > 1/self.framerate):
                #c'est peut-être juste un breakpoint aussi, donc pas d'exception on continue
                pass
            else:
                time.sleep((5/self.framerate)-tempsEcoule)
        self.stop()
        return frameNb, nombreStepAvantLaFin
            #maintenant qu'on a la distance, le convertir en x, y et z

    #fonction qui permet de domir un temps X par rapport à la simulation et non au temps réel.
    def sleep(self, seconde):
        global frameNb
        target = (seconde*self.framerate)+frameNb
        while(frameNb < target):
            time.sleep(0.001)
            if(self.is_alive() is not True):
                raise Exception("La simulation est over")

    #fonction qui permet d'avancer
    def forward(self):
        global frameNb
        print(f"{frameNb} forward()")
        self._avance = True
        self._stop = False
        self._recule = False
    
    #fonction qui permet de reculer
    def backward(self):
        global frameNb
        print(f"{frameNb} backward()")
        self._avance = False
        self._stop = False
        self._recule = True

    #fonction qui permet de stop les roues du véhicule
    def stop(self):
        global frameNb
        print(f"{frameNb} stop()")
        self._avance = False
        self._stop = True
        self._recule = False

    #fonction qui permet d'ajuster un angle de virage
    def turn(self, angle):
        print(f"{frameNb} turn({angle})")
            #raise Exception(f"Angle invalide dans turn({angle})")
        self.vehicule.angle -= self.vehicule.angleVirage
        self._angleRoue = np.radians(angle-90) #-90 pour centrer à 0
        self._debutVirage = self._nombreStep-1
        self.T0 = copy.deepcopy(self.vehicule.position)

    #fonction qui attend de retrouver le centre de la ligne
    def wait_tile_center(self):
        global frameNb
        print(f"{frameNb} wait_tile_center()")
        while True:
            lt_status = self.read_digital()
            if lt_status[2] == 1:
                break
            if(self.is_alive() is not True):
                raise Exception("La simulation est over")
    
    #fonction qui remet les roues droites
    def turn_straight(self):
        global frameNb
        print(f"{frameNb} turn_straight()")
        self.turn(90)
        self.forward()

    #fonction qui tourne à gauche au maximum
    def turn_left(self):
        global frameNb
        print(f"{frameNb} turn_left()")
        self.forward()
        self.turn(135)

    #fonction qui tourne à droite au maximum
    def turn_right(self):
        global frameNb
        print(f"{frameNb} turn_right()")
        self.forward()
        self.turn(45)

    #fonction présente pour minimiser les changements dans le code du picar.
    def ready(self):
        pass

    #Fonction qui retourne -1 ou une distance en mètre d'un obstacle.
    def get_distance(self):
        distance = self.vehicule.sonar.detection(self.listeObj)
        global frameNb
        #print(f"{self.frameNb}  {distance} = get_distance()")
        return distance
    
    #fonction présente pour minimiser les changements dans le code du picar.
    def setup(self):
        pass


def test():
    blender = blenderManager(8, "crochet")
    print("tourne de 45", f" temps = {frameNb}")
    blender.turn(90)
    blender.speed = 100
    blender.start()
    blender.forward()
    blender.sleep(1)
    blender.stop()

#test()