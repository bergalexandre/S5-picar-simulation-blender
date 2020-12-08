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
    disableRotation = False

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
        self.bille = bille(x-0.005, y, z+0.020, scene, self.blenderObj)
        self.sonar = sonar(x-0.065, y, z+0.05, scene, self.blenderObj)
        self.suiveurligne = CapteurLigne(x-0.065, y, z, scene, self.blenderObj)
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
            self.angle -= self.angleVirage
            self.angleVirage = 0
            self.mouvementLocal([vitesse, 0, 0])
        else:
            self.disableRotation = True
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
        super().__init__(x, y, z, "undefined", scene, parent)
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
            for material in bpy.data.materials:
                material.user_clear()
                bpy.data.materials.remove(material)
            
            for obj in bpy.data.objects:
                obj.user_clear()
                bpy.data.objects.remove(obj)

        self.vehicule = vehicule(0, 0, 0, scene)
        self.T0 = self.vehicule.position[:]
        bpy.context.scene.frame_end = int(secondes*self.framerate)
        
        self._nombreStep = 0
        self.turn(90)

        #crée la ligne
        ligne = creationLigne.Ligne(nomDeLigne, getattr(creationLigne, nomDeLigne), 2)
        self.vehicule.suiveurligne.configureLigne(ligne)


    def read_digital(self):
        return self.vehicule.suiveurligne.detection()


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
                print("La simulation n'est pas assez performante, risque d'erreur")
                print(f"etape {self._nombreStep}")
            else:
                time.sleep((1/self.framerate)-tempsEcoule)

            #maintenant qu'on a la distance, le convertir en x, y et z

    def sleep(self, seconde):
        global frameNb
        target = (seconde*self.framerate)+frameNb
        while(frameNb < target):
            time.sleep(0.001)
            if(self.is_alive() is not True):
                return

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

    def turn(self, angle):
        if(angle < 45 or angle > 135):
            raise Exception(f"Angle invalide dans turn({angle})")
        self._angleRoue = np.radians(angle-90) #-90 pour centrer à 0
        self._debutVirage = self._nombreStep
        self.T0 = copy.deepcopy(self.vehicule.position)

#L = capteur_sonar.Check(objlist)
#print(f"obstacle detecte a {L}m")

blender = blenderManager(10, "crochet")
print("tourne de 45", f" temps = {frameNb}")
blender.turn(135)
blender.set_speed(20)
blender.start()
blender.forward()
blender.sleep(1)
print("avance pour 5s", f" temps = {frameNb}")
blender.turn(90)
blender.set_speed(50)
blender.forward()
blender.sleep(5)
print("tourne à 135", f" temps = {frameNb}")
blender.turn(135)
blender.set_speed(30)
blender.forward()
blender.join()
print("fini")