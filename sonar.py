import numpy as np
#import matplotlib.image as mpimg
#import matplotlib.pyplot as plt
import time
import bpy
import os
import time
from pathlib import Path
from threading import Thread
import math
from mathutils import Matrix

class blenderObject():
    scale = 1
    radius = 2
    name = "undefined" #name of the blender object
    frameNb = 0
    framerate = 100 #besoin de savoir ça?

    def __init__(self, x, y, z, name="undefined", scene=None):
        self.position = np.array([x*self.scale, y*self.scale, z*self.scale], dtype=np.float)
        self.rotation = np.array([0, 0, 0])
        self.name = name
        self.scene = scene
        if scene is not None:
            if not self.name in bpy.data.objects:
                path = Path(os.getcwd()) / "blender" / f"{self.name}.dae"
                bpy.ops.wm.collada_import(filepath=str(path), import_units=False, keep_bind_info=False)
            
            self.blenderObj = bpy.data.objects[self.name]
            self.blenderObj.animation_data_clear()
        
            #place l'objet à son point de départ
            self.scene.frame_set(self.frameNb)
            self.blenderObj.location = tuple(self.position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'
    
    def mouvementLocal(self, deltaPosition):
        if self.scene is not None:
            self.frameNb += 1
            self.scene.frame_set(self.frameNb)
            self.position += deltaPosition
            self.blenderObj.location = tuple(self.position/self.scale)
            self.blenderObj.keyframe_insert(data_path="location", index=-1)
            self.blenderObj.animation_data.action.fcurves[-1].keyframe_points[-1].interpolation = 'LINEAR'

    def show(self, fig, ax):
        ax.plot(self.position[0], self.position[1], "xr")



class vehicule(blenderObject):
    def __init__(self, x,y,z,scene):
        super().__init__(x,y,z, "vehicule", scene)
        self.bille = bille(x, y-0.08, z+0.015, scene)
        self.sonar = sonar(x, y-0.14, z, None)
        self.suiveurligne = suiveurligne(x+0.5, y, z, None)
        #ajout du sonar à l'avant
    
    def mouvementLocal(self, deltaPosition):
        super().mouvementLocal(deltaPosition)
        self.suiveurligne.mouvementLocal(deltaPosition)
        self.sonar.mouvementLocal(deltaPosition)

        #déterminer accélération x et y pis shooter ça à bille
        #je pense que deltaposition serait une accélération en fait
        self.bille.bougeBille(deltaPosition)
        

class bille(blenderObject):
    def __init__(self, x, y, z, scene):
        super().__init__(x, y, z, "bille", scene)
        #qu'est-ce qu'on a besoin de savoir? Accélération en x et y? angle en z? faire le z ou pas?

    def bougeBille(self, deltaPosition):
        #offset x à calculer avec la formule d'inertie de la bille
        offset_x = 0
        #offset y
        offset_y = 0

        positionBille = np.array([deltaPosition[0]+offset_x, deltaPosition[1]+offset_y, deltaPosition[2]])
        self.mouvementLocal(positionBille)

class suiveurligne(blenderObject):
    def __init__(self, x, y, z, scene):
        super().__init__(x, y, z, "undefined", scene)

class sonar(blenderObject): 
    def __init__(self, x, y, z, scene):
        self.scale = 100
        super().__init__(x,y,z, "undefined", scene)
        self.max_range = 4.5*self.scale #mètre
        self.angle = 30 # angle en degrées
        self.precision = 0.01*self.scale

        #vision du sonar
        largeur = np.int16(round(np.sin(np.radians(self.angle))*self.max_range, 2))
        milieu = np.int16((largeur)/2)+np.int16(0.04*self.scale) #le +0.04 pour être au milieu
        longueur = np.int16(self.max_range)

        self.onde = np.zeros((largeur, longueur))

        for angle_actuel in range(int(-self.angle/2), int(self.angle/2)):
            angleRad = np.radians(angle_actuel)
            nombre_de_step = np.int16(self.max_range/self.precision)
            for step in np.linspace(0, longueur-1, nombre_de_step, dtype=np.int16):
                step_x = np.int16(np.sin(angleRad)*step) + milieu
                step_y = np.int16(np.cos(angleRad)*step)
                self.onde[step_x][step_y] = 1


    def collisionBound(self, position1, position2):
        return np.abs(position1-position2) < self.radius

    def Check(self, blenderObjectList):
        for obj in blenderObjectList:
            for index, value in np.ndenumerate(self.onde):
                position_obj = obj.position[1]
                position_sonar = index[0]+self.position[1]
                if(self.collisionBound(position_obj, position_sonar) == True):
                    #vérification du y
                    position_obj = obj.position[0]
                    position_sonar = index[1]+self.position[0]
                    if(self.collisionBound(position_obj, position_sonar) == True):
                        #sleep pour simuler le temps réel
                        length = (index[0]**2 + index[1]**2)**0.5 #pythagore
                        C = 333.34 #m/s
                        time.sleep(2*(length/self.scale)/C)
                        return length/self.scale
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
    def __init__(self, secondes):
        super().__init__()
        self._tempsDeSimulation = secondes
        #delete tout les trucs
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False, confirm=False)
        #load blender scene
        scene = bpy.context.scene
        if scene is not None:
            scene.render.fps = self.framerate
        self.vehicule = vehicule(0, 0, 0, scene)
        bpy.context.scene.frame_end = int(secondes*self.framerate)


    def run(self):
        nombreStepAvantLaFin = self.framerate*self._tempsDeSimulation
        nombreStep = 0
        startTurn = 0
        tempsEcoule = time.time()
    
        while(nombreStep < nombreStepAvantLaFin):
            start = time.time()
            distanceX = 0
            if self._avance:
                if self._tourne:
                    if(self._rotationServo > 90):
                        radius = 0.15/np.sin(180-self._rotationServo) 
                        omega = ((self._foward_speed)*self._step*self._circonference_roue*self._rpsMax) / radius
                        degreeSec = np.degrees(omega)
                        distance = (np.cos(degreeSec*startTurn)) * self._step
                        distanceX = -(np.sin(degreeSec*startTurn)) * self._step
                    else:
                        radius = 0.15/np.sin(self._rotationServo) 
                        omega = ((self._foward_speed)*self._step*self._circonference_roue*self._rpsMax) / radius
                        degreeSec = np.degrees(omega)
                        distance = (np.cos(degreeSec*startTurn)) * self._step
                        distanceX = np.abs((np.sin(degreeSec*startTurn))) * self._step

                    startTurn += tempsEcoule
                    print(distance, distanceX)
                else:
                    distance = (self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._recule:
                distance = -(self._foward_speed)*self._step*self._circonference_roue*self._rpsMax
            elif self._stop:
                distance = 0
            else:
                raise Exception("Aucun mode active pour la classe blenderManager")
            
            
            #pas de rotation pour le moment à prendre en compte, donc je vais juste mettre la vitesse dans le y
            position = np.array([distanceX, distance, 0])
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
        self._tourne = False
    
    def backward(self):
        self._avance = False
        self._stop = False
        self._recule = True
        self._tourne = False

    def stop(self):
        self._avance = False
        self._stop = True
        self._recule = False
        self._tourne = False

    def turnForward(self):
        self._avance = True
        self._stop = False
        self._recule = False
        self._tourne = True

    #vitesse de 0 à 100
    def set_speed(self, speed):
        if(speed < 0 or speed > 100):
            raise Exception(f"Vitesse invalide dans set_speed({speed})")
        self._foward_speed = speed/100
    
    #angle de 45 à 135
    def turn(self, angle):
        if(angle < 45 or angle > 135):
            raise Exception(f"Angle invalide dans turn({angle})")
        self._rotationServo = angle


#L = capteur_sonar.Check(objlist)
#print(f"obstacle detecte a {L}m")

blender = blenderManager(60)
blender.start()
blender.set_speed(100)
blender.turnForward()
blender.turn(45)
time.sleep(5)
blender.stop()
time.sleep(2)
blender.turnForward()   
blender.turn(135)
time.sleep(5)
blender.join()

print("fini")