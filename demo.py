#!/usr/bin/env python
'''
**********************************************************************
* Filename    : line_follower
* Description : An example for sensor car kit to followe line
* Author      : Dream
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Dream    2016-09-21    New release
**********************************************************************
'''
import sys
#sinon il trouvera pas les modules custom si on change pas son path d'import
sys.path.insert(0, r"C:\Users\Alexandre.Bergeron\OneDrive - USherbrooke\university\projet\S5_projet_simulation")
import vehicule

sim = vehicule.blenderManager(30, "crochet")
time = sim
Line_Follower = sim
Ultrasonic_Avoidance = sim
front_wheels = sim
back_wheels = sim

#from lib.SunFounder_Line_Follower import Line_Follower
#from lib.SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance
#from picar import front_wheels
#from picar import back_wheels
#import time
from os import system
import random
import threading

#line follow
REFERENCES = [20.0, 20.0, 22.0, 22.0, 19.0]
forward_speed = 65
backward_speed = 65

delay = 0.0005

#fw = front_wheels.Front_Wheels(db='config')
#bw = back_wheels.Back_Wheels(db='config')
#lf = Line_Follower.Line_Follower()
fw = front_wheels
bw = back_wheels
lf = Line_Follower
ua = Ultrasonic_Avoidance

lf.references = REFERENCES
fw.ready()
bw.ready()
fw.turning_max = 45

a_step = 3
b_step = 10
c_step = 30
d_step = 45

#avoid
force_turning = 2    # 0 = random direction, 1 = force left, 2 = force right, 3 = orderdly


turn_distance = 3

timeout = 10
last_angle = 90
last_dir = 0

class active_moving():
	input = ''
	turning_angle = 40
	off_track_count = 0
	max_off_track_count = 40

	def __init__(self, sim=False):
		bw.start()
		fw.turn(90)
		bw.set_speed( 20)
		bw.forward()
		time.sleep(0.01)
		bw.set_speed( 0)
		bw.stop()

	def run(self):
		
		t = threading.Thread(target=self.manage_input)
		t.start()

		while self.input != 'stop':
			if self.input == 'sortie':
				self.input = ''
				t = threading.Thread(target=self.manage_input)
				t.start()
				self.sortie()
				
			self.follow()
			#self.avoid()
			time.sleep(delay)

		bw.stop()

	def manage_input(self):
		self.input = input('Instructions :')

	def turn(self, angle):
		fw.turn(angle)
		#self.fw.turn(angle//4)
		#self.fw.turn((angle//4) * 2)
		#self.fw.turn((angle//4) * 3)
		#self.fw.turn(angle)

	def accelerate(self, s):
		bw.forward()
		bw.set_speed( s)
		#inter = s - self.bw._speed
		#inter = inter // 4
		#self.bw.set_speed( self.bw._speed + inter
		#self.bw.set_speed( self.bw._speed + inter
		#self.bw.set_speed( self.bw._speed + inter
		#self.bw.set_speed( self.bw._speed + inter

	def go(self, d, f=True):
		cm_per_s = 11.5
		if f:
			bw.forward()
		else:
			bw.backward()

		bw.set_speed( 50) 
		time.sleep(d/cm_per_s)
		bw.stop()

	def turn90(self, R=True):
		if R:
			fw.turn_right()
		else:
			fw.turn_left()
		self.go(30)
		
	def follow(self):
		self.accelerate(forward_speed)
		lt_status_now = lf.read_digital()
		# Angle calculate
		if	lt_status_now == [0,0,1,0,0]:
			step = 0
		elif lt_status_now == [0,1,1,0,0] or lt_status_now == [0,0,1,1,0]:
			step = a_step
		elif lt_status_now == [0,1,0,0,0] or lt_status_now == [0,0,0,1,0]:
			step = b_step
		elif lt_status_now == [1,1,0,0,0] or lt_status_now == [0,0,0,1,1]:
			step = c_step
		elif lt_status_now == [1,0,0,0,0] or lt_status_now == [0,0,0,0,1]:
			step = d_step

		# Direction calculate
		if	lt_status_now == [0,0,1,0,0]:
			self.off_track_count = 0
			fw.turn(90)
		# turn right
		elif lt_status_now in ([0,1,1,0,0],[0,1,0,0,0],[1,1,0,0,0],[1,0,0,0,0]):
			self.off_track_count = 0
			self.turning_angle = int(90 - step)
		# turn left
		elif lt_status_now in ([0,0,1,1,0],[0,0,0,1,0],[0,0,0,1,1],[0,0,0,0,1]):
			self.off_track_count = 0
			self.turning_angle = int(90 + step)
		elif lt_status_now == [0,0,0,0,0]:
			self.off_track_count += 1
			if self.off_track_count > self.max_off_track_count:
				tmp_angle = (self.turning_angle-90)/abs(90-self.turning_angle)
				tmp_angle *= fw.turning_max
				self.accelerate(backward_speed)
				bw.backward()
				fw.turn(tmp_angle)

				lf.wait_tile_center()
				bw.stop()

				fw.turn(self.turning_angle)
				time.sleep(0.2)
				bw.forward()
				self.accelerate(forward_speed)
				time.sleep(0.2)

		else:
			self.off_track_count = 0

		self.turn(self.turning_angle)

	def sortie(self):
		bw.stop()
		self.turn90(R=False)
		self.turn90()
		self.go(20)
		fw.turn_right()
		bw.set_speed( 25)

		line = lf.read_digital()
		while line[0] or line[1] or line[2] or line[3] or line[4] != 1:
			line = lf.read_digital()

		bw.stop()


	def avoid(self):
		#avoid
		distance = ua.get_distance()
		if distance < turn_distance: # turn
			bw.stop()

			print('recule')

			#recule
			fw.turn_straight()
			bw.backward()
			self.accelerate(50)

			time.sleep(1.5)

			#ici

			line = lf.read_digital()
			while line[0] or line[1] or line[2] or line[3] or line[4] != 1:
				line = lf.read_digital()

	def rand_dir(self):
		global last_angle, last_dir
		if force_turning == 0:
			_dir = random.randint(0, 1)
		elif force_turning == 3:
			_dir = not last_dir
			last_dir = _dir
			print('last dir  %s' % last_dir)
		else:
			_dir = force_turning - 1
		angle = (90 - fw.turning_max) + (_dir * 2* fw.turning_max)
		last_angle = angle
		return angle

	def opposite_angle(self):
		global last_angle
		if last_angle < 90:
			angle = last_angle + 2* fw.turning_max
		else:
			angle = last_angle - 2* fw.turning_max
		last_angle = angle
		return angle

	def destroy(self):
		bw.stop()
		fw.turn(90)

class demo_AB():
	def __init__(self, sim=False):
		bw.start()
		self.temps = 0
		self.temps_max = 1

	def avoid(self):
		pass

	def follow(self):
		lecture = lf.read_digital()
		#if(lecture != derniereLecture):
		if(lecture[4] == 1):
			variation = 10
			nouvelAngle = angle + variation
			angle = nouvelAngle if nouvelAngle <= (90+fw.turning_max) else (90+fw.turning_max)
		if(lecture[3] == 1):
			variation = 5
			nouvelAngle = angle + variation
			angle = nouvelAngle if nouvelAngle <= (90+fw.turning_max) else (90+fw.turning_max)
		if(lecture[2] == 1):
			angle = 90
		if(lecture[1] == 1):
			variation = 5
			nouvelAngle = angle - variation
			angle = nouvelAngle if nouvelAngle >= (90-fw.turning_max) else (90-fw.turning_max)
		if(lecture[0] == 1):
			variation = 10
			nouvelAngle = angle - variation
			angle = nouvelAngle if nouvelAngle >= (90-fw.turning_max) else (90-fw.turning_max)

		if(sum(lecture) == 0):
			angle = 90
		if(sum(lecture) == 5):
			angle == 90
			stop()
		return angle

	def run(self):
		
		bw.set_speed(50)
		bw.forward()
		angle = 90
		dernierAngle = 0
		derniereLecture = [0,0,0,0,0]
		while(bw.is_alive()):
			
			self.follow()
			
			#si aucune ligne détecté, avance en ligne droite
			if(angle != dernierAngle):
				fw.turn(angle) 
				dernierAngle = angle
			

#demo = active_moving()
demo = demo_AB()
demo.run()