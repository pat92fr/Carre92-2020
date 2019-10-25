## TODO : Panda3D uses a configuration file named Config.prc.
## TODO : include IA into game class 

## PARAMETERS #################################################################

# simulator (calibrated from real world)
steering_clamp = 35.0      # degree
steering_increment = 360.0 # 160 degree per second

## CONSTANTS #################################################################

os_root_dir = 'c:/tmp/'
panda_root_dir = '/c/tmp/'
dataset_dir = 'dataset'
media_dir = 'mediaPR'

## GLOBALS ########################################################################

### https://github.com/jlevy44/UnrealAI/blob/master/CarAI/joshua_work/game/src/simulation.py

from my_math import *
import controlPR
import odometryPR
from trackPR import *

import numpy as np
import math
import random
import cv2
from os import mkdir

import socket
import asyncore

from direct.showbase.ShowBase import ShowBase
from direct.filter.CommonFilters import CommonFilters
from direct.gui.OnscreenText import OnscreenText
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.DirectSlider import DirectSlider
from direct.task import Task

from panda3d.core import *
from panda3d.core import Material
from panda3d.core import Spotlight
from panda3d.core import TransparencyAttrib
from panda3d.core import CollisionTraverser, CollisionNode
from panda3d.core import CollisionHandlerQueue, CollisionRay
from panda3d.core import LPoint3, LVector3, BitMask32
#from direct.actor.Actor import Actor
#from direct.interval.IntervalGlobal import Sequence
#from panda3d.core import Point3

from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletCylinderShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletVehicle

import panda3d.bullet as bullet
from panda3d.bullet import BulletDebugNode

'''''''''''''''''''''''''''''
    Telemetry handler
'''''''''''''''''''''''''''
telemetry_client_connected = False

class telemetry_handler(asyncore.dispatcher_with_send):
  
    def handle_read(self):
        pass

    def handle_close(self):
        global telemetry_client_connected
        print('Telemetry> connection closed.')
        telemetry_client_connected = False
        self.close()

'''''''''''''''''''''''
    Telemetry server
'''''''''''''''''''''
class telemetry_server(asyncore.dispatcher):
    
    def __init__(self, host, port):
        global telemetry_client_connected
        telemetry_client_connected = False
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
    
    def handle_accept(self):
        global telemetry_client_connected
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from ', repr(addr))
            self.handler = telemetry_handler(sock)
            telemetry_client_connected = True

    def sendTelemetry(self, data):
        global telemetry_client_connected
        if telemetry_client_connected:
            self.handler.send(data.encode("utf-8"))
            #self.flush()

'''''''''''''''''''''''
    Game App
'''''''''''''''''''''

# Macro-like function used to reduce the amount to code needed to create the
# on screen instructions
def genLabelText(text, i):
    return OnscreenText(text=text, parent=base.a2dTopLeft, pos=(0.07, -.06 * i - 0.1),
                        fg=(1, 1, 1, 1), align=TextNode.ALeft, shadow=(0, 0, 0, 0.5), scale=.05)

class Simulator(ShowBase):

	def __init__(self, rc, odom):

		ShowBase.__init__(self)

		# external robot controller
		self.robot_controller = rc
		self.robot_odometry = odom

        # Window
		winprops = WindowProperties()
		winprops.setSize(1024, 576)
		base.win.requestProperties(winprops) 
		base.setFrameRateMeter(True)

		# OSD
		self.dr = self.win.makeDisplayRegion()
		self.dr.setSort(20)
        
        # OSD menu
		self.exitText = genLabelText("q: Exit", 0)
		self.autoText = genLabelText("a: Autopilot", 1)
		self.manualText = genLabelText("m: Manualpilot", 2)
		self.recordText = genLabelText("r: Record", 3)
		self.homeText = genLabelText("h: Home", 4)

        # application state
		self.quit = False
		self.autopilot = False
		self.recording = False

		# keyboard events
		self.accept("q",     self.doQuit)
		self.accept("a",     self.doAutopilot)
		self.accept("m",     self.doManualpilot)
		self.accept("r",     self.doRecord)
		self.accept("h",     self.setHome)

        # gamepad
		self.gamepad = None
		devices = self.devices.getDevices(InputDevice.DeviceClass.gamepad)
		if devices:
			print("Devices founds..." + str(len(devices)))
            # gamepad yet.
			if devices[0].device_class == InputDevice.DeviceClass.gamepad and not self.gamepad:
				print("Found %s" % (devices[0]))
				self.gamepad = devices[0]

        # Disable the camera trackball controls.
		self.disableMouse()

		# load map
		self.load_PR_map()

		# physics
		# debugNode = BulletDebugNode('Debug')
		# debugNode.showWireframe(True)
		# debugNode.showConstraints(True)
		# debugNode.showBoundingBoxes(False)
		# debugNode.showNormals(False)
		# debugNP = render.attachNewNode(debugNode)
		# debugNP.show()
		self.world = BulletWorld()
		self.world.setGravity(Vec3(0, 0, -9.81))
		#self.world.setDebugNode(debugNP.node())
		self.worldNP = self.render.attachNewNode('World')

		# ground physics
		self.planePhysShape = BulletPlaneShape(Vec3(0, 0, 1), 0)
		self.planePhysNode = BulletRigidBodyNode('Ground')
		self.planePhysNode.addShape(self.planePhysShape)
		self.planeNP = self.worldNP.attachNewNode(self.planePhysNode)
		self.planeNP.setPos(0, 0, 0)
		self.world.attachRigidBody(self.planePhysNode)

        # make car node (no model attached)
		self.chassisPhysShape = BulletBoxShape(Vec3(0.08, 0.18, 0.02)) # half dim in every directions
		self.chassisTS = TransformState.makePos(Point3(0, 0, 0.02)) # bottom of the car is at 0 altitude
		self.chassisPhysNode = BulletRigidBodyNode('Vehicle')
		self.chassisPhysNode.addShape(self.chassisPhysShape, self.chassisTS)
		self.chassisPhysNode.setMass(26.0) #lbs
		self.chassisPhysNode.setDeactivationEnabled(False)
		self.chassisNP = self.worldNP.attachNewNode(self.chassisPhysNode)
		self.world.attachRigidBody(self.chassisPhysNode)

		model = loader.loadModel(panda_root_dir + media_dir + '/' + 'chassis.bam')
		model.setHpr(180, 0, 180)
		model.setPos(0, 0, 0.1)
		model.reparentTo(self.chassisNP)

		self.vehicle = BulletVehicle(self.world, self.chassisPhysNode)
		self.vehicle.setCoordinateSystem(bullet.ZUp)
		self.world.attachVehicle(self.vehicle)

		voie = 0.44
		empattement = 0.48
		hauteur = 0.35

		FRwheelNP = loader.loadModel(panda_root_dir + media_dir + '/' + 'wheel.bam')
		FRwheelNP.reparentTo(self.worldNP)
		self.addWheel(Point3(voie/2.0, empattement/2.0, hauteur), True, FRwheelNP)

		FLwheelNP = loader.loadModel(panda_root_dir + media_dir + '/' + 'wheel.bam')
		FLwheelNP.reparentTo(self.worldNP)
		self.addWheel(Point3(-voie/2.0, empattement/2.0, hauteur), True, FLwheelNP)

		RRwheelNP = loader.loadModel(panda_root_dir + media_dir + '/' + 'wheel.bam')
		RRwheelNP.reparentTo(self.worldNP)
		self.addWheel(Point3(voie/2.0, -empattement/2.0, hauteur), False, RRwheelNP)

		RLwheelNP = loader.loadModel(panda_root_dir + media_dir + '/' + 'wheel.bam')
		RLwheelNP.reparentTo(self.worldNP)
		self.addWheel(Point3(-voie/2.0, -empattement/2.0, hauteur), False, RLwheelNP)

		# Collision solids/nodes for chassis
		self.chassisCN = CollisionNode('chassisCN')
		self.chassisCS = CollisionBox((0,0,0), 0.1, 0.2, 0.1) # largeur x longeur x hauteur
		self.chassisCN.addSolid(self.chassisCS)
		self.chassisCN.setIntoCollideMask(BitMask32.allOff())
		self.chassisCN.setFromCollideMask(BitMask32.bit(1))
		self.chassisCNP = self.chassisNP.attachNewNode(self.chassisCN)
		#self.chassisCNP.show()

		# Collision for Lidar settings
		self.lidar_maximum_distance = 10.0 #m

		# Collision solids/nodes for Lidar
		self.LidarCN = []
		self.LidarCNP = []
		self.lidar_distance = {}
		for angle in range(-135,+135,1):
			cn = CollisionNode('LidarCN'+str(angle))
			cs = CollisionSegment((0.0,0.0,0.05),(self.lidar_maximum_distance*math.cos(math.radians(angle+90)),self.lidar_maximum_distance*math.sin(math.radians(angle+90)),0.0)) 
			cn.addSolid(cs)
			cn.setIntoCollideMask(BitMask32.allOff())
			cn.setFromCollideMask(BitMask32.bit(2))
			cnp = self.chassisNP.attachNewNode(cn)
			#cnp.show() ###############################################################
			self.LidarCN.append(cn)
			self.LidarCNP.append(cnp)
			#print(cnp.node().name)
			self.lidar_distance[cnp.node().name] = 0.0

		self.chassisNP.setPos(start_position[0], start_position[1], 0.4)
		self.chassisNP.setHpr(start_position[2]-90.0, 0.0, 0.0)

		# camera
		self.camLens.setFov(100)
		self.camLens.setNear(0.01)
		# camera for suspension tuning
		#self.camera.setPos(0.5,-0.7,0.20)
		#self.camera.setHpr(20,-20,0)
		
		####self.camera.setPos(0.0,-0.15,0.25)
		###self.camera.setPos(0.0,-2.0,2.0)
		#self.camera.setPos(0.0,-0.5,0.5)
		self.camera.setPos(0.0,-3.0,3.0)
		#self.camera.setPos(0.0,0.05,0.22) # REFERENCE
		self.camera.setHpr(0,-30,0)
		self.camera.reparentTo(self.chassisNP)

		# tasks
		self.taskMgr.add(self.physics_task, 'updatePhysics')

		# Collisions
		self.ctraverser = CollisionTraverser()  # Make a traverser
		self.cqueue = CollisionHandlerQueue()  # Make a handler
		self.ctraverser.addCollider(self.chassisCNP, self.cqueue)
		for cnp in self.LidarCNP:
			self.ctraverser.addCollider(cnp, self.cqueue)

		# simulator odometry (ground truth)
		self.last_position = self.chassisNP.getPos()
		self.current_position = self.chassisNP.getPos()
		self.heading =  self.chassisNP.getH()+90.0
		self.last_heading =  self.chassisNP.getH()+90.0
		self.total_distance = 0.0
		self.lap_distance = 0.0

		# simulator speed control state
		self.delta_distance = 0.0 # m
		self.actual_speed_ms = 0.0 # m/s
		self.actual_speed_kmh = 0.0

		# simulator speed control state
		self.delta_heading = 0.0 # deg
		self.actual_rotation_speed_dps = 0.0 # dps
		self.gyro_bias = 0.0 #random.uniform(-0.2,0.2) #dps

		# simulator manual control
		self.current_speed_ms = 0.0
		self.max_speed_ms = 3.0
		self.acceleration = 0.02
		self.deceleration = 0.02

		# simulator steering control state
		self.last_steering = 0.0       # degree
		self.steering_clamp = steering_clamp      # degree
		self.steering_increment = steering_increment # degree per second

		# controller output
		self.steering = 0.0
		self.throttle = 0.0
		self.engineForce = 0.0
		self.brakeForce = 0.0

		# OSD graphics
		self.speed_o_meter = OnscreenText(text="0km/h", pos=(1.4,0.80), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.25)
		self.lap_timer = globalClock.getFrameTime()
		self.lap_timer_text = OnscreenText(text=str(round(globalClock.getFrameTime(),1)) +"s", pos=(1.4,0.60), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.15)
		self.best_lap_timer = 999.9
		self.best_lap_timer_text = OnscreenText(text=str(round(self.best_lap_timer,1)) +"s", pos=(1.4,0.50), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.05)
		self.lap_counter = 0
		# odometry
		self.lap_distance_text = OnscreenText(text=str(round(self.lap_distance,1)) +"m", pos=(1.7,0.50), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.05)
		self.heading_text = OnscreenText(text=str(int(self.heading)) +"deg", pos=(1.7,0.4), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.05)
		# controler settings
		self.slider_max_speed = DirectSlider(range=(0,14), value=self.robot_controller.max_speed_ms, pageSize=0.1, command=self.slider_max_speed_change, scale=0.4, pos = (0.0,0.0,0.90))
		self.text_max_speed = OnscreenText(text="Vmax " + str(self.robot_controller.max_speed_ms)+"m/s", fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.04, pos=(-0.55,0.90))
		self.slider_steering_k_speed = DirectSlider(range=(0,2), value=self.robot_controller.steering_k_speed, pageSize=0.1, command=self.slider_steering_k_speed_change, scale=0.4, pos = (0.0,0.0,0.85))
		self.text_steering_k_speed = OnscreenText(text="Steering K speed " + str(round(self.robot_controller.steering_k_speed,2)), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.04, pos=(-0.55,0.85))
		self.slider_lidar_direction_kp = DirectSlider(range=(0.0,0.1), value=self.robot_controller.pid_wall_following.kp, pageSize=0.01, command=self.slider_lidar_direction_kp_change, scale=0.4, pos = (0.0,0.0,0.80))
		self.text_lidar_direction_kp = OnscreenText(text="Lidar Direction Kp " + str(round(self.robot_controller.pid_wall_following.kp,2)), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.04, pos=(-0.55,0.80))
		self.slider_lidar_direction_kd = DirectSlider(range=(0.0,1.0), value=self.robot_controller.pid_wall_following.kd, pageSize=0.01, command=self.slider_lidar_direction_kd_change, scale=0.4, pos = (0.0,0.0,0.75))
		self.text_lidar_direction_kd = OnscreenText(text="Lidar Direction Kd " + str(round(self.robot_controller.pid_wall_following.kd,2)), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.04, pos=(-0.55,0.75))
		self.slider_lidar_direction_k_speed = DirectSlider(range=(0,2), value=self.robot_controller.lidar_direction_k_speed, pageSize=0.1, command=self.slider_lidar_direction_k_speed_change, scale=0.4, pos = (0.0,0.0,0.70))
		self.text_lidar_direction_k_speed = OnscreenText(text="Lidar Direction K speed " + str(round(self.robot_controller.lidar_direction_k_speed,2)), fg=(1, 1, 1, 1), align=TextNode.ARight, shadow=(0, 0, 0, 0.5), scale=.04, pos=(-0.55,0.70))



	def addWheel(self, pos, front, np):
		wheel = self.vehicle.createWheel()
		#http://blender3d.org.ua/forum/game/iwe/upload/Vehicle_Simulation_With_Bullet.pdf
		wheel.setNode(np.node())
		wheel.setChassisConnectionPointCs(pos)
		wheel.setFrontWheel(front)
		wheel.setWheelDirectionCs(Vec3(0, 0, -1))
		wheel.setWheelAxleCs(Vec3(1, 0, 0))
		wheel.setWheelRadius(0.1)
		wheel.setMaxSuspensionTravelCm(15.0) #cm
		wheel.setSuspensionStiffness(80.0)
		wheel.setWheelsDampingRelaxation(0.8)
		wheel.setWheelsDampingCompression(0.6) 
		wheel.setFrictionSlip(40.0);
		wheel.setRollInfluence(0.1)

	def slider_max_speed_change(self):
		self.robot_controller.max_speed_ms = float(self.slider_max_speed['value'])
		self.text_max_speed.setText("Vmax " + str(round(self.robot_controller.max_speed_ms,1))+"m/s")

	def slider_steering_k_speed_change(self):
		self.robot_controller.steering_k_speed = float(self.slider_steering_k_speed['value'])
		self.text_steering_k_speed.setText("Steering K speed " + str(round(self.robot_controller.steering_k_speed,2)))

	def slider_lidar_direction_kp_change(self):
		self.robot_controller.pid_wall_following.kp = float(self.slider_lidar_direction_kp['value'])
		self.text_lidar_direction_kp.setText("LIDAR Direction Kp " + str(round(self.robot_controller.pid_wall_following.kp,2)))

	def slider_lidar_direction_kd_change(self):
		self.robot_controller.pid_wall_following.kd = float(self.slider_lidar_direction_kd['value'])
		self.text_lidar_direction_kd.setText("LIDAR Direction Kd " + str(round(self.robot_controller.pid_wall_following.kd,2)))

	def slider_lidar_direction_k_speed_change(self):
		self.robot_controller.lidar_direction_k_speed = float(self.slider_lidar_direction_k_speed['value'])
		self.text_lidar_direction_k_speed.setText("LIDAR Direction K speed " + str(round(self.robot_controller.lidar_direction_k_speed,2)))

	def physics_task(self, task):

		dt = globalClock.getDt()

		# reset wall following state
		for ld in self.lidar_distance:
			self.lidar_distance[ld]=self.lidar_maximum_distance

		self.ctraverser.traverse(render)
		self.ctraverser.showCollisions(render)
		self.cqueue.sortEntries()
		for entry in self.cqueue.getEntries():
			for cnp in self.LidarCNP:
				if  entry.getFromNodePath() == cnp:
					point = entry.getSurfacePoint(render)
					current = self.chassisNP.getPos()
					#current = cnp.getPos()
					distance = (point-current).length()
					if self.lidar_distance[cnp.node().name] > distance:
						self.lidar_distance[cnp.node().name] = distance
			if entry.getIntoNodePath() == self.archCNP:
				if globalClock.getFrameTime() > self.lap_timer + 3.0:
					print(str(round(globalClock.getFrameTime()-self.lap_timer,1)) +"s")
					if self.lap_counter > 0:
						self.best_lap_timer = min(globalClock.getFrameTime()-self.lap_timer,self.best_lap_timer)
					self.lap_timer = globalClock.getFrameTime()
					self.lap_counter += 1
					self.lap_distance = 0.0
					print(len(self.robot_odometry.map))
		self.lap_timer_text.setText(text=str(round(globalClock.getFrameTime()-self.lap_timer,1)) +"s")
		self.best_lap_timer_text.setText(text=str(round(self.best_lap_timer,1)) +"s")


		#print(self.lidar_distance)

		# if gamepad detected in human mode
		# if self.gamepad and not self.autopilot:
		# 	# gamepad inputs
		# 	self.direction = self.gamepad.findAxis(InputDevice.Axis.right_x).value
		# 	self.throttle = self.gamepad.findAxis(InputDevice.Axis.left_x ).value

		# 	# center
		# 	self.direction -= 0.38
		# 	self.throttle -= 0.41
		# 	print(str(self.direction) + "   " + str(self.throttle))
		    
		# 	# inertial
		# 	self.last_steering = self.steering
		# 	self.steering = self.direction * self.steeringClamp
		# 	if self.steering > self.last_steering:
		# 		self.steering = min(self.steering, self.last_steering+dt*self.steeringIncrement)
		# 	if self.steering < self.last_steering:
		# 		self.steering = max(self.steering, self.last_steering-dt*self.steeringIncrement)


		# 	# clamp
		# 	self.steering = min(self.steering, self.steeringClamp)
		# 	self.steering = max(self.steering, -self.steeringClamp)


		# 	if self.throttle > 0:
		# 		self.engineForce = self.throttle * 1.0
		# 		self.engineForce = min(self.engineForce, 0.6)
		# 		self.engineForce = max(self.engineForce, 0.0)
		# 		self.brakeForce = 0.0
		# 	else:
		# 		self.brakeForce = -self.throttle * 1.0
		# 		self.brakeForce = min(self.engineForce, 2.0)
		# 		self.brakeForce = max(self.engineForce, 0.0)
		# 		self.engineForce = 0.0

		# elif not self.gamepad and not self.autopilot:


		# actual speed computation
		self.last_position = self.current_position
		self.current_position = self.chassisNP.getPos()
		self.delta_distance = math.sqrt( 
			(self.current_position.getX()-self.last_position.getX())*(self.current_position.getX()-self.last_position.getX()) + 
			(self.current_position.getY()-self.last_position.getY())*(self.current_position.getY()-self.last_position.getY())
			)
		self.total_distance += self.delta_distance
		self.lap_distance += self.delta_distance
		#if  dt != 0:
		self.actual_speed_ms = self.delta_distance/dt
		self.actual_speed_kmh = self.actual_speed_kmh * 0.9 + 0.1 * self.actual_speed_ms*60*60/1000
		self.speed_o_meter.setText(str(int(self.actual_speed_kmh))+ "km/h")
		self.lap_distance_text.setText(str(int(self.lap_distance))+ "m")

		# heading
		self.last_heading = self.heading
		self.heading = self.chassisNP.getH()+90.0
		self.delta_heading = self.heading - self.last_heading
		self.actual_rotation_speed_dps = self.delta_heading/dt
		self.heading_text.setText('(' + str(round(self.current_position.getX(),1)) +',' + str(round(self.current_position.getY(),1)) +')   ' + str(int(self.heading))+ "deg")

		# odometry
		self.robot_odometry.process(
			dt,
			self.actual_speed_ms, #+ random.uniform(-0.2,0.2),
			self.actual_rotation_speed_dps, #+ random.uniform(-2.0,2.0) + self.gyro_bias*dt,
			self.lidar_distance
		)

		# display virtual anchor
		for va in self.virtualanchorNodePath :
			va.setPos(1000.0,1000.0,1000.0)
		virtual_anchor_index = 0
		for a in self.robot_odometry.landmarks :
			px = self.current_position.getX() + a[1]*math.cos(math.radians(self.heading+a[0]))
			py = self.current_position.getY() + a[1]*math.sin(math.radians(self.heading+a[0]))
			self.virtualanchorNodePath[virtual_anchor_index].setPos(px,py,0.0)
			virtual_anchor_index += 1

		# display virtual odometry
		self.virtualodometryNodePath.setPos(self.robot_odometry.odom_with_slam.x,self.robot_odometry.odom_with_slam.y,0.0)

		# reset control state
		self.engineForce = 0.0
		self.brakeForce = 0.0

		# chose controller
		if not self.autopilot: # manual controller

			# key pressed
			self.up_button = self.mouseWatcherNode.isButtonDown(KeyboardButton.up())
			self.down_button = self.mouseWatcherNode.isButtonDown(KeyboardButton.down())
			self.left_button = self.mouseWatcherNode.isButtonDown(KeyboardButton.left())
			self.right_button = self.mouseWatcherNode.isButtonDown(KeyboardButton.right())

			# keys to speed
			if self.up_button and not self.down_button:
				self.throttle += self.acceleration
				self.throttle = min(self.throttle, 1.0)
			if not self.up_button and self.down_button:
				self.throttle -= self.deceleration
				self.throttle = max(self.throttle, -1.0)
			if not self.up_button and not self.down_button:
				self.throttle = 0.0
			
			# keys to steering
			if self.left_button and not self.right_button:
				self.steering += dt*self.steering_increment*0.15
				self.steering = min(self.steering, self.steering_clamp)
			if not self.left_button and self.right_button:
				self.steering -= dt*self.steering_increment*0.15
				self.steering = max(self.steering, -self.steering_clamp)
			if not self.left_button and not self.right_button:
				if self.steering < 0:
					self.steering += dt*self.steering_increment*0.4
					self.steering = min(self.steering, 0)
				if self.steering > 0:
					self.steering -= dt*self.steering_increment*0.4
					self.steering = max(self.steering, 0)


		elif self.autopilot: # automatic controller

			# simulator steering
			self.last_steering = self.steering

			# add picture here
			# add picture here
			# add picture here
			# add picture here

			# call external robot controller
			self.steering, self.throttle = self.robot_controller.process(
				dt,
				self.actual_speed_ms,
				self.actual_rotation_speed_dps,
				self.total_distance,
				self.lidar_distance,
				self.current_position.getX(),
				self.current_position.getY(),
				self.heading
			)
			#print(self.heading)

			# simulator steering
			self.steering *= self.steering_clamp
			# simulator steering steering inertia
			if self.steering > self.last_steering:
				self.steering = min(self.steering, self.last_steering+dt*self.steering_increment)
			if self.steering < self.last_steering:
				self.steering = max(self.steering, self.last_steering-dt*self.steering_increment)
			# simulator steering steering clamp
			self.steering = constraint(self.steering, -self.steering_clamp, self.steering_clamp)

		#print(self.throttle)
		max_engine_force = 60.0
		max_brake_force = 10.0
		if self.throttle >= 0.0:
			self.engineForce = self.throttle*max_engine_force
			self.engineForce = min(self.engineForce, max_engine_force)
			self.engineForce = max(self.engineForce, 0.0)
			self.brakeForce = 0.0
		else:
			self.brakeForce = -self.throttle*max_brake_force
			self.brakeForce = min(self.brakeForce, max_brake_force)
			self.brakeForce = max(self.brakeForce, 0.0)
			self.engineForce = 0.0

		# Apply steering to front wheels
		self.vehicle.setSteeringValue(self.steering, 0);
		self.vehicle.setSteeringValue(self.steering, 1);

		# Apply engine and brake to rear wheels
		self.vehicle.applyEngineForce(self.engineForce, 2);
		self.vehicle.applyEngineForce(self.engineForce, 3);
		#self.vehicle.applyEngineForce(self.engineForce, 2);
		#self.vehicle.applyEngineForce(self.engineForce, 3);
		#self.vehicle.setBrake(self.brakeForce, 0);
		#self.vehicle.setBrake(self.brakeForce, 1);
		self.vehicle.setBrake(self.brakeForce, 2);
		self.vehicle.setBrake(self.brakeForce, 3);

		self.world.doPhysics(dt)
		#world.doPhysics(dt, 10, 1.0/180.0)

		return task.cont

	def load_PR_map(self):

        # load track
		self.trackNodePath = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'ground.bam')
		self.trackNodePath.setScale(1.0, 1.0, 1.0)
		self.trackNodePath.setPos(0.0, 0.0, 0.0)
		self.trackNodePath.setHpr(0.0, 90.0, 0.0)
		self.trackNodePath.reparentTo(self.render)

		# load arch
		self.archNodePath = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'arch.bam')
		self.archNodePath.setScale(2.6, 2.6, 2.6)
		self.archNodePath.setPos(-17.2, 22.5, 0.0)
		self.archNodePath.setHpr(0.0, 90.0, 90.0)
		self.archNodePath.reparentTo(self.render)

		# Collision solids/nodes for arch
		self.archCN = CollisionNode('archCN')
		self.archCS = CollisionBox((-9.0,0.0,1.0), 0.2, 1.0, 1.0)
		self.archCN.addSolid(self.archCS)
		self.archCN.setIntoCollideMask(BitMask32.bit(1))
		self.archCN.setFromCollideMask(BitMask32.allOff())
		self.archCNP = self.archNodePath.attachNewNode(self.archCN)
		#self.archCNP.show()

        # load anchor and do collide
		self.anchorNodePath = []
		self.anchorCollisionNodePath = []
		for op in anchor_position :
			onp = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'plot.bam')
			onp.setScale(2.0, 2.0, 2.0)
			onp.setHpr(0.0, 180.0, 0.0)
			onp.setPos(op)
			onp.reparentTo(self.render)
			cn = CollisionNode('anchorCN')
			cs = CollisionSphere(0.0,0.0,-0.1,0.05) # scale is 2, then R is 0.5 to get 0.1 at the end ! 
			cn.addSolid(cs)
			cn.setIntoCollideMask(BitMask32.bit(2))
			cn.setFromCollideMask(BitMask32.allOff())
			cnp = onp.attachNewNode(cn)
			#cnp.show() ###############################################################
			self.anchorNodePath.append(onp)
			self.anchorCollisionNodePath.append(cnp)

		# load anchros (odometry)
		self.virtualanchorNodePath = []
		for index in range(20) :
			onp = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'vplot.bam')
			onp.setScale(2.0, 2.0, 2.0)
			onp.setHpr(0.0, 90.0, 0.0)
			onp.setPos(100.0,100.0,100.0)
			onp.reparentTo(self.render)
			self.virtualanchorNodePath.append(onp)

		# load virtual robot odometry
		self.virtualodometryNodePath = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'vplot.bam')
		self.virtualodometryNodePath.setScale(2.0, 2.0, 2.0)
		self.virtualodometryNodePath.setHpr(0.0, 90.0, 0.0)
		self.virtualodometryNodePath.setPos(100.0,100.0,100.0)
		self.virtualodometryNodePath.reparentTo(self.render)

		# load wp
		self.waypointNodePath = []
		for op in wp_position :
			onp = self.loader.loadModel(panda_root_dir + media_dir + '/' + 'waypoint.bam')
			onp.setScale(1.0, 1.0, 1.0)
			onp.setHpr(0.0, 90.0, op[2])
			onp.setPos(op[0],op[1],0.01)	
			onp.reparentTo(self.render)
			self.waypointNodePath.append(onp)

        # Lights
		self.render.clearLight()
		# Ambiant Light
		self.alight = AmbientLight('ambientLight')
		self.alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
		self.alightNP = self.render.attachNewNode(self.alight)
		self.render.setLight(self.alightNP)
		# Directional light
		self.directionalLight = DirectionalLight('directionalLight')
		self.directionalLight.setDirection(Vec3(1, 1, -2))
		self.directionalLight.setSpecularColor((0.8, 0.8, 0.8, 1))
		self.temperature = 6500
		self.directionalLight.setColorTemperature(self.temperature)
		self.directionalLightNP = self.render.attachNewNode(self.directionalLight)
		self.render.setLight(self.directionalLightNP)
        # Per-pixel lighting and shadows are initially off
		#self.directionalLightNP.node().setShadowCaster(True, 512, 512)
		self.render.setShaderAuto()

	def doQuit(self):
        # De-initialization code goes here!
		self.quit = True

	def doAutopilot(self):
        # De-initialization code goes here!
		self.autopilot = True

	def doManualpilot(self):
        # De-initialization code goes here!
		self.autopilot = False

	def doRecord(self):
        # De-initialization code goes here!
		self.recording = True

	def setHome(self):
		self.chassisNP.setPos(start_position[0], start_position[1], 0.4)
		self.chassisNP.setHpr(start_position[2]-90.0, 0.0, 0.0)
		self.lap_counter = 0
		self.total_distance = 0
		self.lap_distance = 0

	def get_camera_image(self, requested_format=None):
		"""
		Returns the camera's image, which is of type uint8 and has values
		between 0 and 255.
		The 'requested_format' argument should specify in which order the
		components of the image must be. For example, valid format strings are
		"RGBA" and "BGRA". By default, Panda's internal format "BGRA" is used,
		in which case no data is copied over.
		"""
		tex = self.dr.getScreenshot()
		if requested_format is None:
		    data = tex.getRamImage()
		else:
		    data = tex.getRamImageAs(requested_format)
		image = np.frombuffer(data, np.uint8)  # use data.get_data() instead of data in python 2
		image.shape = (tex.getYSize(), tex.getXSize(), tex.getNumComponents())
		image = np.flipud(image)
		return image

## MAIN ########################################################################

print("Init telemetry server...")
tserver = telemetry_server("192.168.1.34", 7001)
#tserver = telemetry_server("192.168.43.5", 7001)
#tserver = telemetry_server("192.168.1.11", 7001)
print("Done!")

print("Create dataset file...")
try:
    mkdir(os_root_dir+dataset_dir)
except FileExistsError:
    pass
dataset_file = open(os_root_dir+dataset_dir+'/'+'dataset.txt',  'w')
print("Done!")

print("Init external robot controller...")
rc = controlPR.robot_controller(); 
print("Done!")

print("Init external robot odometry...")
odom = odometryPR.robot_odometry(); 
print("Done!")

print("Init sim engine...")
app = Simulator(rc,odom)
print("Done!")

# log
data_logger = open("log.txt","w")

# game loop
counter = 0
record_counter = 0
print("Start sim engine loop...")
while not app.quit:
	# Non blocking call
	asyncore.loop(timeout=0, count=1)
	# game tick
	taskMgr.step()
	# # get picture for CNN
	# frame_orign = app.get_camera_image()
	# # supress alpha channel
	# frame_orign = frame_orign[:, :, 0:3] 
	# #resize to CNN input format
	# frame_orign = cv2.resize(frame_orign, (160, 90),   interpolation = cv2.INTER_AREA)
	# # smotth and gray scale
	# frame = cv2.blur(frame_orign,(3,3))
	# frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	# # reshape for CNN
	# frame = frame.reshape(90,160,1)
	# # use CNN
	# app.robot_controller.frame_update(frame.reshape(1,90,160,1))
	# dataset recording
	if app.recording and counter != 0: #and not app.autopilot : # first frame buffer is empty, skip it!
		filename = dataset_dir + '/render_' + str(record_counter) + '.jpg'
		cv2.imwrite(root_dir + '/' + filename, frame_orign) 
		dataset_file.write(filename +';' + str(int(128.0-app.steering*127.0*1.4)) + ';' + str(int(app.throttle*127.0*1.4+128.0)) + '\n') # *1.4 gain
		dataset_file.flush()
		record_counter += 1
	# Telemetry
	if True:
	#if counter % 2 == 0:
		msg = str(int(counter/2)) + ';'

		#msg += str( float(app.robot_controller.target_speed_ms) ) + ';'
		msg += str( float(app.robot_controller.current_speed_ms) ) + ';'
		msg += str( float(app.robot_controller.actual_speed_ms) ) + ';' 
		msg += str( float(app.robot_controller.actual_speed_error_ms) ) + ';'
		msg += str( float(app.throttle) ) + ';' 

		msg += str( float(app.robot_controller.actual_lidar_direction_error) ) + ';'
		msg += str( float(app.robot_controller.pid_wall) ) + ';'
		msg += str( float(app.robot_controller.ratio_ai*10) ) + ';' 
		msg += str( float(app.steering) ) 
		
		msg_length = str(len(msg)).ljust(4)
		tserver.sendTelemetry(msg_length)
		tserver.sendTelemetry(msg)
	counter += 1
	#print(telemetry_client_connected)

	# log
	data_logger.write(

			str(round(app.current_position.getX(),2)) + ";" +
			str(round(app.current_position.getY(),2)) + ";" +
			str(round(app.heading,2)) + ";" +

			str(round(odom.odom.x,2)) + ";" +
			str(round(odom.odom.y,2)) + ";" +
			str(round(odom.odom.h,2)) + ";" +


			str(round(odom.odom_with_slam.x,2)) + ";" +
			str(round(odom.odom_with_slam.y,2)) + ";" +
			str(round(odom.odom_with_slam.h,2)) + "\n"
		)
	data_logger.flush()

dataset_file.close()
print('m:' + str(record_counter))

    
  


