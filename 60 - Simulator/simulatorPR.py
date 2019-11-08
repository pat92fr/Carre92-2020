from simulatorEngine import *

import controlPR
import odometryPR

from my_math import *
from trackPR import *
from ekf_slam import *

## CONSTANTS #################################################################

os_root_dir = 'c:/tmp/'
panda_root_dir = '/c/tmp/'
dataset_dir = 'dataset'
track_media_dir = 'mediaPR'

## GLOBALS ########################################################################

class SimulatorPR(Simulator):

	def __init__(self, start_position, rc, odom):

		Simulator.__init__(self, start_position, arch_position, rc, odom)

		# load map
        # load track
		self.trackNodePath = self.loader.loadModel(panda_root_dir + track_media_dir + '/' + 'ground.bam')
		self.trackNodePath.setScale(1.0, 1.0, 1.0)
		self.trackNodePath.setPos(0.0, 0.0, 0.0)
		self.trackNodePath.setHpr(0.0, 90.0, 0.0)
		self.trackNodePath.reparentTo(self.render)


        # load anchor and do collide
		self.anchorNodePath = []
		self.anchorCollisionNodePath = []
		for op in anchor_position :
			onp = self.loader.loadModel(panda_root_dir + track_media_dir + '/' + 'plot.bam')
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
		for index in range(50) :
			onp = self.loader.loadModel(panda_root_dir + track_media_dir + '/' + 'vplot.bam')
			onp.setScale(2.0, 2.0, 2.0)
			onp.setHpr(0.0, 90.0, 0.0)
			onp.setPos(100.0,100.0,100.0)
			onp.reparentTo(self.render)
			self.virtualanchorNodePath.append(onp)

		# load virtual robot odometry
		self.virtualodometryNodePath = self.loader.loadModel(panda_root_dir + track_media_dir + '/' + 'vplot.bam')
		self.virtualodometryNodePath.setScale(2.0, 2.0, 2.0)
		self.virtualodometryNodePath.setHpr(0.0, 90.0, 0.0)
		self.virtualodometryNodePath.setPos(100.0,100.0,100.0)
		self.virtualodometryNodePath.reparentTo(self.render)

		# load wp
		self.waypointNodePath = []
		for op in wp_position :
			onp = self.loader.loadModel(panda_root_dir + track_media_dir + '/' + 'waypoint.bam')
			onp.setScale(1.0, 1.0, 1.0)
			onp.setHpr(0.0, 90.0, op[2])
			onp.setPos(op[0],op[1],0.01)	
			onp.reparentTo(self.render)
			self.waypointNodePath.append(onp)


	def step(self):

		Simulator.step(self)
		# display virtual anchor
		for va in self.virtualanchorNodePath :
			va.setPos(1000.0,1000.0,1000.0)
		virtual_anchor_index = 0
		for i in range(calc_n_lm(self.robot_odometry.xEst)):
			px = self.robot_odometry.xEst[STATE_SIZE + i * 2]
			py = self.robot_odometry.xEst[STATE_SIZE + i * 2 + 1]
			#px = self.robot_odometry.odom_with_slam.x + a[1]*math.cos(math.radians(self.robot_odometry.odom_with_slam.h+a[0]))
			#py = self.robot_odometry.odom_with_slam.y + a[1]*math.sin(math.radians(self.robot_odometry.odom_with_slam.h+a[0]))
			self.virtualanchorNodePath[virtual_anchor_index].setPos(px,py,0.0)
			virtual_anchor_index += 1

		# display virtual odometry
		self.virtualodometryNodePath.setPos(self.robot_odometry.xEst[0],self.robot_odometry.xEst[1],0.0)






## MAIN ########################################################################

print("Init telemetry server...")
#tserver = telemetry_server("192.168.1.34", 7001)
tserver = telemetry_server("192.168.43.5", 7001)
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
app = SimulatorPR(start_position, rc, odom)
print("Done!")

# log
data_logger = open("log.txt","w")

# game loop
counter = 0
record_counter = 0
print("Start sim engine loop...")
while not app.quit:
	app.step()
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

		# #msg += str( float(app.robot_controller.target_speed_ms) ) + ';'
		# msg += str( float(app.robot_controller.current_speed_ms) ) + ';'
		# msg += str( float(app.robot_controller.actual_speed_ms) ) + ';' 
		# msg += str( float(app.robot_controller.actual_speed_error_ms) ) + ';'
		# msg += str( float(app.throttle) ) + ';' 

		# msg += str( float(app.robot_controller.actual_lidar_direction_error) ) + ';'
		# msg += str( float(app.robot_controller.pid_wall) ) + ';'
		# msg += str( float(app.robot_controller.ratio_ai*10) ) + ';' 
		# msg += str( float(app.steering) ) 

		for lm in app.robot_odometry.landmarks:
			msg += str( float(lm[0]) ) + ';'
			msg += str( float(lm[1]) ) + ';'
##		for i in range(calc_n_lm(app.robot_odometry.xEst)):
##			x = app.robot_odometry.xEst[STATE_SIZE + i * 2]
##			y = app.robot_odometry.xEst[STATE_SIZE + i * 2 + 1]
##			msg += str( float(x) ) + ';'
##			msg += str( float(y) ) + ';'
		msg += "0"

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
			str(round(odom.odom_with_slam.h,2)) + ";" +

			str(round(odom.error_x,3)) + ";" +
			str(round(odom.error_y,3)) + ";" +
			str(round(odom.error_h,3)) + ";" +


			str(round(odom.xEst.item(0),3)) + ";" +
			str(round(odom.xEst.item(1),3)) + ";" +
			str(round(odom.xEst.item(2),3)) + "\n"			
		)
	data_logger.flush()

dataset_file.close()
print('m:' + str(record_counter))

    
  


