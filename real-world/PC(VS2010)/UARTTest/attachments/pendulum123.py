import numpy as np
import vrep # access all the VREP elements
import time
import math

#Hyperparameters
learning_rate = 0.01
gamma = 0.99


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start aconnection
if clientID!=-1:
	print ("Connected to remote API server")
else:
	print("Not connected to remote API server")
	sys.exit("Could not connect")


vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
err_code, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_blocking)
err_code, bob = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)
t = time.time()
err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
rev =0 
c=0
while c<1000:
	time.sleep(0.02)
	c=c+1
	err_code, vel = vrep.simxGetFloatSignal(clientID, 'velocity', vrep.simx_opmode_streaming)
	err_code, vel2 = vrep.simxGetFloatSignal(clientID, 'vel2', vrep.simx_opmode_streaming)
	err_code, gamma = vrep.simxGetFloatSignal(clientID, 'gamma', vrep.simx_opmode_streaming)
	vrep.simxSetIntegerSignal(clientID, 'rev', rev, vrep.simx_opmode_oneshot)

	err_code, pos_x = vrep.simxGetFloatSignal(clientID,'pos_x',vrep.simx_opmode_streaming)
	err_code, pos_y = vrep.simxGetFloatSignal(clientID,'pos_y',vrep.simx_opmode_streaming)
	# err_code, pos_z = vrep.simxGetFloatSignal(clientID,'pos_z',vrep.simx_opmode_streaming)
	err_code, pos_b_x = vrep.simxGetFloatSignal(clientID,'pos_b_x',vrep.simx_opmode_streaming)
	err_code, pos_b_y = vrep.simxGetFloatSignal(clientID,'pos_b_y',vrep.simx_opmode_streaming)
	# err_code, pos_b_z = vrep.simxGetFloatSignal(clientID,'pos_b_z',vrep.simx_opmode_streaming)
	# if pos_b_x!=0:
	# 	temp=round(pos_b_y/pos_b_x,3)
	# 	angle=math.degrees(math.atan(temp))
	# else:
	# 	angle=90
	# if angle<0:
	# 	angle=-(90+angle)
	# else:
	# 	angle=90-angle

	# if c%15 ==0:
	# 	a=round(time.time()-t,3)
	# 	print (a)
	# 	t=time.time()
	# 	print('signal sent')
		# if rev ==0:
		# 	rev =1
		# else:
		# 	rev=0 
	print(c,vel2,rev)


	
print('time tp stop')
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
time.sleep(1)
print('done')