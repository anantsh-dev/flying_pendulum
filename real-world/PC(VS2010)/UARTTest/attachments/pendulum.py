import numpy as np
import vrep # access all the VREP elements
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Bernoulli
from torch.autograd import Variable
from statistics import mean
import math

#Hyperparameters
learning_rate = 0.01
gamma = 0.99

def getAngle(pos_b_x,pos_b_z):
	# print(pos_b_x,pos_b_z)
	angle=0.0
	if pos_b_z!=0.0:
		temp=round(pos_b_x/abs(pos_b_z),3)
		angle=math.degrees(math.atan(temp))
	return angle

class PolicyNet(nn.Module):
    def __init__(self):
        super(PolicyNet, self).__init__()

        self.fc1 = nn.Linear(3, 200)
        self.dropout = nn.Dropout(p=0.6)
        self.fc3 = nn.Linear(200, 1)  # Prob of Left

    def forward(self, x):
        x = self.fc1(x)
        x = self.dropout(x)
        x = F.relu(x)
        x = F.sigmoid(self.fc3(x))
        return x


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

pre_angle=0.0
num_episode = 52
batch_size = 3
learning_rate = 0.01
gamma = 1
load_inp = input("Load the model, type 1 -->")
policy_net = PolicyNet()
try:       
	if int(load_inp) == 1:
		print('Previous Model Loaded')
		policy_net = torch.load('model.p')
except:
	print('Error loading model')

optimizer = torch.optim.RMSprop(policy_net.parameters(), lr=learning_rate)
# Batch History
state_pool = []
action_pool = []
reward_pool = []
steps = 0
best_reward=0
steps_grad = 0

try:
	for e in range(num_episode):
		print('Episode : ', e)
		time.sleep(1)
		vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
		err_code, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_blocking)
		err_code, bob = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)
		t = time.time()
		err_code, pos = vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
		err_code, vel = vrep.simxGetFloatSignal(clientID, 'vel2', vrep.simx_opmode_streaming)
		err_code, pos_b_x = vrep.simxGetFloatSignal(clientID,'pos_b_x',vrep.simx_opmode_streaming)
		err_code, pos_b_z = vrep.simxGetFloatSignal(clientID,'pos_b_z',vrep.simx_opmode_streaming)
		angle = getAngle(pos_b_x,pos_b_z)
		state = [[pos[0], vel, angle]]
		state = torch.FloatTensor(state)
		state = Variable(state)
		rev =0
		steps=0
		while steps<1000:
			while pos[0]<0.001 and pos[0]>-0.001:
				err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
				err_code, vel = vrep.simxGetFloatSignal(clientID, 'vel2', vrep.simx_opmode_streaming)
				err_code, pos_b_x = vrep.simxGetFloatSignal(clientID,'pos_b_x',vrep.simx_opmode_streaming)
				err_code, pos_b_z = vrep.simxGetFloatSignal(clientID,'pos_b_z',vrep.simx_opmode_streaming)
				angle=getAngle(pos_b_x,pos_b_z)
				state = [[pos[0], vel, angle]]
				state = torch.FloatTensor(state)
				state = Variable(state)

			# print('state', state)
			probs = policy_net(state)
			# print('probs', probs)
			state_pool.append(state)
			m = Bernoulli(probs)
			# print('m', m)
			action = m.sample()
			action = action.data.numpy().astype(int)[0]
			time.sleep(0.02)
			rev = action
			vrep.simxSetIntegerSignal(clientID, 'rev', rev, vrep.simx_opmode_oneshot)
			err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
			err_code, vel = vrep.simxGetFloatSignal(clientID, 'vel2', vrep.simx_opmode_streaming)
			err_code, pos_b_x = vrep.simxGetFloatSignal(clientID,'pos_b_x',vrep.simx_opmode_streaming)
			err_code, pos_b_z = vrep.simxGetFloatSignal(clientID,'pos_b_z',vrep.simx_opmode_streaming)
			err_code, pos_x = vrep.simxGetFloatSignal(clientID,'pos_x',vrep.simx_opmode_streaming)
			err_code, pos_y = vrep.simxGetFloatSignal(clientID,'pos_y',vrep.simx_opmode_streaming)
			# err_code, pos_b_z = vrep.simxGetFloatSignal(clientID,'pos_b_z',vrep.simx_opmode_streaming)
			angle=getAngle(pos_b_x,pos_b_z)
			
			reward= abs(abs(angle)-abs(pre_angle))*0.2
			if (pre_angle>30 and angle>30) or (pre_angle<-30 and angle<-30):
				reward=-10

			pre_angle=angle
			next_state = [[pos[0], vel, angle]]
			next_state = torch.FloatTensor(next_state)
			# To mark boundarys between episodes
			action_pool.append(float(action))
			reward_pool.append(reward)

			state = next_state
			state = Variable(state)

			steps += 1
			steps_grad += 1
			if pos_x<-.75 or pos_x>.75:
				reward_pool[-2]=reward_pool[-2]-50
				break
			if pos_y<-.75 or pos_y>.75:
				reward_pool[-2]=reward_pool[-2]-50
				break
		
		reward_pool[-1] = 0
		print(e, sum(reward_pool), len(reward_pool))

		# Update policy
		if e > 0 and e % batch_size == 0:
			# Gradient Desent
			optimizer.zero_grad()

			for i in range(steps_grad):
			    state = state_pool[i]
			    action = Variable(torch.FloatTensor([action_pool[i]]))
			    reward = reward_pool[i]

			    probs = policy_net(state)
			    m = Bernoulli(probs)
			    loss = -m.log_prob(action) * reward  # Negtive score function x reward
			    loss.backward()

			optimizer.step()
			if sum(reward_pool)>best_reward:
				best_reward=sum(reward_pool)
				torch.save(policy_net,'model_best.p')

			state_pool = []
			action_pool = []
			reward_pool = []
			steps_grad = 0
		vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
		time.sleep(1)
	torch.save(policy_net,'model.p')
except Exception as e:
	print("Exception : ", e)
	print('Training Interrupted')
finally:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
	save_inp = input("do you want to save, type 1 -->")
	if int(save_inp) == 1:
		torch.save(policy_net,'model.p')	


