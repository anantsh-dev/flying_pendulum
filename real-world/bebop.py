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
from pyparrot.Bebop import Bebop

def getAngle():
    with open("PC(VS2010)/UARTTest/angles.txt", "r") as f:
        a=(f.readlines())
    for x in range(5):
        b=a[-(x+1)].split(' ')
        if len(b)==9:
            break
    if abs(float(b[1])) > abs(float(b[2])):
        return float(b[1])
    else:
        return float(b[2])


class PolicyNet(nn.Module):
    def __init__(self):
        super(PolicyNet, self).__init__()

        self.fc1 = nn.Linear(3, 100)
        self.dropout = nn.Dropout(p=0.5)
        self.fc3 = nn.Linear(100, 1)  # Prob of Left

    def forward(self, x):
        x = self.fc1(x)
        x = self.dropout(x)
        x = F.relu(x)
        x = F.sigmoid(self.fc3(x))
        return x

bebop = Bebop()
policy_net = PolicyNet()
load_inp = input("Load the model, type 1 -->")
try:       
	if int(load_inp) == 1:
		print('Previous Model Loaded')
		policy_net = torch.load('model_best.p')
except:
	print('Error loading model')

print("connecting")
success = bebop.connect(10)
print(success)
dx = []

pre_angle=0.0
num_episode = 22
batch_size = 3
learning_rate = 0.01
gamma = 1
optimizer = torch.optim.RMSprop(policy_net.parameters(), lr=learning_rate)
# Batch History
state_pool = []
action_pool = []
reward_pool = []
best_reward=0
steps_grad = 0
dx = [0]
try:
	pitch = 80
	if (success):
	    bebop.smart_sleep(2)
	    bebop.ask_for_state_update()
	    print('safe_takeoff')
	    bebop.safe_takeoff(10)
	    bebop.set_hull_protection(1)
	    bebop.set_max_distance(10)
	    bebop.set_max_altitude(5)	    
	    print('going up')
	    bebop.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=10, duration=3)
	    bebop.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=10, duration=3)
	    print('add pendulum')
	    bebop.smart_sleep(10)
		for e in range(num_episode):
			print('Episode : ', e)
			count =0
			angle = getAngle()
		    state = [[sum(dx), (pitch*0.01)*2.5, angle]]
			state = torch.FloatTensor(state)
			state = Variable(state)
		    while count<50:
				state_pool.append(state)
				m = Bernoulli(probs)
				# print('m', m)
				action = m.sample()
				action = action.data.numpy().astype(int)[0]

	            if action==0:
                	pitch=pitch
            	else:
                	pitch=-pitch

		    	bebop.fly_direct(roll=0, pitch=pitch, yaw=0, vertical_movement=0, duration=0.5)
	            angle = getAngle()
				reward= abs(abs(angle)-abs(pre_angle))*0.2
				if (pre_angle>30 and angle>30) or (pre_angle<-30 and angle<-30):
					reward=-10
				
				dx.append((pitch*0.01)*2.5*0.5)
				pre_angle=angle
            	next_state = [[sum(dx), (pitch*0.01)*2.5, angle]]
            	next_state = torch.FloatTensor(next_state)
				action_pool.append(float(action))
				reward_pool.append(reward)
            	state = Variable(next_state)
		    	count = count+1
		    	steps_grad += 1
				if dist<-.75 or dist>.75:
					reward_pool[-2]=reward_pool[-2]-50
					break

			reward_pool[-1] = 0
			print(e, sum(reward_pool), len(reward_pool))

			if e > 0 and e % batch_size == 0:
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

		torch.save(policy_net,'model.p')
		print('-'*81)
		print('remove'*80)
	    bebop.smart_sleep(30)
	    bebop.safe_land(10)
	    bebop.smart_sleep(2)
	    bebop.disconnect()
	except KeyboardInterrupt:
		bebop.smart_sleep(30)
