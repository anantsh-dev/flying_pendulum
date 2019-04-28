import numpy as np
import vrep # access all the VREP elements
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Bernoulli
from torch.autograd import Variable
from statistics import mean

#Hyperparameters
learning_rate = 0.01
gamma = 0.99


class PolicyNet(nn.Module):
    def __init__(self):
        super(PolicyNet, self).__init__()

        self.fc1 = nn.Linear(3, 24)
        self.fc3 = nn.Linear(24, 1)  # Prob of Left

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.sigmoid(self.fc3(x))
        return x


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")


num_episode = 500
batch_size = 5
learning_rate = 0.01
gamma = 0.99
policy_net = PolicyNet()        
optimizer = torch.optim.RMSprop(policy_net.parameters(), lr=learning_rate)


# Batch History
state_pool = []
action_pool = []
reward_pool = []
steps = 0



for e in range(num_episode):
    print('Episode : ', e)
    time.sleep(1)
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
    err_code, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_blocking)
    err_code, bob = vrep.simxGetObjectHandle(clientID, "Sphere", vrep.simx_opmode_blocking)
    t = time.time()
    err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
    err_code, vel = vrep.simxGetFloatSignal(clientID, 'velocity', vrep.simx_opmode_streaming)
    err_code, gamma = vrep.simxGetFloatSignal(clientID, 'gamma', vrep.simx_opmode_streaming)
    state = [[pos[0], vel, gamma]]
    state = torch.FloatTensor(state)
    state = Variable(state)
    rev =0 
    steps=0
    all_pos = [pos[0]]
    while steps<1000:
        while pos[0]<0.001 and pos[0]>-0.001:
            err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
            err_code, vel = vrep.simxGetFloatSignal(clientID, 'velocity', vrep.simx_opmode_streaming)
            err_code, gamma = vrep.simxGetFloatSignal(clientID, 'gamma', vrep.simx_opmode_streaming)
            state = [[pos[0], vel, gamma]]
            state = torch.FloatTensor(state)
            state = Variable(state)

        print('state', state)
        probs = policy_net(state)
        print('probs', probs)
        state_pool.append(state)
        m = Bernoulli(probs)
        print('m', m)
        action = m.sample()
        action = action.data.numpy().astype(int)[0]
        time.sleep(0.02)
        rev = action
        vrep.simxSetIntegerSignal(clientID, 'rev', rev, vrep.simx_opmode_oneshot)
        err_code, pos= vrep.simxGetObjectPosition(clientID, target, -1,vrep.simx_opmode_streaming)
        err_code, vel = vrep.simxGetFloatSignal(clientID, 'velocity', vrep.simx_opmode_streaming)
        err_code, gamma = vrep.simxGetFloatSignal(clientID, 'gamma', vrep.simx_opmode_streaming)
        all_pos.append(pos[0])
        if(gamma>50 or gamma<-50):
            reward = -1
        elif (gamma>20 or gamma<-20):
            reward = 2
        else:
            reward = 0

        next_state = [[pos[0], vel, gamma]]
        next_state = torch.FloatTensor(next_state)
        # To mark boundarys between episodes
        action_pool.append(float(action))
        reward_pool.append(reward)

        state = next_state
        state = Variable(state)

        steps += 1
    mean_pos  = mean(all_pos)
    if mean_pos<1 and  mean_pos> -1:
        reward_pool[-2] = 400
        reward_pool[-1] = 0
    else:
        reward_pool[-2] = -((abs(mean_pos)-1)*400)
        reward_pool[-1] = 0

    # Update policy
    if e > 0 and e % batch_size == 0:

        # Discount reward
        running_add = 0
        for i in reversed(range(steps)):
            if reward_pool[i] == 0:
                running_add = 0
            else:
                running_add = running_add * gamma + reward_pool[i]
                reward_pool[i] = running_add

        # Normalize reward
        reward_mean = np.mean(reward_pool)
        reward_std = np.std(reward_pool)
        for i in range(steps):
            reward_pool[i] = (reward_pool[i] - reward_mean) / reward_std

        # Gradient Desent
        optimizer.zero_grad()

        for i in range(steps):
            state = state_pool[i]
            action = Variable(torch.FloatTensor([action_pool[i]]))
            reward = reward_pool[i]

            probs = policy_net(state)
            m = Bernoulli(probs)
            loss = -m.log_prob(action) * reward  # Negtive score function x reward
            loss.backward()

        optimizer.step()

        state_pool = []
        action_pool = []
        reward_pool = []
        steps = 0
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
    time.sleep(1)


















# rev =0 
# c=0

# while c<1000:
#   time.sleep(0.02)
#   c=c+1
#   err_code, vel = vrep.simxGetFloatSignal(clientID, 'velocity', vrep.simx_opmode_streaming)
#   err_code, gamma = vrep.simxGetFloatSignal(clientID, 'gamma', vrep.simx_opmode_streaming)
#   # if (time.time() -t)%1 <0.05:
#   # a=round(time.time()-t,2)
#   if c%100 ==0:
#       a=round(time.time()-t,3)
#       print (a)
#       t=time.time()
#       print('signal sent')
#       if rev ==0:
#           rev =1
#       else:
#           rev=0
#   vrep.simxSetIntegerSignal(clientID, 'rev', rev, vrep.simx_opmode_oneshot)
#   print(c, vel, rev, gamma)