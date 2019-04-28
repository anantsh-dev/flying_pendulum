# Flying Pendulum
This is code to train a deep reinforcement policy to control the movement of the drone in a manner such that the attached payload achieves a pendulum like motion.

## Running the Code
The simulation code run on top of V-REP and the real-world training has been done using a parrot bebop2 drone. Follow the instructions in the respective sections to train a model.

### Simulation
* Install V-REP
* Import the pendulum.ttt scene in V-REP.
* Check the port on which the V-REP Server is running and change accordingly in the pendulum.py file.
* V-REP import files are already present in the folder, you can replace these files from your V-REP installation if it causes some error.
* Install python3 libraries required to run pendulum.py and then use this command to start the training:
```
python3 pwndulum.py
```

### Real-World
* PC(VS2010) folder contains the code to recieve readings from WitMotion BWT901CL IMU sensor.
* Import the PC(VS2010) folder as a project in Visual Studio.
* Run the project and it starts writing the readings to a text file - angles.txt
* Install python3 libraries required to run bebop.py and use use this command to start the training:
```
python3 bebop.py
```

