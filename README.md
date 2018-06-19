# CarND-P8-Particle-Filter
Udacity Self-Driving Car Nanodegree - Kidnapped Vehicle Project Implementation

## Overview
This project consists of implementing an [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) with C++. A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Particle Filter[PF] use those measurements to predict the position of the object. The communication between the simulator and the PF is done using the [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the PF side. To get this project started, Udacity provides a seed project that could be found [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

Basically, what particle filter does is applying the Bayesian filter algorithm to hundreds or thousands of particles, where each particle represents a possible state for the system. We extract the estimated state from handreds or thousands of particles using weighted statistics of the particles. Using a finite number of randomly sampled points to compute a result is called Monte Carlo (MC) method. The idea is generating enough points to get a representative sample of the problem, run the points through the system we are modeling and then compute the results on the transformed points.

## Prerequisites
The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

This particular implementation is done on Linux OS. In order to install the necessary libraries, use the install-ubuntu.sh.

## Compiling and executing the project
These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `make`: This will create an executable `particle_filter`

## Running the Filter
From the build directory, execute `./particle_filter`. The output should be:
```
Listening to port 4567
Connected!!!
```

## Particle Filter Implementation
The following is the flowchart from Udacity lesson
![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/reference/PFImplementation.png)

As shown above, particle filter has four implementation steps:

1. Initialization Step

Initialize all particles to the first position based on estimates of x, y, theta and their uncertainties from GPS input. Set the number of particles and set all weights to 1.0. Also add random Gaussian noise to each particle.

2. Prediction Step

Predict the state of the next time step using the process model, input the control parameters, i.e. velocity[m/s] and yaw rate[rad/s] of the vehicle from time t to time t+1, the standard deviation of x [m], y [m] and yaw [rad], and the time [s] between step t and t+1 in measurements.

Here is the equation used for prediction step

![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/reference/Prediction.PNG)

3. Update Step

During this step, the particle weights are updated based on the likelihood of the observed map landmark positions and feature measurements. This step includes four sub-steps -- 

a) Find landmark predictions that are within sensor range, then create a vector to store the filtered landmark predictions.

b) Use homogenous transformation which performs rotation and translation as the following equation shown, to transfer observations from the vehicle coordinates to map coordinates.

![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/reference/HomogenousTransformation.PNG)

c) Data association: find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark. One of the techniques is using the nearest neighbour algorithm which is simple to implement.

d) Calculate particle weights: using Multivariate Gaussian Probability for each observation weight. The particle's weight is the product of total observation weights.

![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/reference/ParticleWeight.PNG)

4. Resampling Step

Resample particles with replacement with probability proportional to their weight.

Here is the algorithm from the lecture (presented in python)

![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/reference/Resampling.PNG)


## Output

The following is the screenshot of the simulator after running my implementation

![](https://github.com/JuAnne/CarND-P8-Particle-Filter/blob/master/Simulator.PNG)
