# Model Predictive Control

Model predictive control is process control mechanism that enables controling a process (such as managing vehicle trajectory) by 
satisfying certain constraints.

[//]: # (Image References) 
[image1]: ./images/KinematicModel.png
[image2]: ./images/PolynomialFit.png


---

### Model Predictive Control Algorithm

Model Predictive Control (MPC) algorithm sets up the task of finding the ideal trajectory around a track as an optimization problem.
Solving optimization problem results in finding the control inputs that result in an ideal trajectory. MPC algorithm overcome
some of the limitations of PID controller

Below are the steps involved in Model Predictive Control Algorithm:

1. We are provided a set of way points that represents an ideal path. In a real world scenario we can compute this using lane markings or through
   semantic segmenation to find drivable path.

2. These waypoints are translated to Vehicle coordinate systems by roation transformation and a polynomial is fit to these way points.

![alt text][image2]

3. One of the limitations of the PID controller is that it cannot handle delays in actuator control inputs. MPC overcomes such a problem
   by anticipating vehicle position after elapsed time delay and finds the ideal acuator control inputs (steering angle and throttle).
   For instance if there is 100 ms delay for the actuator inputs to take effect, vehicle's state after 100 ms can be predicted and can 
   be used to solve for optimal control inputs. Typically Kinametic model or dynamic model can be used for prediction. Kinematic model is
   much more simpler to compute and can be executed in real time. Below diagram shows Kinematic model equations:

![alt text][image1]

4. Once the state after time delay is predicted, predicted state is passed on to a solver that solves optimization problem.
   Below are the components of the optimization problem:


   a) Cost function / Objective function components are:
      
      - Cross-track error cost
      - Steering angle error cost
      - Minimimum velocity cost
      - Minimize usage of steering/throttle
      - Error between two consequetive steps


    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += weight_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weight_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weight_throttle * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weight_delta_seq * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weight_throttle_seq * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


   b) Further weights have been given to above costs so that vehicle is controlled in the center of the track. Weights have been added to
      actuator costs so that sharp steering or throttle is avoided.

   c) After setting up the cost function, model constraints are set up.

   d) After setting up model constraints, upper and lower bounds for non-actuators, bounds for steering angle is set to (-25,25"). Throttle
      is set in the range [-1,1]. 

   e) After the solver solves the optimal solution that consists of next state and the ideal actuator inputs, a vector consisting of (actuator inputs, next state)
      are returned. 

5. Using the estimated steering angle, throttle vehicle is moved along.


---

### Managing Latency

Limitation of PID controller is that it cannot handle latency in actuator inputs. In case MPC, such a situation can be modelled.
In this project, I have used Kinematic model to predict the future state based on current state, and used predicted new state to
solve for actuator inputs. 


---

### Tuning Time steps and time duration

Time step (dt) of 0.1 seconds or 100 milliseconds is used since we know that there is a 100 ms delay in actuaotor input.
10 time steps are used. I have experimented with multiple values such as dt = {0.01, 0.05, 0.1} and time duration = {25, 20, 15, 10}
Time duration of 0.1 seconds and time steps of 10 provided the best result.


---

### Result Videos

Below is the link to the output youtube video:

https://youtu.be/m5VvxddT808

