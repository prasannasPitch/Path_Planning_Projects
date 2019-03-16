# Path_Planning_Projects

Path planning and decision making for autonomous vehicles in urban environments enable self-driving cars to find the safest, most convenient, and most economically beneficial routes from point A to point B. Finding routes is complicated by all of the static and maneuverable obstacles that a vehicle must identify and bypass. Today, the major path planning approaches include the predictive control model, feasible model, and behavior-based model. Let’s first get familiar with some terms to understand how these approaches work.

![image](https://user-images.githubusercontent.com/37708330/53702229-f6a04280-3e04-11e9-95c6-26888f5b78ff.png)



- A path is a continuous sequence of configurations beginning and ending with boundary configurations. These configurations are also referred to as initial and terminating.
- Path planning involves finding a geometric path from an initial configuration to a given configuration so that each configuration and state on the path is feasible (if time is taken into account).
- A maneuver is a high-level characteristic of a vehicle’s motion, encompassing the position and speed of the vehicle on the road. Examples of maneuvers include going straight, changing lanes, turning, and overtaking.
- Maneuver planning aims at taking the best high-level decision for a vehicle while taking into account the path specified by path planning mechanisms.
- A trajectory is a sequence of states visited by the vehicle, parameterized by time and, most probably, velocity.
T- rajectory planning or trajectory generation is the real-time planning of a vehicle’s move from one feasible state to the next, satisfying the car’s kinematic limits based on its dynamics and as constrained by the navigation mode.


### Prediction
Prediction is the first step of path planning. It involves, predicting the behavior of other vehicles, and estimating their location at a time step in the future. Because of the time limitations, predictions only performed by increasing the s values of other cars, and the d value of each other car assumed to be constant in each step. The provided vx and vy values were combined in a single v, and this v value used to calculate the s value in the future.

### Trajectory Generation
Trajectory Generation is the second step and probably the most challenging step of path planning. During generation step, maximum acceleration and maximum velocity values were set in advance. The interactions with the other cars were calculated, such as closest approach, and lowest time to collide. In addition, other penalties added for not driving at the target speed, changing lanes, driving at side lanes, canceling the previous action, driving in an occupied lane.

![image](https://user-images.githubusercontent.com/37708330/53702249-29e2d180-3e05-11e9-82ea-c99735b90cbe.png)



### Finite State Machines

![image](https://user-images.githubusercontent.com/37708330/53702329-0bc9a100-3e06-11e9-80c7-f48eb9fa8072.png)


### Pseudocode for state transition

```
def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):
    # only consider states which can be reached from current FSM state.
    possible_successor_states = successor_states(current_fsm_state)

    # keep track of the total cost of each state.
    costs = []
    for state in possible_successor_states:
        # generate a rough idea of what trajectory we would
        # follow IF we chose this state.
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)

        # calculate the "cost" associated with that trajectory.
        cost_for_state = 0
        for i in range(len(cost_functions)) :
            # apply each cost function to the generated trajectory
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            # multiply the cost by the associated weight
            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
         costs.append({'state' : state, 'cost' : cost_for_state})

    # Find the minimum cost state.
    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state

```
### Cost Function Defined

![image](https://user-images.githubusercontent.com/37708330/53994408-8b22e180-4132-11e9-8a6a-267e1094d312.png)

### Cost Function Priorities for Specific Scenarios:

![image](https://user-images.githubusercontent.com/37708330/54076423-c563be80-42ab-11e9-98cd-1887956e1b35.png)


### Motion Planning Algorithms
![image](https://user-images.githubusercontent.com/37708330/54482564-cebdcf80-4845-11e9-93ff-8df0a8787813.png)

