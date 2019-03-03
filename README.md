# Path_Planning_Projects

![image](https://user-images.githubusercontent.com/37708330/53702229-f6a04280-3e04-11e9-95c6-26888f5b78ff.png)

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
