# Path_Planning_Projects

![image](https://user-images.githubusercontent.com/37708330/53702229-f6a04280-3e04-11e9-95c6-26888f5b78ff.png)

### Prediction
Prediction is the first step of path planning. It involves, predicting the behavior of other vehicles, and estimating their location at a time step in the future. Because of the time limitations, predictions only performed by increasing the s values of other cars, and the d value of each other car assumed to be constant in each step. The provided vx and vy values were combined in a single v, and this v value used to calculate the s value in the future.

### Trajectory Generation
Trajectory Generation is the second step and probably the most challenging step of path planning. During generation step, maximum acceleration and maximum velocity values were set in advance. The interactions with the other cars were calculated, such as closest approach, and lowest time to collide. In addition, other penalties added for not driving at the target speed, changing lanes, driving at side lanes, canceling the previous action, driving in an occupied lane.

![image](https://user-images.githubusercontent.com/37708330/53702249-29e2d180-3e05-11e9-82ea-c99735b90cbe.png)