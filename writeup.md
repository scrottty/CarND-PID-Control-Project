# PID Controller Project


## PID Control
The PID controller is made up of three elements which, when combined, output a control effort to minimise the error in a system. The combination of the elements, controlled via coefficients, defines the properties of the system and how it reacts to various inputs.

### Controller Tuning
The PID controller was initially tuned manually to get the controller to produce the approximate desired properties for the cars control. As the PID controller can produce different styles of control dependent upon the problem it was thought that it would be best to get a broad controller design down and then use the twiddle algorithm to fine tune the design.

From the initial tuning it was found that this was a disturbance problem and the final controller would need to be designed to handle a large number of disturbances. PID control in its basic form is for driving a signal to a set point and holding it there. The car can be thought of as the signal and the middle of the road as the set point. Once there there is no controller effort required to hold the car on the set point.

The corners can be seen as a disturbance to the system which require a control effect to negate. As the car goes around the corner it is being *'pulled'* from its set point. The controller will then start to build a control effort to counter this and return the car to the middle of the road / its set point.

The inherent fault in this control strategy is that it requires an error to produce a reaction from the system. By using the distance to the middle of the road as the error metric the controller is lead to a repetition of the car drifting from the centre line and then being returned. This is an erratic and probably uncomfortable behaviour for a passenger. A potentially better strategy would be to try and match the steering angle and the curvature of the road. Then the controller would hopefully be smoother and the car be able to follow the center of the road without needing to wait for error build up.

After the manual tuning the twiddle algorithm was implemented via a *'TWIDDLE'* class [(link)](/src/TWIDDLE.cpp). This ran the algorithm and fined tuned the controller finding the coefficients that produced the smallest cumulative error.

The final PID controller was:

``` c++

effort = -0.0545267 x error - 0.001 x i_error - 0.596748 x d_error

```
The final reulst can be seen in this [video](Final_Tuning.mov)

## Controller Elements
### Propotional Element
The proportional element of the PID controller is used to provide a base steering angle to minimise the error of the car and the center of the road. The further from the center of the road the larger the steering angle. It acts as the fine control to bring the car to center of the road. Without it the car doesn't get any effort to move towards the centre until the integral build up enough to do so.

The [video](No_Proportional.mov) shows the car without the proportional element of the control. It can be seen that the car sits on the side of the road and will only turn once the corner is sharp enough for the derivative to produce a controller effort.

### Integral Element
The integral element of the PID controller is the smallest element and in the final tuning has minimal effect on the final output. As there is no continuous force stopping the car from reaching the middle of the road the integral is needed little. It would however be useful if the car had a permanent tendency to shift to one side of the road and if the car required effort to stay in the middle of the road once there.

As integral is a continuous build up of error it can have the effect of delaying the controller when error first occurs as it has to be *'shed'* before to controller can act. eg. if the integral has built up due to the car sitting on the right side of the road and a corner required the car to turn right, all of the left turning that has been built up would have to be overcome by the other controller efforts for the car to turn right.

This had the effect of making the car take the corners wider than without and integral at all as can be seen in this [video](No_Integral.mov) where the integral is zero. However the plus side of the integral was that it could help the proportional move the car back towards the centre if the proportional was not doing enough.

### Differential Element
The differential element of the PID controller is the main effort of the controller and is used to turn the corners. As mentioned above, the system is a disturbance system therefore a large amount of differential is required.

The differential can clearly be seen in the cars behaviour whilst driving. As the car moves further from the center of the road the eventual difference causes a big adjustment in the steering. The car then turns sharply towards the middle.

The other effect of the differential is to help limit large oscillations in the control due to the proportional control. If the car starts to oscillate quickly the resulting differences cause a big derivate effort that limits the amount the car turns. This stops the control from becoming unstable.

The [video](No_Derivative) shows that controller without the derivate element of the control. It can be seen that the proportional element of the controller starts becoming unstable (bigger oscillations) and that the car struggles to handle the sharpening corner. The car ends up off the road. The derivate element could be thought of as the corner turner part of the control
