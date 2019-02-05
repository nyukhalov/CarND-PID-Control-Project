# PID Controller Project: Writeup

The goal of the project was to build and tune a PID controller for controlling the car's steering angle.

This writeup describes the approach taken to successfully complete the project.

## Project structure

The `src/` directory contains the source code of two applications:

- `pid` - a program which uses PID controllers to control the steering angle and throttle of the vehicle in the simulator.
- `twiddle` - a program which continuously runs the simulation to identify the best hyperparameters for the steering PID controller.

The project's source code files are listed below:

- `PID.h` and `PID.cpp` - an implementation of a PID controller.
- `main.cpp` - the entry point of the `pid` program.
- `twiddle.cpp` - the entry point of the `twiddle` program.

## PID Controller

The implemented PID controller is represented by the `PID` class. A user can create an instance of the class by calling its constructor and passing in the `P-`, `I-` and `D-` hyperparameters.

The initialization of the PID controller may look like this:

```cpp
double K_p = 0.1;    // P-gain
double K_i = 0.0001; // I-gain
double K_d = 0.3;    // D-gain

PID controller(K_p, K_i, K_d);
```

The class has two methods: `void UpdateError(double cte)` for updating the controller's state, and `double TotalError()` for getting the total error.

The methods are used as shown below:

```cpp
steer_pid.UpdateError(cte);
double steer_value = -steer_pid.TotalError();
```

## Hyperparameters

There are three hyperparameters in the PID controller (`P`, `I` and `D`) assigned to the proportional, integral and derivative errors respectively. The hyperparameters define how well the PID controller works. In this section I describe the effects of each parameter.

### `P` component

The `P` hyperparameter controls how much the current cross track error's (CTE) value impacts the total error. When the `P` value is big the car trajectory has many oscillations which oftern results in inability to follow the desired trajectory. In constrast, low `P` values make the car not responsive and unable to turn toward the center of a lane in order to reduce the CTE.

The `P` value can be considered well choosen when the car is stable (able to follow its trajectory). 

### `I` component

The `I` hyperparameter controls how much the sum of CTEs impacts the total error on each iteration. Well choosen value helps to tolarate continuous errors cased by the inaccuracy of hardware of dynamically changing environment such as a trajectory turns.

### `D` component

The `D` hyperparameter controls how much the recent change of CTE impacts the total error. It is used to avoid oscillations cased by the `P` and `I` components.

## Tuning the steering controller

The most difficult part of the project was to find the right hyperparameters for the steering controller.

The hyperparameters can be chosen manually or automatically, both methods have pros and cons. For this project, I intentified the parameters manually, though I implemented the twiddle algorithm which in fact didn't work well.

The final hyperparameters' values which smoothly control the car at 20-40 MPH are listed below:

- `K_p = 0.12;`
- `K_i = 0.0005;`
- `K_d = 3.8;`

A higher speed (up to 60 MPH) is drivable but not that smooth and safe. After 60 MPH the given controller is not able to control the car.

### Steering smoothing

Using the PID's total error as the value of the steering angle resulted in jerky driving. This could make the car not stable and it is simply not comfortable for people to experience such a jerk while driving.

In order to solve the issue, I used the low-pass filter to filter spikes of steering angle values.

The filter's coefficients were chosen manually after several experiments. The values are:

- Current value gain: `0.3`
- Previous value gain: `1-0.3=0.7`

After implementing the filter the car started driving much smoother but lost a bit of responsiveness.

### Choosing PID hyperparameters

The algorithm I followed to manually choose the hyperparameters:

- Set all values to 0
- Increase `K_p` until the car is stable and able to drive left turns
- Increase `K_d` until oscillations disappear
- Increase `K_i` to make the car more responsive while turning

### Twiddle

The manual approach described above takes a lot of human time. In order to speed up this process, I implemented an automated solution based on the twiddle algorithm given in the classroom.

The `twiddle` program is a client for the simulator which uses twiddle to choose the best hyperparameters.

Each run (stage) in the twiddle is `num_iter` long.
I set `num_iter` to `4000` which is enough to pass the first three turns of the track at speed of 30MPH.

Once a stage is completed the program sends a restart message (`42["reset",{}]"`) to the simulator in order to restart car's position and speed.

The output of the `twiddle` program shown below shows the current stage's number, best error and values of the `p` and `d` parameters.

```
stage_no=605  , d_sum=0.000168018, best_err=0.0229817, MSE=0.0443565, p={0.669082, 0.00207447, 16.5094}, d={9.54586e-06, 2.90464e-08, 0.000158443}
stage_no=606  , d_sum=0.000168018, best_err=0.0229817, MSE=0.0475602, p={0.669082, 0.00207442, 16.5094}, d={9.54586e-06, 2.90464e-08, 0.000158443}
stage_no=607  , d_sum=0.000168015, best_err=0.0229817, MSE=0.0481742, p={0.669082, 0.00207445, 16.5095}, d={9.54586e-06, 2.61417e-08, 0.000158443}
stage_no=608  , d_sum=0.000168015, best_err=0.0229817, MSE=0.0460843, p={0.669082, 0.00207445, 16.5092}, d={9.54586e-06, 2.61417e-08, 0.000158443}
stage_no=609  , d_sum=0.000152171, best_err=0.0229817, MSE=0.0452825, p={0.669091, 0.00207445, 16.5094}, d={9.54586e-06, 2.61417e-08, 0.000142599}
```

The twiddle algorithm chooses the hyperparameters' values in order to minimize the MSE error. Its results, however, were not good. The PID controller based on the parameters chosen by the algorithm was very jerky. Most likely people won't like to be driving by such a controller due to the lack of smoothness.

## Speed controller

I used the PID controller described above to control car's speed. The gain parameters were choosen manually after several experiments:

- `K_p = 0.9`
- `K_i = 0.0001`
- `K_d = 0`

## Conclusion

In this project, I implemented and tuned the PID controller which successfully drove the car throughout the virtual track in the simulator at 30 MPH.

The low-pass filter was used to tolerate spikes of steering values which made the car driving much smoother.

The `twiddle` algorithm along with the ability of the simulator to programmatically restart the car's position allowed me to implement an automated solution for choosing the controller's hyperparameters.

The final hyperparameters' values were chosen manually. The knowledge about the parameters' nature helped me to find the right values faster.

## Further improvements

In order to drive at higher speed, one must find different hyperparameters' values. Probably, it is possible to find a function `K_p, K_i, K_d = f(speed)` which for the given speed will return the hyperparameters' values. Such adaptive parameters could give better smoothing and fewer errors.

The twiddle algorithm can be improved by taking into account steering smoothness while calculating the error.