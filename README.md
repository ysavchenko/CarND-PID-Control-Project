# PID Controller Project
Self-Driving Car Engineer Nanodegree Program

---

In this project I'm usine PID controller to steer vehicle on the track in simulator.

## Dependencies & Build Instructions

You can use the [parent repository](https://github.com/udacity/CarND-PID-Control-Project) decription if you need help building this project.

## PID Controller explained

PID is an algorithm, which takes the error value (which is a difference between desired state of the system and current one) and applies a correction to the system based on three terms: P, I and D which stand for 'propotional', 'integral' and 'derivative'.

In the context of our project desired state is the position of the vehicle relative to the center of the road and correction is its steering angle value.

Each term plays its own role in PID controller. Here is a short description for each of them:

- Propotional (or P) applies correction proportionally to the current error value. Its goal is to get the system back to the desired state
- Integral (or I) uses the sum of all errors in the past states of the system and applies correction proportional to this sum. It allows to compensate for the error built in within the system (for example, we set our steering to a particular value, but the actual steering differs because of the error in the system).
- Differential (or D) takes the difference beteween current and past error and calculates correction based on this dirrerence. This term allows 'smoothing' the effect of proportional control, if the error already decreeses it slows down the correction preventing 'overshoots' (when system state oscillates near the desired state never actually staying in it).

## PID Controller implementation

You can find PID controller implementation in `PID` class. Here is a part where the correction is calculated:

```cpp
double PID::GetCorrection() {
    return -(Kd * d_error + Kp * p_error + Ki * i_error);
}
```

Where `Kd`, `Kp` and `Ki` are differential, proportional and integral coefficients. And `d_error`, `p_error` and `i_error` are errors: differential (difference between two errors), proportional (current error) and integral (accumulated error).

## Parameter tuning

I've started with P coefficient set to 1 and all other set to 0 (just to establish the scale of coefficients). As expected, vehicle tried to stay on the track, but absense of differential correction made it oscillate around the center of the track eventually falling off of it on the first turn.

[![](http://img.youtube.com/vi/ZsqQFtoDBCM/0.jpg)](http://www.youtube.com/watch?v=ZsqQFtoDBCM)

Then I've tried tuning the differential coefficient (last parameter). Value of `1` did not do much difference (vehicle still fell off the track on the first turn), so I've decided to go with `100`. This made vehicle stay almost always on the center of the trac, but it was steering left and right needlesly even when it was on a straight path (as you can see on the video below).

[![](http://img.youtube.com/vi/ErXaZp84cUY/0.jpg)](http://www.youtube.com/watch?v=ErXaZp84cUY)

On the next step I've decreased P and D terms proportionally (to `0.2` and `20` respectively). This made things almost the same, but vehicle tended to tolerate going out off the center so I've increased P coefficient a little to `0.3`.

And finally there was a time to tune integral coefficient. During testing I did not notice any system error, so I had to add one myself. I've chosen to add `0.2` to each steering value, here is the part of code where it is done:

```cpp
msgJson["steering_angle"] = steer_value + .2;
```

As expected vehicle started moving closer to the right side of the track so I had to add an integral term to compensate for the error. Since it is an integral parameter and it uses the sum of all the errors I've chosen it to be several orders of magnitude smaller than the rest. The first value tried (`0.001`) did a good jod of bringing vehicle back to the center of the road, so I've kept it.

So the final values for coefficients vere:

K<sub>P</sub> = 0.3

K<sub>I</sub> = 0.001

K<sub>D</sub> = 20

Below you can see the video of vehicle going through the whole track using PID controller with these coefficients.

[![](http://img.youtube.com/vi/1aMZHZmcBXA/0.jpg)](http://www.youtube.com/watch?v=1aMZHZmcBXA)

## Adding PID Controller for speed

I've also tried to add a second controller to control vehicle speed. In the initial project code throttle was set to be the constant (`0.3`) and top speed of the vehicle was aroung 30 mph. So the goal was to make it drive faster on the straighe lines, but lower the speed on turns. Here is a formula I came up with:

```cpp
double target_speed = max_speed - ((max_speed - min_speed) * abs(steer_value));
```

Since steering value is between -1 and 1 we get target speed to equal maximum on zero steering and minimum on maximum steering.

With the PID coefficients, the one I've tried initially (the same as location PID controller) were accelerating vehicle too fast, I reduced P and D values `0.1` and `1` respectively.

You can see the video of the final project using two PID controllers (one for steering and one for speed) below:

[![](http://img.youtube.com/vi/OUrdy6t2klo/0.jpg)](http://www.youtube.com/watch?v=OUrdy6t2klo)
