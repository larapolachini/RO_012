# TP EKF

## Question 1

For this question we'll modify the number and position of landmarks and the robot trajectory for:

#### A short loop and a dense map with many landmarks inside the robot perception radius

<p align="center">
  <img src="Q1-a.png" alt="" title="" width="500">
  <br>
  <em>Figure 1 – A short loop and a dense map with many landmarks inside the robot perception radius </em>
</p>

In this case it's possible to see that the map quality seems very good, since the robot frequently re-sees many nearby landmarks, the filter gets lots of informative updates.
The error evolution stays very small and inside the covariance bounds. When the robot returns near the start, it's possible to see a small, smooth correction in the estimated trajectory toward the true.

#### A long loop and a dense map with many landmarks all along the loop

<p align="center">
  <img src="Q1-b.png" alt="" title="" width="500">
  <br>
  <em>Figure 2 – A long loop and a dense map with many landmarks all along the loop </em>
</p>

For the long loop with landmarks distributed along the path, the EKF-SLAM maintains reasonable accuracy throughout the trajectory. Pose uncertainty increases gradually while moving away from known landmarks and drops sharply upon loop closure, demonstrating effective map correction and consistent filter behavior.

#### A long loop and a sparse map with only few landmarks near the start position

<p align="center">
  <img src="Q1-c.png" alt="" title="" width="500">
  <br>
  <em>Figure 3 – A long loop and a sparse map with only few landmarks near the start position </em>
</p>

In the long-loop sparse-map experiment, the EKF-SLAM shows significant drift while landmarks are not visible, leading to large uncertainty growth. When the robot re-enters the region containing the few known landmarks, a strong loop-closure correction occurs, sharply reducing both the error and covariance. This demonstrates how insufficient landmark coverage degrades map accuracy and increases reliance on loop closure for correction.

## Question 2

## Question 3 

## Question 4