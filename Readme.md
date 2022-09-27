# Flight Control

This is a package to control UAV by communicating with flight controller by ROS.

[MAVROS](http://wiki.ros.org/mavros) should be installed for PX4!

## Note

* motive - Control UAV under motion capture system
    * Input(from mocap): /vrpn_client_node/RigidBody*/pose
    * Output(to PX4): /mavros/setpoint_velocity/cmd_vel
    * Output(vir): /desired_position /desired_velocity
    * Features: Keyboard control and circular trajectory are also included

* vio - Control UAV under SLAM
    * Input(from VIO): /vins_estimator/imu_propagate
    * Input(from LIO): /estimator/imu_propagate
    * Output(to PX4): /mavros/setpoint_velocity/cmd_vel
    * Output(vir): /desired_position /desired_velocity
    * Features: Keyboard control and QP trajectory are also included

* vio_trajectory - Control UAV under SLAM + Search-Based Planning
    * Input(from VIO): /vins_estimator/imu_propagate
    * Input(from LIO): /estimator/imu_propagate
    * Input(from Search-Based Planning): /result
    * Output(to PX4): /mavros/setpoint_velocity/cmd_vel
    * Output(vir): /desired_position /desired_velocity
    * Features: Keyboard control, circular trajectory and Search-Based planning are included

## Learning
* 


## Authors

* **Johnson**


