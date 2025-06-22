# Robust Colision Cone Control Barrier Function (RC3BF)
Implementation of Robust Colision Cone Control Barrier Function for holonomic mobile robot, with kinematic modelled with equations:

![Kinematic Model](fig/kinematic_model.svg)

Where $v$ and $\omega$ are controll inputs.


## Colision Cone Conrtol Barrier Function (C3BF)
Colision Cone Control Barrier Function is defined in following way:

![c3bf](fig/c3bf.svg)

Where $p_r$ and $v_r$ are relative position and velocity of robot and obstacle - as it is presented in figure below.

![collision_cone](fig/colision_cone_fig.png)

And are defined in following way:

![c3bf](fig/pv_relative.svg)

To stay in safe space control needs to met following condition:

![trajectories](fig/condition.svg)

Where derivative is defined as:

![trajectories](fig/h_prim.svg)

Example trajectory of two robots in colision course is presented below:

![trajectories](scripts/robot_trajectory.gif)
