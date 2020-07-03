1、demo_task_sys2D_vrep.m Used to generate obstacle avoidance paths.
By adjusting the following parameters, you can make the program work in SRF, DRF, SRDF, DRDF algorithm.
b=0.5; %Decision-making force scaling factor,b=0 without decision-making force
a=0.8; %Repulse force scaling factor
dsflag=0;%0 dynamic repulsion field, 1 static repulsion field

SRF:b=0 a=0.8 dsflag=1;DRF:b=0 a=0.8 dsflag=0;SRDF:b=0.5 a=0.8 dsflag=1;DRDF:b=0.5 a=0.8 dsflag=0.

The following parameters are used to set the position of the obstacle
obj =0.1; %Position of obstacle x
objy = -0.39; %Position of obstacle y    up the path

obj =0.1; %Position of obstacle x
objy = -0.40; %Position of obstacle y    on the path

obj =0.1; %Position of obstacle x
objy = -0.41; %Position of obstacle y    down the path

2、sys2D_path_reproduction.m  Used to reproduce obstacle avoidance paths
