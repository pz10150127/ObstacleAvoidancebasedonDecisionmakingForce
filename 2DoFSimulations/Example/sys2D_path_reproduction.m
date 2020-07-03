close all;
clear;
clc;
clf;
addpath('../Lib');%Add libaries' path
addpath('../Simulinkmodel');
%%%%%%%%%%%%%%%%%Establish a connection with vrep%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[vrep,clientID,joint_handles] = connect2VREP();

if clientID<=-1
    return;
end
controlIKGroup2D(vrep,clientID,1);
disp('=========OK to GO==========');
objy = -0.39;
obj =0.1;
obs_pos=[obj,objy];
hold on;
plotCircle(obj,objy,0.1);
plotCircle1(obj,objy,0.05);
plotObstacle(obj,objy,0.025);
load pathdata.mat;
for i=1:length(X)
    setTaskTarget2D(vrep,clientID,joint_handles,[X(i),Y(i),0]); %Set the robot init position
    plot(X(i),Y(i),'b.-');
    drawnow;
   pause(0.01);
end
quiver(X(1:2:end),Y(1:2:end),Frx(1:2:end),Fry(1:2:end),'r');
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);