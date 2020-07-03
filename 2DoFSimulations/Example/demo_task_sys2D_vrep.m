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
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('=========OK to GO==========');
tm=[];
FRX=[];
VX=[];
VY=[];
%%%%%%%%%%%%%%%%%%%%%%%%%Generate the desired trajectory%%%%%%%%%%%%%%%%%%%%%%%%
traj_duration = 10;
td = 0:0.1:traj_duration;
xd_array = linspace(-0.4,0.4,length(td));
[dxd_array,ddxd_array]=diff_motion2(td,xd_array);
x0=-0.4;
dx0=0;
yd_array = -0.4*ones(1,length(td));
[dyd_array,ddyd_array]=diff_motion2(td,yd_array);

%%%%%%%%Initialize parameters and set the initial position of the robot%%%%%%%%%%%%%
y0=-0.4;
dy0=0;
X=[];
Y=[];
J1=[];
J2=[];
Frx=[];
Fry=[];
setTaskTarget2D(vrep,clientID,joint_handles,[x0,y0,0]); %Set the robot init position

%%%%%%%%%%%%%%%%%Parameter Settings for program%%%%%%%%%%%%%%%%%%%%%%%

obj =0.1; %Position of obstacle x
objy = -0.39; %Position of obstacle y
b=0.5; %Decision-making force scaling factor,b=0 without decision-making force
a=0.8; %Repulse force scaling factor
dsflag=0;%0 dynamic repulsion field, 1 static repulsion field

%%%%%%%%%%%%%%%%%%%Draw obstacles and set work environment%%%%%%%%%%%%%%%%
obs_pos=[obj,objy];
hold on;
plotCircle(obj,objy,0.1);
plotCircle1(obj,objy,0.05);
plotObstacle(obj,objy,0.025);
titlestr = sprintf('Obstacle Position [%1.2f,%1.2f]',obj,objy);
plot(xd_array,yd_array,'k:'); % Plot the desired trajectory.
setTaskTarget2D(vrep,clientID,joint_handles,[-0.4,-0.4,0]);%
setObstaclePos(vrep,clientID,joint_handles,[obs_pos,0]);%
%%%%%%%%%%%%%%%%Calculate LQR feedback coefficient according to system parameters%%%%%%%%%%%%%%%%
m=0.5;c=0.01;k=0.1;fr=2;
A=[0,1;-k/m,-c/m];
B=[0;1/m];
Q=[400,0;0,20];
R=1;
KK=lqr(A,B,Q,R,0);
K1=KK(1);K2=KK(2);
%%%%%%%%%Load the model and simulate%%%%%%%%%%%%%%%%
xd=xd_array(1);
dxd=dxd_array(1);
ddxd=ddxd_array(1);
yd=yd_array(1);
dyd=dyd_array(1);
ddyd=ddyd_array(1);
frx = 0;
fry = 0;
xnow = x0;
ynow = y0;
dxnow = dx0;
dynow = dy0;
potential =0;
time_scale = 1;
t=0;
mdl = 'sys_2D';
load_system(mdl);
set_param(mdl, 'SimulationCommand', 'stop');
set_param(mdl, 'StopTime', sprintf('%d',10));
set_param(mdl, 'SimulationCommand', 'start');
set_param(mdl, 'SimulationCommand', 'pause');
ppp = [];
fff=[];
last_states=[0,x0,y0,dx0,dy0];
ll=[];

for i =1:1000000
    t_true = get_param(mdl,'SimulationTime');
    if strcmp(get_param(mdl', 'SimulationStatus'),'stopped')
        disp(i);
        break;
    end    
    xd= interp1(td,xd_array,t);
    yd= interp1(td,yd_array,t);   
    if t==0
        dxd= spline(td,dxd_array,t);
        ddxd= spline(td,ddxd_array,t);
        dyd= spline(td,dyd_array,t);
        ddyd= spline(td,ddyd_array,t);
    else
        dxd = (xd-last_states(2))/(t_true-last_states(1));
        ddxd = (dxd-last_states(4))/(t_true-last_states(1));
        dyd = (yd-last_states(3))/(t_true-last_states(1));
        ddyd = (dyd-last_states(5))/(t_true-last_states(1));
    end
    last_states=[t_true,xd,yd,dxd,dyd];
    % Get the actual position of the robot
    [~,pos]=simxGetObjectPosition (vrep,clientID,joint_handles(2),-1,vrep.simx_opmode_oneshot_wait);
    %Calculate repulsion and decision force
    [force,potential,selection_force,cos_theta,p] = RepulsiveForce2DSpeed(vrep,clientID,joint_handles,dsflag,b,[xnow;ynow;0],[dxnow;dynow;0],[obs_pos,0],pos,0.2,0.1,1.);
    frx = double(a*force(1)+b*selection_force(1));
    fry = double(a*force(2)+b*selection_force(2));
    if norm(dxd-obj,dyd-objy)<0.1
        time_scale=2;
    elseif norm(xnow-dxd,ynow-dyd)>0.05
        time_scale=0.05;
    end
    Frx=[Frx,frx];
    Fry=[Fry,fry];  
    set_param(mdl, 'SimulationCommand', 'update');
    set_param(mdl, 'SimulationCommand', 'step');  
    xnow = simout.signals.values(end,1);
    ynow = simout.signals.values(end,3);
    dxnow = simout.signals.values(end,2);
    dynow = simout.signals.values(end,4);
    X=[X xnow];
    Y=[Y ynow];
    VX=[VX,dxnow];
    VY=[VY,dynow];
    t = simout.signals.values(end,5);
    if t>10
        break;
    end
    setTaskTarget2D(vrep,clientID,joint_handles,[xnow,ynow,0]);
    pause(0.0001);
    plot(xnow,ynow,'b.-');
    drawnow;
    [~,r1angle]=vrep.simxGetJointPosition(clientID,joint_handles(1),vrep.simx_opmode_oneshot_wait);
    [~,r2angle]=vrep.simxGetJointPosition(clientID,joint_handles(2),vrep.simx_opmode_oneshot_wait);
    J1=[J1,r1angle];
    J2=[J2,r2angle];
end
set_param(mdl, 'SimulationCommand', 'stop');
quiver(X(1:2:end),Y(1:2:end),Frx(1:2:end),Fry(1:2:end),'r');
legend('Field area','Danger area','Obstacle','Task path','Actual path','Force');
xlabel('X(m)');
ylabel('Y(m)');
pause(2);
save('pathdata.mat','X','Y','Frx','Fry');
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);