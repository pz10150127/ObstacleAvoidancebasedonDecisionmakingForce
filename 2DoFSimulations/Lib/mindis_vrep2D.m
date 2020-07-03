function [pos,dis,mindis,num] = mindis_vrep2D(vrep,clientID,joint_handles,obs)
%Calculate the minimum distance between the obstacle and the robot through vrep
pos=zeros(3,2);
dis=zeros(2,1);
%mindis=0;
[~,p]=simxGetObjectPosition (vrep,clientID,joint_handles(1),-1,vrep.simx_opmode_oneshot_wait);
temp=p;
for i=[2 5]
    [~,p]=simxGetObjectPosition (vrep,clientID,joint_handles(i),-1,vrep.simx_opmode_oneshot_wait);
    pos(:,i)=p;
    [~,dis(i)]=point2line(obs,temp,pos(:,i));
    temp=p;
end
[mindis,num]=min(dis);
end