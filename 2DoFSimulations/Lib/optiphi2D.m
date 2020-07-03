function [selection_force,phi] = optiphi2D(vrep,clientID,joint_handles,obs,Fr,xnow,repulsive_vector)
%%%%%%%%%%%%%%%Used to calculate decision force and decision angle
dis=zeros(1,2);
s=[];
p=[];
for j=1:2
    if j<2
        phi=1;
    else
        phi=-1;
    end
    selection_force=selection_force2D(repulsive_vector,phi);
    force=reshape(Fr,3,1)+selection_force;
    force=reshape(force,3,1);
    setTaskTarget2D(vrep,clientID,joint_handles,xnow+0.01*force);
    [~,~,mindis,~]=mindis_vrep2D(vrep,clientID,joint_handles,obs);
    dis(j)=mindis;
    s(j,:)=selection_force;
    p(j,:)=phi;
end
[~,num]=max(dis);
selection_force=s(num,:);
phi=p(num,:);
end