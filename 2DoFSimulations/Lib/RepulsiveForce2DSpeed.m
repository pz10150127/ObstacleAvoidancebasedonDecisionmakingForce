function [force,potential,selection_force,cos_theta,p] = RepulsiveForce2DSpeed(vrep,clientID,joint_handles,dsflag,b,p,v,p0,pos,gamma,rou0,maxf)
%This function is used to calculate the repulsion and decision-making between the robot and the obstacle
%   vrep,clientID,joint_handles :Parameters related to Vrep
%   p :vector of test position
%   v :speed vector of test position
%   p0:vector of obstacle location
% gamma: The force constant
% rou0: The range of force effect
%phi=0;
p = reshape(p,3,1);
v = reshape(v,3,1);
p0 = reshape(p0,3,1);
[~,d]=point2line(p0,p,pos);
if p==p0
    force =[maxf;maxf;maxf];
    return;
end
rou = norm(p-p0);
cos_theta = dot(v,p-p0)/norm(v)/rou;
potential=0;
force = [0 0 0];
selection_force = [0;0;0];
repulsive_vector = p-p0;
if  (d<=0.11)
    if(dsflag==0)
        force=frdyn01(p,p0,v,gamma,2);
    else
        force= frstatic(p,p0,v,gamma,cos_theta,rou);
    end
    if(d<0.05)
        force=maxf*force;
    else
    end
    %Get decision-making force
    if  cos_theta<-0.95 && b>0
        [selection_force,~] = optiphi2D(vrep,clientID,joint_handles,p0,force,p,repulsive_vector);
    else
        selection_force = [0;0;0];
    end
end

end

