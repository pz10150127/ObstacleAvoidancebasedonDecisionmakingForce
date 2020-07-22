function [force,potential,selection_force,cos_theta,phi,p] = RepulsiveForce3DSpeed(vrep,clientID,joint_handles,phi,p,v,p0,pos,gamma,rou0,maxf)
%该函数用于计算两点之间的斥力
%force = RepulsiveForce3DSpeed(p,v,p0,gamma,rou0,maxf)
%   p :vector of test position
%   v :speed vector of test position
%   p0:vector of obstacle location
% gamma: The force constant
% rou0: The range of force effect
%phi=0;
p = reshape(p,3,1);
v = reshape(v,3,1);
p0 = reshape(p0,3,1);
[point,d]=point2line(p0,p,pos);
if p==p0
    force =[maxf;maxf;maxf];
    return;
end
rou = norm(p-p0);
cos_theta = dot(v,p-p0)/norm(v)/rou;
potential=0;
force = [0;0;0];
selection_force = [0;0;0];
%lama=norm(p-point)+1
    if  (rou<=rou0) && (cos_theta<0)
        %force = double( gamma*(cos_theta^2)/(rou^3)*norm(v)*(p-p0));
          %[p,force]=FRDyn(p,p0,v,gamma,2)
             force=frdyn01(p,p0,v,gamma,2);
        if(rou<0.1)
           % force=[maxf maxf maxf];
           force=maxf*force;
        else
         %force = double( gamma*(cos_theta^2)/(rou^3)*norm(v)*(p-p0));
          force=frdyn01(p,p0,v,gamma,2);
        end
       % [p,force]=FRDyn(p,p0,v,gamma,2)
      
        potential=gamma*cos_theta^2/(rou^3);
   
    %以下代码计算选择力
        repulsive_vector = p-p0;
        %selection_force= selection_force2D(repulsive_vector,1);
     if cos_theta<-0.9
      %selection_force=0.05*(cos_theta^2)/(d^3)*norm(v)*selection_force;
     % selection_force= selection_force2D(repulsive_vector,1);
      %selection_force=0.05*(cos_theta^2)/(d^3)*norm(v)*selection_force3D_1(1*pi,repulsive_vector);
      %selection_force=selection_force3D_1(1*pi,repulsive_vector);
     [selection_force,phi,p]=optiphi(vrep,clientID,joint_handles,p0,force,phi,p,repulsive_vector);  
   % selection_force=0
     end
   end
end

