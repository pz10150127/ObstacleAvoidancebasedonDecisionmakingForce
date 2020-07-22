function [selection_force,phi,dis] = optiphi(vrep,clientID,joint_handles,obs,Fr,phi,xnow,repulsive_vector)
%%%在phi的左右随机搜索5个值，然后通过比较最小距离。
    
    dis=zeros(1,10);
    s=[];
    p=[];
    for j=1:30
    if j<15
       phi=phi+rand(1,1)*pi;       
    else      
       phi=phi-rand(1,1)*pi;         
    end   
    selection_force=selection_force3D_1(phi,repulsive_vector);
    
    force=reshape(Fr,3,1)+selection_force;
    force=reshape(force,3,1);
    %dis(j)=x+force;
    setTaskTarget(vrep,clientID,joint_handles,xnow+0.01*force);
    [~,~,mindis,~]=mindis_vrep(vrep,clientID,joint_handles,obs);
    dis(j)=mindis;
    s(j,:)=selection_force;
    p(j,:)=phi;
    end
    [~,num]=max(dis);
    selection_force=s(num,:);
    phi=p(num,:);    
end

