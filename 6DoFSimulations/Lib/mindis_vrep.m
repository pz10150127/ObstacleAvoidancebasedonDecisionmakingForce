function [pos,dis,mindis,num] = mindis_vrep(vrep,clientID,joint_handles,obs)
%ͨ��vrep�����ϰ����������֮�����С����
%   �˴���ʾ��ϸ˵��
pos=zeros(3,6);
dis=zeros(6,1);
%mindis=0;
temp=[0.0690 0 0]';
 for i=1:6      
    [~,p]=simxGetObjectPosition (vrep,clientID,joint_handles(i),-1,vrep.simx_opmode_oneshot_wait);
    pos(:,i)=p;
    [~,dis(i)]=point2line(obs,temp,pos(:,i));
    temp=p;
 end
 [mindis,num]=min(dis);
end

