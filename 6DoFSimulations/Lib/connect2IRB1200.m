function [vrep,clientID,joint_handles] = connect2IRB1200()
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
joint_handles = zeros(1,20);
if clientID>-1
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint1',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(1)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint2',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(2)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint3',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(3)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint4',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(4)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint5',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(5)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint6',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(6)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB120_manipulationSphere',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(7)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Obstacle1',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(8)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetDistanceHandle(vrep,clientID,'Min_Distance',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(9)=handle;
    else
        disp('Error to Get the distance handle!');
    end
    [rtn,handle] = simxGetCollisionHandle(vrep,clientID,'Collision_Detection',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(10)=handle;
    else
        disp('Error to Get the obstacle handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Origin',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(11)=handle;
    else
        disp('Error to Get the Origin handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint7',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(12)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint8',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(13)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint9',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(14)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint10',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(15)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint11',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(16)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Joint12',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(17)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB120_manipulationSphere',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(18)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Obstacle2',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(19)=handle;
    else
        disp('Error to Get the joint handle!');
    end
    %          [rtn,handle] = simxGetDistanceHandle(vrep,clientID,'Min_Distance',vrep.simx_opmode_oneshot_wait);
    %         if rtn ==vrep.simx_return_ok
    %             joint_handles(9)=handle;
    %         else
    %             disp('Error to Get the distance handle!');
    %         end
    %         [rtn,handle] = simxGetCollisionHandle(vrep,clientID,'Collision_Detection',vrep.simx_opmode_oneshot_wait);
    %         if rtn ==vrep.simx_return_ok
    %             joint_handles(10)=handle;
    %         else
    %             disp('Error to Get the obstacle handle!');
    %         end
    [rtn,handle] = simxGetObjectHandle(vrep,clientID,'Origin0',vrep.simx_opmode_oneshot_wait);
    if rtn ==vrep.simx_return_ok
        joint_handles(20)=handle;
    else
        disp('Error to Get the Origin handle!');
    end
else
    disp('Error to Connect to the Server!');
end

end

