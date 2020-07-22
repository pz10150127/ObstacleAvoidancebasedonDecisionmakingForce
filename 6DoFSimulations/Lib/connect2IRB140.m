function [vrep,clientID,joint_handles] = connect2IRB140()
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    joint_handles = [0,0,0,0,0,0,0,0,0];
    if clientID>-1
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint1',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(1)=handle;
        else
            disp('Error to Get the joint handle!');
        end
        
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint2',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(2)=handle;
        else
            disp('Error to Get the joint handle!');
        end
        
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint3',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(3)=handle;
        else
            disp('Error to Get the joint handle!');
        end    
        
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint4',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(4)=handle;
        else
            disp('Error to Get the joint handle!');
        end 
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint5',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(5)=handle;
        else
            disp('Error to Get the joint handle!');
        end 
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_joint6',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(6)=handle;
        else
            disp('Error to Get the joint handle!');
        end 
        [rtn,handle] = simxGetObjectHandle(vrep,clientID,'IRB140_manipulationSphere',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(7)=handle;
        else
            disp('Error to Get the joint handle!');
        end 
         [rtn,handle] = simxGetObjectHandle(vrep,clientID,'obstacle1',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(8)=handle;
        else
            disp('Error to Get the joint handle!');
        end
         [rtn,handle] = simxGetObjectHandle(vrep,clientID,'obstacle2',vrep.simx_opmode_oneshot_wait);
        if rtn ==vrep.simx_return_ok
            joint_handles(9)=handle;
        else
            disp('Error to Get the joint handle!');
        end 
        
    else
        disp('Error to Connect to the Server!');
    end
end

