function setJointTarget2D(vrep,clientID,joint_handles,value)
%%%%Set joint position
for i =1:2
    rtn = simxSetJointPosition(vrep,clientID,joint_handles(i),value(i),vrep.simx_opmode_oneshot);
    if rtn ~=vrep.simx_return_ok
        disp('Error to set the joint position!');
    end
end
vrep.simxPauseCommunication(clientID, 0);
end