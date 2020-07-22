function setTaskTarget(vrep,clientID,joint_handles,value)
    rtn = simxSetObjectPosition(vrep,clientID,joint_handles(7),joint_handles(1),value,vrep.simx_opmode_oneshot_wait);
    if rtn ~=vrep.simx_return_ok
        disp('Error to set the target position!');
    end
end