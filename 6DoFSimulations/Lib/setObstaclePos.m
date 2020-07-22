function setObstaclePos(vrep,clientID,joint_handles,value)
    rtn = simxSetObjectPosition(vrep,clientID,joint_handles(8),joint_handles(1),value,vrep.simx_opmode_oneshot_wait);
    if rtn ~=vrep.simx_return_ok
        disp('Error to set the obstacle position!');
    end
end