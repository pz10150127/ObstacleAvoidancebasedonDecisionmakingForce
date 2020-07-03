function setTaskTarget2D(vrep,clientID,joint_handles,value)
%Set the target position of the robot
rtn = simxSetObjectPosition(vrep,clientID,joint_handles(4),joint_handles(1),value,vrep.simx_opmode_oneshot_wait);
if rtn ~=vrep.simx_return_ok
    disp('Error to set the target position!');
end
end