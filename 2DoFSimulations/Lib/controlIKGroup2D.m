function controlIKGroup2D(vrep,clientID,value)
    vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_ik_handling_enabled,value,vrep.simx_opmode_oneshot_wait);
end