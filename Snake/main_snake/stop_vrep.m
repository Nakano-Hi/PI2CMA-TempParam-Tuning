function stop_vrep(param,vrep,clientID,yaw_joint_handle_)
        %% Stopping the Simulation
    for i = 1:param.num_link_-1
        vrep.simxSetJointForce(clientID,yaw_joint_handle_(i,1), 0,vrep.simx_opmode_blocking);
    end
    disp('Trying to stop sim')
    pause(1);
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    pause(1);
    vrep.simxFinish(clientID);
    pause(1);
    vrep.delete();
    disp('Vrep has stopped')

end