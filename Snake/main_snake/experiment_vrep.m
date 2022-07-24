function [HeadPosition,log_yaw_forces_,log_pitch_forces_]=experiment_vrep(param,paramRL)

% set tentative value for save data
param.total_step_ = floor(param.total_step_);
HeadPosition = zeros(paramRL.K,6, param.total_step_);
log_yaw_forces_ = zeros(paramRL.K,param.num_yaw_joint_, param.total_step_); % yaw joint forces
log_pitch_forces_ = zeros(paramRL.K,param.num_pitch_joint_, param.total_step_); % pitch joint forces


% initial setting of vrep
[vrep,head_handle_,yaw_body_1_handle_,yaw_joint_handle_,pitch_joint_handle_,clientID]=vrep_initial(param);


k=1;
while k<=paramRL.K

    if rem(k,20)==0
        disp(['experiment k:',num2str(k)])
    end

    %% initial setting of DMP
    paramDMP=DMP_initial(paramRL);
    t_sim_old=0;
    [set_angle_, set_angle_vel_,paramDMP] = calc_target(paramDMP,paramRL,k,0,0);
    set_angle_ = Translation(set_angle_, 'Vrep');
    set_angle_vel_ = Translation(set_angle_vel_, 'Vrep');

    %% initialize the VREP model
    for l = 1:param.num_link_
        if l<=param.num_yaw_joint_
            vrep.simxSetJointTargetVelocity(clientID,yaw_joint_handle_(l,1),set_angle_(1,2*l),vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,yaw_joint_handle_(l,1),set_angle_(1,2*l),vrep.simx_opmode_blocking);
            vrep.simxSetJointPosition(clientID,yaw_joint_handle_(l,1),set_angle_(1,2*l),vrep.simx_opmode_blocking);
        end
        if l<=param.num_pitch_joint_
            vrep.simxSetJointPosition(clientID,pitch_joint_handle_(l,1),set_angle_(1,2*l-1),vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetPosition(clientID,pitch_joint_handle_(l,1),set_angle_(1,2*l-1),vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,pitch_joint_handle_(l,1),set_angle_(1,2*l-1),vrep.simx_opmode_blocking);
        end
        pause(0.01)
    end

    pre_head_yaw_ = 0; 
    init_euler_angles_ = [0; 0; 0];
    % set the initial posture of snake robot (euler angle)
    [errorCode]=vrep.simxSetObjectOrientation(clientID, yaw_body_1_handle_, -1, init_euler_angles_, vrep.simx_opmode_oneshot);

    % set the initial posture of snake robot (x-y-z)
    init_cartesian_ = [0;0; 0.1];
    [errorCode]=vrep.simxSetObjectPosition(clientID, yaw_body_1_handle_, -1, init_cartesian_, vrep.simx_opmode_oneshot);


    %% simulation start
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    first_time_ = vrep.simxGetLastCmdTime(clientID)/1000;

    % set the parameter relating time
    count_ = 0;
    old_time_ = -param.time_step_;
    t_sim = 0;
    time_vrep_ = -param.waiting_time_;
    control_time_ = 0;
    flag_finish_ = 0;
    
    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot);

    %% simulation
    while (flag_finish_ == 0) 
                
        while t_sim - old_time_ < param.time_step_*0.99
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
            vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot);
            t_sim = vrep.simxGetLastCmdTime(clientID)/1000 - first_time_;
        end
        old_time_ = old_time_ + param.time_step_; 
        
        if count_>param.total_step_
            break
        end

        t_sim = vrep.simxGetLastCmdTime(clientID)/1000 - first_time_; % Get the time from Vrep start

        % get the position of head
        [~,res] = vrep.simxGetObjectPosition(clientID,head_handle_,-1,vrep.simx_opmode_streaming);
        head_position_(1:3,1) = res;
        [~,res] = vrep.simxGetObjectOrientation(clientID,head_handle_,-1,vrep.simx_opmode_streaming);
        head_position_(4:6,1) = res;

        % correct the angle range to (-pi< angle <=pi)
        [head_position_,pre_head_yaw_]=revise_angle(head_position_,pre_head_yaw_);

        % get the angle of joint
        for l = 1:param.num_yaw_joint_
            [~,res] = vrep.simxGetJointPosition(clientID,yaw_joint_handle_(l),vrep.simx_opmode_streaming);
            yaw_angle_(l,1) = res;
        end
        for l=1:param.num_pitch_joint_
            [~,res] = vrep.simxGetJointPosition(clientID,pitch_joint_handle_(l),vrep.simx_opmode_streaming);
            pitch_angle_(l,1) = res;
        end
        for l = 1:param.num_yaw_joint_
            [~,res] = vrep.simxGetJointForce(clientID,yaw_joint_handle_(l),vrep.simx_opmode_streaming);
            yaw_force_(l,1) = res;
        end
        for l=1:param.num_pitch_joint_
            [~,res] = vrep.simxGetJointForce(clientID,pitch_joint_handle_(l),vrep.simx_opmode_streaming);
            pitch_force_(l,1) = res;
        end

        time_vrep_ = t_sim - param.waiting_time_;  

        %% calculate the target angle
        if time_vrep_ <= 0 
        else
            [set_angle_, set_angle_vel_,paramDMP] = calc_target(paramDMP,paramRL,k,t_sim,t_sim_old);
            t_sim_old=t_sim;
            set_angle_ = Translation(set_angle_, 'Vrep');
            set_angle_vel_ = Translation(set_angle_vel_, 'Vrep');
        end 


        % send the target angle to vrep
        for l=1:param.num_yaw_joint_
            vrep.simxSetJointTargetVelocity(clientID,yaw_joint_handle_(l,1),set_angle_vel_(1,2*l),vrep.simx_opmode_streaming); % 実機は速度制御未実装だが速度指令を与えないとシミュレーションがうまくいかない
            vrep.simxSetJointTargetPosition(clientID,yaw_joint_handle_(l,1),set_angle_(1,2*l),vrep.simx_opmode_streaming);
        end
        for l=1:param.num_pitch_joint_
            vrep.simxSetJointTargetPosition(clientID,pitch_joint_handle_(l,1),set_angle_(1,2*l-1),vrep.simx_opmode_streaming); 
            vrep.simxSetJointTargetVelocity(clientID,pitch_joint_handle_(l,1),set_angle_vel_(1,2*l-1),vrep.simx_opmode_streaming);
        end
        
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

        %% save the data of simulation
        if count_ > 0 
            HeadPosition(k,:,count_)=head_position_(:,1); % head position
            log_yaw_forces_(k,:,count_)=yaw_force_(:,1); % yaw joint forces
            log_pitch_forces_(k,:,count_)=pitch_force_(:,1); % yaw joint forces
        end
                
        % generate the exit flag
        if t_sim >= param.end_time_+param.waiting_time_
            flag_finish_ = 1;
        end

        count_=count_+1;

    end

    sw_isnan=sum(sum(isnan(head_position_)))+sum(sum(isnan(yaw_force_)))+sum(sum(isnan(pitch_force_)));
    
    % re-conduct the experiment when a glitch occurs (sometimes snake robot goes through the floor)
    if min(min(squeeze(HeadPosition(:,3,:))+0.1))>=0 && sw_isnan==0
        % if glitch is not occured, conduct the next simulation
        k=k+1;
        
    else
        % if glitch is not occured, retry the experiment. 
        disp('restart (z:negative)')
        
        % change the policy parameter for experiment
        paramRL.theta_test(k,:)=mvnrnd(paramRL.mu,paramRL.Sigma,1);
        
        % reconnect to VREP, and Stopping the Simulation and restart
        stop_vrep(param,vrep,clientID,yaw_joint_handle_)
        [vrep,head_handle_,yaw_body_1_handle_,yaw_joint_handle_,pitch_joint_handle_,clientID]=vrep_initial(param);
    end
    
end


    % Stopping the Simulation
    stop_vrep(param,vrep,clientID,yaw_joint_handle_)

end




%% function of correct angle

function [head_position_,pre_head_yaw_]=revise_angle(head_position_,pre_head_yaw_)

        % correct the angle range to (-pi< angle <=pi)
        delta_ = head_position_(6) - pre_head_yaw_;
        while norm(delta_) > pi
            if delta_ < -pi
                head_position_(6) = head_position_(6) + 2*pi;
            else 
                head_position_(6) = head_position_(6) - 2*pi;
            end
            delta_ = head_position_(6) - pre_head_yaw_;
        end
        pre_head_yaw_ = head_position_(6);
end




