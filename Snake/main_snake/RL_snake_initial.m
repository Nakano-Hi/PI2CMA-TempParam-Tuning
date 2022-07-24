function [param,constant_,paramRL,result_save]=RL_snake_initial()


    paramRL = struct();
    paramRL.K=200;             % population
    paramRL.num_update=100;    % the number of update per experiment
    paramRL.M=5; % the number of policy parameter per joint
    
    
    % select the method(1:proposed method, 2:existing method)
    paramRL.sw_method=1; 
    if  paramRL.sw_method==1
        paramRL.param_update=0.9; % alpha
        paramRL.lambda_proposed=1; % initial lambda(tentative value)           
    elseif paramRL.sw_method==2 % existing value
        paramRL.param_update=5; % h
        paramRL.lambda_proposed=-9999; % tentative value (non use)        
    end
    

    param.num_pitch_joint_ = 8;  % The number of pitch joints
    param.num_yaw_joint_ = 8;  % The number of yaw joints
    param.num_joint_ = param.num_pitch_joint_ + param.num_yaw_joint_;  % the number of joints
    param.num_link_ = 8; % the number of links
    param.head_position_ = zeros(6, 1);    % pos. and orientation of the first unit


    
    param.time_step_ = 0.050;   % time step
    param.waiting_time_ = 0.1; % waiting time for itinialization
    param.end_time_ = 8; % time to finish [s]
    param.total_step_ = (param.end_time_ + param.waiting_time_) / param.time_step_;   % total number of steps
    param.t_set=0:param.time_step_:param.end_time_;

    
    

    constant_ = struct();
    constant_.num_joint_ = param.num_joint_; % the number of joint
    constant_.time_ = 0; % initial time
    constant_.delta_time_ = param.time_step_; % 1 period time of control


    
    
    
    paramRL.L=param.num_joint_; % the numbers of joints
    paramRL.num_param=paramRL.M*paramRL.L; % the number of policy parameters
    paramRL.mu=zeros(1,paramRL.num_param); % initial mu
    paramRL.Sigma=eye(paramRL.num_param)*10; % initial covariance matrix
    
       
        
    
    % set tentative value for save data
    result_save= struct();
    result_save.K=paramRL.K;
    result_save.time_step=param.time_step_;
    result_save.endtime=param.end_time_;
    result_save.num_yaw_joint_=param.num_yaw_joint_;
    result_save.num_update=paramRL.num_update;
    result_save.sum_J_save=zeros(paramRL.num_update,paramRL.K);
    result_save.mu_save=zeros(paramRL.num_update,paramRL.M*paramRL.L);
    result_save.Sigma_save=zeros(paramRL.num_update,paramRL.M*paramRL.L,paramRL.M*paramRL.L);
    result_save.param_update=paramRL.param_update;
    result_save.lambda_proposed_save=zeros(1,paramRL.num_update);
    
    % other tentative value
    paramRL.Sigma_before_update=paramRL.Sigma;
    param.yaw_angle_ = zeros(param.num_yaw_joint_, 1);      % yaw joint angles
    param.pitch_angle_ = zeros(param.num_pitch_joint_, 1);      % pitch joint angles
    param.yaw_force_ = zeros(param.num_yaw_joint_, 1);      % yaw joint forces

end