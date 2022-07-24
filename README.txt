We provide the codes of Arm/Ant/Snake robot. 
The codes of Arm/Ant are written by python.
The code of Snake is written by MATLAB. 

Note that 

Each folder concludes the folder of the program for the experiment and one of the results. 

##########
Arm

1. Please open ``main_arm.py''.
2. Please set t_max to 0.5 or 1.0. 
3. Please set sw_method (sw_method=0: existing method, sw_method=1: proposed method, sw_method=2: Constant temperature parameter )
4. Please set hyper_param. (this value is h or alpha or lambda). 
5. Please run ``main_arm.py''.
Note: All of the parameter to set is in ``main_arm.py''.

[S_save,P_sum_save,P_save_all,theta_save,Sigma_save,Lambda_h_save,theta_test_initial]
S_save             : trajectory of sum_i(J_{k,i}) against update
mu_save            : trajectory of mu against update
P_save_all         : trajectory of P_{k,i}
Sigma_save         : trajectory of Sigma against update
Lambda_h_save      : trajectory of lambda or h against update
theta_test_initial : initial policy parameter for test

##########
Ant

1. Please install the package of pybullet and gym.
2. Please replace the ``gym_locomotion_envs.py'' in ``anaconda3/envs/(the name of virtual environment)/Lib/site-packages/pybullet_envs''
   for one in the provided folder ``Ant''.
3. Please open ``main_ant_constant_proposed.py''(constant/proposed method) or ``main_arm_existing.py''(existing method).

(if open ``main_ant_constant_proposed.py'')
4a. Please set sw_Lambda (True: proposed method, False: Constant lambda). 
5a. Please set hyper_param (this value is alpha or lambda). 
6a. Please run ``main_ant_constant_proposed.py''.

(if open ``main_arm_existing.py'')
4b. Please set h. 
5b. Please run ``main_arm_existing.py''.

theta0 : initial mu
Sigma0 : initial Sigma
theta_test_set: trajectory of theta_k against update
S_set         : trajectory of sum_i(J_{k,i}) against update
theta_save    : trajectory of mu against update
Sigma_save    : trajectory of Sigma against update
Lambda_save   : trajectory of lambda against update (only proposed method)



##########
Snake

1. Please install Coppeliasim.
2. Please launch Coppeliasim and set ``Snake_robot.ttt''. 
3. Please open ``RL_snake_initial.m'' and set paramRL.sw_method and paramRL.param_update. 
4. Please run ``main_snake.m''.


There are all of the results of learning in struct ``result_save''. 

result_save.mu_save              : trajectory of mu against update
result_save.Sigma_save           : trajectory of Sigma against update
result_save.theta_test_save      : trajectory of theta_k against update
result_save.sum_J_save           : trajectory of sum_i(J_{k,i}) against update
result_save.lambda_proposed_save : trajectory of lambda against update (proposed method only)