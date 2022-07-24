
function [vrep,head_handle_,yaw_body_1_handle_,yaw_joint_handle_,pitch_joint_handle_,clientID]=vrep_initial(param)

%% create an instance of vrep class
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)

% set joint names for vrep model
name_yaw_joint_ = {'yawjoint1';'yawjoint2';'yawjoint3';'yawjoint4';'yawjoint5';'yawjoint6';'yawjoint7';'yawjoint8'};
name_pitch_joint_ = {'pitchjoint1';'pitchjoint2';'pitchjoint3';'pitchjoint4';'pitchjoint5';'pitchjoint6';'pitchjoint7';'pitchjoint8';};

%% connect to VREP
vrep.simxFinish(-1); % just in case, close all opened connections

while 1
    % open scene in vrep in advance
    clientID=vrep.simxStart('127.0.0.1', 19997, true, true, 500, 5);
    disp('Trying to connect to v-rep');
    if clientID>-1
        break;
    end
 end
disp('connect to v-rep');

%% define handlers to access vrep model
yaw_joint_handle_ = zeros(param.num_yaw_joint_, 1); % Handlers for yaw joints
pitch_joint_handle_ = zeros(param.num_pitch_joint_, 1); % Handlers for pitch joints
for i = 1:param.num_pitch_joint_
    [~,res] = vrep.simxGetObjectHandle(clientID, char(name_yaw_joint_(i)), vrep.simx_opmode_blocking);
    yaw_joint_handle_(i, 1) = res;
end
for i=1:param.num_pitch_joint_
    [~, res] = vrep.simxGetObjectHandle(clientID, char(name_pitch_joint_(i)), vrep.simx_opmode_blocking);
    pitch_joint_handle_(i, 1) = res;
end
% get handlers of Head and first link
[~, res] = vrep.simxGetObjectHandle(clientID, 'Head', vrep.simx_opmode_blocking);
head_handle_ = res;
[~, res] = vrep.simxGetObjectHandle(clientID, 'yawbody1', vrep.simx_opmode_blocking);
yaw_body_1_handle_ = res;



end