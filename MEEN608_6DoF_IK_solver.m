%%% MEEN 612
%%% Semester Project for a 6R Robot Arm
%%% Due 5/4/23
%%% By James Puckett



%% Please input these values first
kinematicType = 'i'; % put 'i' for inverse or 'f' for forward
data1 = 30/80*pi; % x location or joint 1 depending on selected kinematic type
data2 = 30/80*pi; % y or jnt2
data3 = 30/80*pi; % z or jnt3
data4 = 30/80*pi; % roll(rad) or jnt4
data5 = 30/80*pi; % pitch(rad) or jnt5
data6 = 30/80*pi; % yaw(rad) or jnt6

% constants
clear input
clear pos
clear selectedConfig
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;

%% Characteristics of the Robot

% DH Table
DH =[0     0      1.57   theta1; ... #(radians inch inch radians)
     pi/2  8.5    0      theta2; ...
     0     8.5    0      theta3; ...
     0     8.5    0      theta4; ...
     pi/2  2.065  2.065  theta5; ...
    -pi/2  2.065  2.065  theta6];

% Define the robot
robot = rigidBodyTree;
bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},DH(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

%% Solve the kinematics of this robot

% set up the robot
config = robot.homeConfiguration;
config(1).JointPosition = data1;
config(2).JointPosition = data2;
config(3).JointPosition = data3;
config(4).JointPosition = data4;
config(5).JointPosition = data5;
config(6).JointPosition = data6;

if (kinematicType == 'i') %% Solve the inverse kinematics

    % solve the inverseKinematics
    tform = getTransform(robot,config,'body6','base');
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = robot.homeConfiguration;
    [configSolution, solutionInfo] = ik('body6', tform, weights, initialguess);
    fprintf('The inverse kinematic solutions are:')
    configSolution.JointPosition

    % compute the Jacobian
    J = Jacobian(configSolution(1).JointPosition, ...
                 configSolution(2).JointPosition, ...
                 configSolution(3).JointPosition, ...
                 configSolution(4).JointPosition, ...
                 configSolution(5).JointPosition, ...
                 configSolution(6).JointPosition);

    % Prepare to compute transformations
    theta1 = configSolution(1).JointPosition;
    theta2 = configSolution(2).JointPosition;
    theta3 = configSolution(3).JointPosition;
    theta4 = configSolution(4).JointPosition;
    theta5 = configSolution(5).JointPosition;
    theta6 = configSolution(6).JointPosition;


elseif (kinematicType == 'f') %% Solve the forward kinematics

    % compute the Jacobian
    J = Jacobian(data1, data2, data3, data4, data5, data6);

    % solve the forward kinematics
    tform = getTransform(robot, config, 'base', 'body6');
    fprintf('The forward kinematic solutions are:')
    translationXYZ = tform2trvec(tform)
    rotationZYX = tform2eul(tform)

    % Prepare to compute transformations
    theta1 = data1;
    theta2 = data2;
    theta3 = data3;
    theta4 = data4;
    theta5 = data5;
    theta6 = data6;

else

    fprintf('Error - Incorrect kinematic type')

end

%% Compute the tf
% tf from wrist to Link 1
tf12 = transformation(DH, 2);
tf23 = transformation(DH, 3);
tf34 = transformation(DH, 4);
tf45 = transformation(DH, 5);
tf56 = transformation(DH, 6);
tf16 = tf12 * tf23 * tf34 * tf45 * tf56;


%% Function definitions
function tf = transformation(DH, i)
    % finds the transformation from frame i to i-1
    tf = [cos(DH(i,4))              -sin(DH(i,4))                 0               DH(i,2);
          sin(DH(i,4))*cos(DH(i,1))  cos(DH(i,4))*cos(DH(i-1,1)) -sin(DH(i,1))   -(sin(DH(i,1)))*DH(i,3);
          sin(DH(i,4))*sin(DH(i,1))  cos(DH(i,4))*sin(DH(i-1,1))  cos(DH(i,1))    (cos(DH(i,1)))*DH(i,3);
          0                          0                            0               1];
end

function J = Jacobian(t1,t2,t3,t4,t5,t6)
    % Robot characteristic lengths
    L1 = 8.5;
    L2 = 8.5;
    L3 = 8.5;
    L4 = 2;
    L5 = 2;
    L6 = 1;

    %these trig functions currently accepts radians
    level6c = L6*cos(t1+t2+t3+t4+t5+t6);
    level6s = L6*sin(t1+t2+t3+t4+t5+t6);
    level5c = L5*cos(t1+t2+t3+t4+t5)+level6c;
    level5s = L5*sin(t1+t2+t3+t4+t5)+level6s;
    level4c = L4*cos(t1+t2+t3+t4)+level5c;
    level4s = L4*sin(t1+t2+t3+t4)+level5s;
    level3c = L3*cos(t1+t2+t3)+level4c;
    level3s = L3*sin(t1+t2+t3)+level4s;
    level2c = L2*cos(t1+t2)+level3c;
    level2s = L2*sin(t1+t2)+level3s;
    level1c = L1*cos(t1)+level2c;
    level1s = L1*sin(t1)+level2s;
    J = zeros(2,6);
    J = [-level1s -level2s -level3s -level4s -level5s -level6s;
        level1c level2c level3c level4c level5c level6c];
end
