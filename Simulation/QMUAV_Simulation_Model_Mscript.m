%% Define Mechanical Constants
ry = 1;
rx = 1;

Iy = 1;
Ix = 1;
Iz = 1;

m = 1;

maxPWM = 255;
minPWM = 0;

%% Define PID parameters
Kp_alt = 0;
Ki_alt = 0;
Kd_alt = 0;

Kp_yaw = 0;
Ki_yaw = 0;
Kd_yaw = 0;

Kp_roll = 0;
Ki_roll = 0;
Kd_roll = 0;

Kp_pitch = 0;
Ki_pitch = 0;
Kd_pitch = 0;

Kp_x = 0;
Ki_x = 0;
Kd_x = 0;

Kp_y = 0;
Ki_y = 0;
Kd_y = 0;

%% Setpoints
setpoint_alt = 0;
setpoint_yaw = 0;
setpoint_x = 0;
setpoint_y = 0;

%% Run Simulation Model
simTime = 10;
maxStepSize = 0.01;
sim('QMUAV_Simulation_Model_V2')

