clc;
clear;
clf;
close all;
format compact

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [140,280,360,480,600]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1.2 1 -6;
             1.5 0 -5;
             0 1.5 -7;
             -1 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

% Constants
% Radians to degree
RadianToDegree = 180/pi; 
% Degree to radians
DegreeToRadian = pi/180;

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]

% Initial state of quadcopter
quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
quadcopterInitState.BodyXYZVelocity.X = 0;            
quadcopterInitState.BodyXYZVelocity.Y = 0;            
quadcopterInitState.BodyXYZVelocity.Z = 0;
quadcopterInitState.BodyEulerAngle.Phi = 0;
quadcopterInitState.BodyEulerAngle.Theta = 0;
quadcopterInitState.BodyEulerAngle.Psi = 0;
quadcopterInitState.BodyAngularRate.dPhi = 0;
quadcopterInitState.BodyAngularRate.dTheta = 0;
quadcopterInitState.BodyAngularRate.dPsi = 0;

% Control variables - total thrust and moments on each control axis
quadcopterInitControlInputs = [0, 0, 0, 0]';     % (T, M1, M2, M3)
                           
% Initiate quadcopter
quadcopter = Quadcopter(Mass, ...               
               XMomentOfInertia, ...
               YMomentOfInertia, ...
               ZMomentOfInertia, ...
               quadcopterInitState, ...
               quadcopterInitControlInputs,...
               deltaT);

% Simulation
for i = 1 : deltaT : simulationTime
    
    currentWaypointIdx = find(timeForWaypointPasage >= i, 1, 'first');
    if isempty(currentWaypointIdx)
        targetPosition = wayPoints(end, :);  % Stay at last waypoint
    else
        targetPosition = wayPoints(currentWaypointIdx, :);
    end
    targetOrientation = [0, 0, 0];  % Keep yaw (Psi) constant

    % Apply control action
    quadcopter.ControlAction(targetPosition, targetOrientation);

    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();
    
    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end
    
    isWaypointPassed(quadcopterActualState.BodyXYZPosition, i * deltaT, timeForWaypointPasage,wayPoints,positionTolerance)

    % Waypoint check
    if (CheckWayPointTrack(...
                quadcopterActualState.BodyXYZPosition,...
                i * deltaT,...
                timeForWaypointPasage,...
                wayPoints,...
                positionTolerance))
        msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
        break;
    end
end                              