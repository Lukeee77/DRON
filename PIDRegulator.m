classdef PIDRegulator < handle
    % PIDRegulator implements a simple PID controller
    properties
        Kp % Proportional gain
        Ki % Integral gain
        Kd % Derivative gain

        previousError % Error in the previous step
        integralSum   % Cumulative integral of error

        output % Current output of the PID controller
    end

    methods
        % Constructor
        function obj = PIDRegulator(Kp, Ki, Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;

            obj.previousError = 0;
            obj.integralSum = 0;
            obj.output = 0;
        end

        % Calculate PID action
        function CalculateAction(obj, currentValue, targetValue, deltaTime)
            % Compute error
            error = targetValue - currentValue;

            % Compute proportional term
            proportional = obj.Kp * error;

            % Compute integral term
            obj.integralSum = obj.integralSum + error * deltaTime;
            integral = obj.Ki * obj.integralSum;

            % Compute derivative term
            derivative = obj.Kd * (error - obj.previousError) / deltaTime;

            % Compute total output
            obj.output = proportional + integral + derivative;

            % Store current error for next derivative calculation
            obj.previousError = error;
        end

        % Get current action
        function output = GetCurrentAction(obj)
            output = obj.output;
        end

        % Reset the PID controller
        function Reset(obj)
            obj.previousError = 0;
            obj.integralSum = 0;
            obj.output = 0;
        end
    end
end
