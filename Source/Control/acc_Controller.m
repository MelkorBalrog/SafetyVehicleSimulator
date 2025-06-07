classdef acc_Controller < handle
    % ACC Controller that modulates acceleration output from a PID controller
    % when approaching a curve. It reduces speed to a fraction of the
    % approach speed and predicts trailer rotation assuming simple bicycle
    % model kinematics. Deceleration begins when the vehicle is within a
    % configurable time of the next curve, determined via map localization.

    properties
        speedReduction (1,1) double {mustBePositive} = 0.75
        maxDecel (1,1) double {mustBePositive} = 2.0
        trailerLength (1,1) double {mustBeNonnegative} = 12.0
        wheelbase (1,1) double {mustBePositive} = 3.0
        decelLookaheadTime (1,1) double {mustBePositive} = 5.5
    end

    properties(Access=private)
        decelActive(1,1) logical = false
        baseSpeed(1,1) double = NaN
    end

    methods
        function obj = acc_Controller(speedReduction, maxDecel, trailerLength, wheelbase, lookaheadTime)
            if nargin >= 1 && ~isempty(speedReduction); obj.speedReduction = speedReduction; end
            if nargin >= 2 && ~isempty(maxDecel); obj.maxDecel = maxDecel; end
            if nargin >= 3 && ~isempty(trailerLength); obj.trailerLength = trailerLength; end
            if nargin >= 4 && ~isempty(wheelbase); obj.wheelbase = wheelbase; end
            if nargin >= 5 && ~isempty(lookaheadTime); obj.decelLookaheadTime = lookaheadTime; end
        end

        function [accelOut, predictedRotation] = adjust(obj, currentSpeed, pidAccel, distToCurve, turnRadius, dt)
            if nargin < 5 || isempty(turnRadius)
                turnRadius = inf;
            end
            if nargin < 6
                dt = 0.01;
            end

            if ~obj.decelActive
                obj.baseSpeed = currentSpeed; % remember speed before decel
            end

            decel = max(obj.maxDecel, -pidAccel); % use pid accel if stronger braking
            targetSpeed = obj.speedReduction * obj.baseSpeed;

            triggerDist = currentSpeed * obj.decelLookaheadTime;
            if distToCurve <= triggerDist
                obj.decelActive = true;
            elseif isinf(turnRadius)
                % reset when road straightens and far from curve
                obj.decelActive = false;
            end

            if obj.decelActive
                if currentSpeed > targetSpeed
                    accelOut = -decel;
                elseif currentSpeed < targetSpeed
                    accelOut = max(pidAccel, 0); % do not accelerate above target
                else
                    accelOut = 0;
                end
            else
                accelOut = pidAccel;
            end

            if isinf(turnRadius)
                predictedRotation = 0;
            else
                predictedRotation = currentSpeed * dt / turnRadius * (obj.trailerLength / obj.wheelbase);
            end
        end
    end

    methods(Access=private)
        function dist = computeStoppingDistance(~, v0, v1, decel)
            if decel <= 0
                dist = inf;
                return;
            end
            if v0 <= v1
                dist = 0;
            else
                dist = (v0^2 - v1^2) / (2*decel);
            end
        end
    end
end
