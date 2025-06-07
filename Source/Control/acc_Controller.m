classdef acc_Controller < handle
    % ACC Controller that modulates acceleration output from a PID controller
    % when approaching a curve. It computes the stopping distance required to
    % reduce speed from the current value to a fraction of it and predicts the
    % trailer rotation assuming simple bicycle model kinematics.

    properties
        speedReduction (1,1) double {mustBePositive} = 0.75
        maxDecel (1,1) double {mustBePositive} = 2.0
        trailerLength (1,1) double {mustBeNonnegative} = 12.0
        wheelbase (1,1) double {mustBePositive} = 3.0
    end

    methods
        function obj = acc_Controller(speedReduction, maxDecel, trailerLength, wheelbase)
            if nargin >= 1 && ~isempty(speedReduction); obj.speedReduction = speedReduction; end
            if nargin >= 2 && ~isempty(maxDecel); obj.maxDecel = maxDecel; end
            if nargin >= 3 && ~isempty(trailerLength); obj.trailerLength = trailerLength; end
            if nargin >= 4 && ~isempty(wheelbase); obj.wheelbase = wheelbase; end
        end

        function [accelOut, predictedRotation] = adjust(obj, currentSpeed, pidAccel, distToCurve, turnRadius, dt)
            if nargin < 5 || isempty(turnRadius)
                turnRadius = inf;
            end
            if nargin < 6
                dt = 0.01;
            end

            targetSpeed = obj.speedReduction * currentSpeed;
            decel = max(obj.maxDecel, -pidAccel); % use pid accel if stronger braking
            stopDist = obj.computeStoppingDistance(currentSpeed, targetSpeed, decel);

            if distToCurve <= stopDist
                accelOut = -decel;
            else
                accelOut = pidAccel;
            end

            if isinf(turnRadius)
                predictedRotation = 0;
            else
                predictedRotation = currentSpeed * dt / turnRadius;
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
