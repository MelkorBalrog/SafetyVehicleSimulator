%{
 @file curveSpeed_Limiter.m
 @brief Limits commanded speed when approaching a curve by ramping
        from the current target speed down to a reduced value based on
        stopping distance.
%}

classdef curveSpeed_Limiter < handle
    properties
        % Reduction factor applied at the start of the curve (0-1)
        reductionFactor
        % Maximum comfortable deceleration magnitude (m/s^2)
        maxDecel
    end

    methods
        function obj = curveSpeed_Limiter(reductionFactor, maxDecel)
            if nargin < 1 || isempty(reductionFactor)
                reductionFactor = 0.75; % 25% reduction
            end
            if nargin < 2 || isempty(maxDecel)
                maxDecel = 5; % default deceleration
            end
            obj.reductionFactor = reductionFactor;
            obj.maxDecel = maxDecel;
        end

        function limitedSpeed = limitSpeed(obj, currentSpeed, targetSpeed, distToCurve)
            if nargin < 4
                error('limitSpeed requires current speed, target speed and distance to curve start.');
            end
            stopDist = obj.computeStoppingDistance(currentSpeed, targetSpeed * obj.reductionFactor);
            if distToCurve >= stopDist || stopDist <= 0
                limitedSpeed = targetSpeed;
            else
                ratio = max(0, min(1, distToCurve / stopDist));
                rampFactor = obj.reductionFactor + (1 - obj.reductionFactor) * ratio;
                limitedSpeed = targetSpeed * rampFactor;
            end
        end

        function dist = computeStoppingDistance(obj, v0, v1)
            if v0 <= v1
                dist = 0;
            else
                dist = (v0^2 - v1^2) / (2 * obj.maxDecel);
            end
        end
    end
end
