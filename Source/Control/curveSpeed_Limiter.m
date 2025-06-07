%{
% @file curveSpeed_Limiter.m
% @brief Simple limiter that reduces target speed when a curve is detected.
%        When a valid turn radius is provided the target speed is scaled
%        to a fraction of its value.
%}

classdef curveSpeed_Limiter
    % curveSpeed_Limiter Limit target speed when negotiating curves.
    %   limitedSpeed = obj.limitSpeed(targetSpeed, turnRadius) returns a
    %   reduced speed whenever turnRadius is finite and positive.

    properties
        reductionFactor (1,1) double {mustBeGreaterThan(reductionFactor,0),mustBeLessThanOrEqual(reductionFactor,1)} = 0.25
    end

    methods
        function obj = curveSpeed_Limiter(reductionFactor)
            if nargin >= 1
                obj.reductionFactor = reductionFactor;
            end
        end

        function limitedSpeed = limitSpeed(obj, targetSpeed, turnRadius)
            if nargin < 3 || isinf(turnRadius) || turnRadius <= 0
                limitedSpeed = targetSpeed;
            else
                limitedSpeed = targetSpeed * obj.reductionFactor;
            end
        end
    end
end

