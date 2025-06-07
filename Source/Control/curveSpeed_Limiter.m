%{
% @file curveSpeed_Limiter.m
% @brief Simple limiter that reduces target speed on tight curves.
%        The target speed is only reduced when a finite turn radius
%        below 100 meters is provided.
%}

classdef curveSpeed_Limiter
    % curveSpeed_Limiter Limit target speed when negotiating curves.
    %   limitedSpeed = obj.limitSpeed(targetSpeed, turnRadius) returns a
    %   reduced speed whenever turnRadius is finite, positive and below 100 m.

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
            % limitSpeed Reduces the target speed for tight curves.
            %   The speed is scaled by the reductionFactor only when the
            %   provided turnRadius is finite, positive and below 100 m.
            if nargin < 3 || isinf(turnRadius) || turnRadius <= 0 || turnRadius >= 100
                limitedSpeed = targetSpeed;
            else
                limitedSpeed = targetSpeed * obj.reductionFactor;
            end
        end
    end
end

