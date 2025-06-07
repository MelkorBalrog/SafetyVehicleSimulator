%{
 @file curveSpeed_Limiter.m
 @brief Limits commanded speed when approaching a curve.
        Ramps the target speed down to a fraction of the PID commanded
        speed as the vehicle gets within a specified time to a curve
        and ramps it back up smoothly after the curve.
%}

classdef curveSpeed_Limiter < handle
    properties
        % Reduction factor applied at the centre of the curve (0-1)
        reductionFactor
        % Time in seconds to ramp from full speed to the reduced speed
        rampDownTime
        % Maximum acceleration used when ramping back up (m/s^2)
        maxAccel

        % Internal state: current reduction multiplier (0-1)
        currentFactor
    end

    methods
        function obj = curveSpeed_Limiter(reductionFactor, rampDownTime, maxAccel)
            if nargin < 1 || isempty(reductionFactor)
                reductionFactor = 0.75; % 25% reduction
            end
            if nargin < 2 || isempty(rampDownTime)
                rampDownTime = 5.5; % seconds
            end
            if nargin < 3 || isempty(maxAccel)
                maxAccel = 2; % m/s^2 for ramp up
            end

            obj.reductionFactor = reductionFactor;
            obj.rampDownTime    = rampDownTime;
            obj.maxAccel        = maxAccel;
            obj.currentFactor   = 1.0;
        end

        function limitedSpeed = limitSpeed(obj, currentSpeed, targetSpeed, distToCurve, inCurve, dt)
            % limitSpeed Applies ramping to the target speed based on distance
            % to the upcoming curve and whether the vehicle is currently in a
            % curve.
            %
            % Parameters:
            %   currentSpeed - Current vehicle speed (m/s)
            %   targetSpeed  - PID commanded speed (m/s)
            %   distToCurve  - Distance to the start of the next curve (m)
            %   inCurve      - Logical flag indicating if the vehicle is in a curve
            %   dt           - Timestep of the simulation (s)

            if nargin < 5
                error('limitSpeed requires current speed, target speed, distance to curve and curve flag.');
            end
            if nargin < 6
                dt = 0.01; % default small timestep
            end

            if inCurve
                obj.currentFactor = obj.reductionFactor;
            else
                timeToCurve = distToCurve / max(currentSpeed, eps);
                if timeToCurve <= obj.rampDownTime && distToCurve >= 0
                    factor = obj.reductionFactor + ...
                        (1 - obj.reductionFactor) * (timeToCurve / obj.rampDownTime);
                    obj.currentFactor = min(obj.currentFactor, factor);
                else
                    % Ramp up towards 1 using maxAccel constraint
                    rate = obj.maxAccel * dt / max(targetSpeed, eps);
                    obj.currentFactor = min(1, obj.currentFactor + rate);
                end
            end

            limitedSpeed = targetSpeed * obj.currentFactor;
        end
    end
end
