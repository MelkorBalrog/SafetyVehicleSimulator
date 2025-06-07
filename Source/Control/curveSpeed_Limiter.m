%{
 @file curveSpeed_Limiter.m
 @brief Limits commanded speed when approaching a curve.
        Applies a predefined deceleration profile when approaching a
        curve: -1 m/s^2 for 0.5 s followed by -4.5 m/s^2 for 1 s.
        After the curve it ramps the speed back up using +6 m/s^2.
%}

classdef curveSpeed_Limiter < handle
    properties
        % Reduction factor applied at the centre of the curve (0-1)
        reductionFactor
        % Time in seconds prior to a curve when ramping begins
        rampDownTime
        % Maximum acceleration used when ramping back up (m/s^2)
        rampUpAccel

        % Internal state: current reduction multiplier (0-1)
        currentFactor
    end

    methods
        function obj = curveSpeed_Limiter(reductionFactor, rampDownTime, rampUpAccel)
            if nargin < 1 || isempty(reductionFactor)
                reductionFactor = 0.75; % 25% reduction
            end
            if nargin < 2 || isempty(rampDownTime)
                rampDownTime = 1.5; % seconds (0.5s at -1 m/s^2, 1s at -4.5 m/s^2)
            end
            if nargin < 3 || isempty(rampUpAccel)
                rampUpAccel = 6; % m/s^2 for ramp up phase
            end

            obj.reductionFactor = reductionFactor;
            obj.rampDownTime    = rampDownTime;
            obj.rampUpAccel     = rampUpAccel;
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
                % Piecewise deceleration profile when approaching a curve
                if timeToCurve <= obj.rampDownTime && distToCurve >= 0
                    if timeToCurve > 1.0
                        % First 0.5 s: -1 m/s^2
                        elapsed = obj.rampDownTime - timeToCurve;
                        deltaV = 1 * elapsed;
                    else
                        % Second 1 s: -4.5 m/s^2
                        elapsed = obj.rampDownTime - 1.0;
                        deltaV = 1 * 0.5 + 4.5 * (1.0 - timeToCurve);
                    end
                    factor = 1 - deltaV / max(targetSpeed, eps);
                    factor = max(obj.reductionFactor, min(1, factor));
                    obj.currentFactor = min(obj.currentFactor, factor);
                else
                    % Ramp up using specified acceleration
                    rate = obj.rampUpAccel * dt / max(targetSpeed, eps);
                    obj.currentFactor = min(1, obj.currentFactor + rate);
                end
            end

            limitedSpeed = targetSpeed * obj.currentFactor;
        end
    end
end
