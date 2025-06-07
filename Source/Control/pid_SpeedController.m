%{
% @file pid_SpeedController.m
% @brief PID-based speed controller with smoothing and cornering logic.
%        Reduces speed for tight turns to avoid skidding.
% @author Miguel Marina
%}

classdef pid_SpeedController < handle
    % pid_SpeedController Controls the speed of the truck using PID control with SMA and Gaussian filtering
    % and reduces desired speed if the turn radius is small (to avoid skidding).
    %
    %   This class computes the required acceleration to maintain the truck's speed
    %   at a desired setpoint while ensuring that the speed does not exceed a 
    %   specified maximum limit. It incorporates:
    %       - Simple Moving Average (SMA) or Gaussian filters (optional)
    %       - PID control
    %       - Turn-radius-based speed reduction
    %
    %   Usage:
    %       controller = pid_SpeedController(desiredSpeed, maxSpeed, Kp, Ki, Kd, ...
    %           maxAccel, minAccel, 'FilterType', 'sma', 'SMAWindowSize', 5, ...
    %           'GaussianWindowSize', 5, 'GaussianStd', 1, ...
    %           'FrictionCoeff', 0.7, 'Gravity', 9.81, 'SafetyFactor', 0.85, ...
    %           'Verbose', false);
    %
    %       % In your main loop:
    %       acceleration = controller.computeAcceleration(currentSpeed, currentTime, turnRadius, upcomingRadii);
    %
    %   Author: [Your Name]
    %   Date:   [Date]

    properties
        % --------------------- PID Control Properties ---------------------
        desiredSpeed        % Desired speed to maintain (m/s)
        maxSpeed            % Maximum allowed speed (m/s)
        Kp                  % Proportional gain (unitless)
        Ki                  % Integral gain (unitless)
        Kd                  % Derivative gain (unitless)
        maxAcceleration     % Maximum allowed acceleration (m/s^2)
        minAcceleration     % Minimum allowed acceleration (m/s^2)
        integral            % Accumulated integral term
        previousError       % Error at the previous time step
        previousTime        % Time at the previous time step
        controllerActive    % Flag to indicate if the controller is active

        % ------------------------ Filtering Props -------------------------
        filterType          % Type of filter: 'none', 'sma', 'gaussian'

        % SMA Filter
        smaWindowSize       % Window size for SMA
        smaBuffer           % Circular buffer for SMA
        smaIndex            % Current index for SMA buffer

        % Gaussian Filter
        gaussianWindowSize  % Window size for Gaussian filter (must be odd)
        gaussianStd         % Standard deviation for Gaussian kernel
        gaussianKernel      % Precomputed Gaussian kernel
        gaussianBuffer      % Circular buffer for Gaussian filter
        gaussianIndex       % Current index for Gaussian buffer

        % -------------------- Skidding-Avoidance Props -------------------
        frictionCoeff  % (mu) friction coefficient for cornering speed calc
        gravity        % gravitational accel (m/s^2)
        safetyFactor   % a factor < 1 to add safety margin to cornering speed

        % ------------------------ Control Flags ---------------------------
        verbose        % Flag to control verbosity of logs

        % -------------------- Speed Profile Props -----------------------
        speedSmoothing     % Smoothing factor (0-1) for target speed updates
        currentTargetSpeed % Internally smoothed target speed

        % -------------------- Jerk Limit Props -------------------------
        jerkLimit         % Maximum allowable jerk (m/s^3)

        % Reduction factor (<1) applied to cornering speed limits
        curveSpeedReduction
    end

    methods
        %% Constructor
        function obj = pid_SpeedController(desiredSpeed, maxSpeed, Kp, Ki, Kd, ...
                maxAcceleration, minAcceleration, varargin)

            if nargin < 7
                error(['Not enough input arguments. Required at least: ', ...
                    'desiredSpeed, maxSpeed, Kp, Ki, Kd, maxAccel, minAccel.']);
            end

            % Assign basic PID and speed properties
            obj.desiredSpeed      = desiredSpeed;
            obj.maxSpeed          = maxSpeed;
            obj.Kp                = Kp;
            obj.Ki                = Ki;
            obj.Kd                = Kd;
            obj.maxAcceleration   = maxAcceleration;
            obj.minAcceleration   = minAcceleration;
            obj.integral          = 0;
            obj.previousError     = 0;
            obj.previousTime      = 0;
            obj.controllerActive  = true;

            % ----------------- Default filtering configs ------------------
            p = inputParser;
            addParameter(p, 'FilterType', 'none', @(x) ismember(x, {'none','sma','gaussian'}));
            addParameter(p, 'SMAWindowSize', 5, @(x) isnumeric(x) && x>0 && floor(x)==x);
            addParameter(p, 'GaussianWindowSize', 5, @(x) isnumeric(x) && x>0 && mod(x,2)==1);
            addParameter(p, 'GaussianStd', 1, @(x) isnumeric(x) && x>0);

            % ---- Speed smoothing when updating target speed -----
            addParameter(p, 'SpeedSmoothing', 0.2, @(x) isnumeric(x) && x>0 && x<=1);

            % ---- Jerk limit for deceleration planning -----
            addParameter(p, 'JerkLimit', 0.7*9.81, @(x) isnumeric(x) && x>0);
            addParameter(p, 'CurveSpeedReduction', 0.75, @(x) isnumeric(x) && x>0 && x<=1);

            % ---- New parameters for friction-based cornering speed  -----
            addParameter(p, 'FrictionCoeff', 0.7, @(x) isnumeric(x) && x>0 && x<=1);
            addParameter(p, 'Gravity', 9.81, @(x) isnumeric(x) && x>0);
            addParameter(p, 'SafetyFactor', 0.85, @(x) isnumeric(x) && x>0 && x<=1);

            % ---- Verbosity Control -----
            addParameter(p, 'Verbose', false, @(x) islogical(x) || isnumeric(x));

            parse(p, varargin{:});

            % Assign filter type
            obj.filterType = p.Results.FilterType;

            % Initialize SMA (if selected)
            obj.smaWindowSize = p.Results.SMAWindowSize;
            if strcmp(obj.filterType, 'sma')
                obj.smaBuffer = zeros(obj.smaWindowSize, 1);
                obj.smaIndex  = 1;
            end

            % Initialize Gaussian (if selected)
            obj.gaussianWindowSize = p.Results.GaussianWindowSize;
            obj.gaussianStd        = p.Results.GaussianStd;
            if strcmp(obj.filterType, 'gaussian')
                obj.gaussianKernel = gausswin(obj.gaussianWindowSize)';
                obj.gaussianKernel = obj.gaussianKernel / sum(obj.gaussianKernel); % normalize
                obj.gaussianBuffer = zeros(obj.gaussianWindowSize, 1);
                obj.gaussianIndex  = 1;
            end

            % Skid-avoidance parameters
            obj.frictionCoeff = p.Results.FrictionCoeff;
            obj.gravity       = p.Results.Gravity;
            obj.safetyFactor  = p.Results.SafetyFactor;

            obj.jerkLimit = p.Results.JerkLimit;
            obj.curveSpeedReduction = p.Results.CurveSpeedReduction;

            % Speed smoothing factor and current target speed
            obj.speedSmoothing     = p.Results.SpeedSmoothing;
            obj.currentTargetSpeed = desiredSpeed;

            % Verbosity flag
            obj.verbose = logical(p.Results.Verbose);

            if obj.verbose
                fprintf('pid_SpeedController initialized with cornering safety.\n');
            end
        end

        %% computeAcceleration (Main)
        %  Now accepts an optional 'turnRadius' input. If you have a 
        %  real-time estimate of turn radius, pass it here. If not 
        %  used, you can keep it as `[]` or skip it in calls.
        function acceleration = computeAcceleration(obj, currentSpeed, currentTime, turnRadius, upcomingRadii)
            if nargin < 4 || isempty(turnRadius)
                % If turnRadius not provided, assume no cornering limit needed
                turnRadius = Inf;
            end
            if nargin < 5
                upcomingRadii = [];
            end

            % ---------------- 1) Adjust desired speed for cornering ----------------
            corneringSpeed = obj.computeCorneringSpeed(turnRadius);

            if ~isempty(upcomingRadii)
                waypointSpacing = 1.0; % meters between waypoints
                for idx = 1:numel(upcomingRadii)
                    R = upcomingRadii(idx);
                    distAhead = (idx-1) * waypointSpacing;
                    if isinf(R) || isnan(R)
                        continue;
                    end
                    limitSpeed = obj.computeCorneringSpeed(R);
                    stopDist = obj.computeStoppingDistance(currentSpeed, limitSpeed);
                    if stopDist >= distAhead
                        corneringSpeed = min(corneringSpeed, limitSpeed);
                    end
                end
            end

            % Compute smoothed target speed
            targetSpeed = min(obj.desiredSpeed, corneringSpeed);
            obj.currentTargetSpeed = obj.currentTargetSpeed + obj.speedSmoothing*(targetSpeed - obj.currentTargetSpeed);

            % ---------------- 2) Filter the current speed reading ------------------
            filteredSpeed = obj.applyFilter(currentSpeed);

            % ---------------- 3) Controller Activation -------------------------
            % Keep the controller active even when slowing down so that the PID
            % can generate negative acceleration commands for braking.
            if ~obj.controllerActive && obj.verbose
                fprintf('[pid_SpeedController] Re-enabling controller.\n');
            end
            obj.controllerActive = true;

            % Ensure filtered speed is not negative
            if filteredSpeed < 0
                filteredSpeed = 0;
            end

            if obj.currentTargetSpeed <= 0
                % No movement needed
                acceleration = 0;
                obj.controllerActive = false;
                if obj.verbose
                    fprintf('[pid_SpeedController] Desired speed <= 0. Controller disabled.\n');
                end
                return;
            end

            % ---------------- 4) PID control for Acceleration ----------------------
            error = obj.currentTargetSpeed - filteredSpeed;
            dt    = currentTime - obj.previousTime;
            if dt <= 0
                dt = 1e-6; % Prevent division by zero or negative dt
            end

            % Integral term update
            obj.integral = obj.integral + error * dt;

            % Derivative term
            derivative = (error - obj.previousError) / dt;

            % PID formula
            acceleration = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;

            % Bound acceleration
            acceleration = min(max(acceleration, obj.minAcceleration), obj.maxAcceleration);

            % Prevent negative speed
            if (filteredSpeed + acceleration * dt) < 0
                acceleration = -filteredSpeed / dt;
                acceleration = min(max(acceleration, obj.minAcceleration), obj.maxAcceleration);
            end

            % Prevent exceeding maxSpeed
            if (filteredSpeed + acceleration * dt) > obj.maxSpeed
                acceleration = (obj.maxSpeed - filteredSpeed) / dt;
                acceleration = min(max(acceleration, obj.minAcceleration), obj.maxAcceleration);
            end

            % Save for next iteration
            obj.previousError = error;
            obj.previousTime  = currentTime;

            % Allow negative acceleration so the braking system can act
            if acceleration < 0 && obj.verbose
                fprintf('[pid_SpeedController] Requesting deceleration of %.2f m/s^2.\n', acceleration);
            end
        end

        %% computeCorneringSpeed
        %  Simple utility to compute recommended cornering speed given a radius.
        function cornerSpeed = computeCorneringSpeed(obj, turnRadius)
            if isinf(turnRadius) || turnRadius <= 0
                % Straight line or invalid
                cornerSpeed = obj.maxSpeed;
            else
                cornerSpeed = sqrt(obj.frictionCoeff * obj.gravity * turnRadius) * obj.safetyFactor;
                % Also clamp it to maxSpeed then apply reduction factor
                cornerSpeed = min(cornerSpeed, obj.maxSpeed) * obj.curveSpeedReduction;
            end
        end

        %% computeStoppingDistance
        %  Computes stopping distance from v0 to v1 using jerk limited profile
        function dist = computeStoppingDistance(obj, v0, v1)
            if v0 <= v1
                dist = 0;
                return;
            end

            aMax = -obj.minAcceleration; % positive deceleration
            jMax = obj.jerkLimit;

            dv = v0 - v1;
            threshold = aMax^2 / jMax;

            if dv >= threshold
                t1 = aMax / jMax;
                t2 = (dv - threshold) / aMax;

                d1 = v0*t1 - jMax*t1^3/6;
                v1a = v0 - 0.5*jMax*t1^2;
                d2 = v1a*t2 - 0.5*aMax*t2^2;
                v2 = v1a - aMax*t2;
                d3 = v2*t1 - aMax*t1^2/2 + jMax*t1^3/6;
                dist = d1 + d2 + d3;
            else
                t1 = sqrt(dv/jMax);
                aPeak = jMax * t1;

                d1 = v0*t1 - jMax*t1^3/6;
                v1a = v0 - 0.5*jMax*t1^2;
                d3 = v1a*t1 - aPeak*t1^2/2 + jMax*t1^3/6;
                dist = d1 + d3;
            end
        end

        %% applyFilter
        function filteredSpeed = applyFilter(obj, currentSpeed)
            switch obj.filterType
                case 'none'
                    filteredSpeed = currentSpeed;
                case 'sma'
                    % Update SMA buffer using circular buffer
                    obj.smaBuffer(obj.smaIndex) = currentSpeed;
                    obj.smaIndex = obj.smaIndex + 1;
                    if obj.smaIndex > obj.smaWindowSize
                        obj.smaIndex = 1;
                    end
                    % Compute mean without shifting data
                    filteredSpeed = mean(obj.smaBuffer);
                case 'gaussian'
                    % Update Gaussian buffer using circular buffer
                    obj.gaussianBuffer(obj.gaussianIndex) = currentSpeed;
                    obj.gaussianIndex = obj.gaussianIndex + 1;
                    if obj.gaussianIndex > obj.gaussianWindowSize
                        obj.gaussianIndex = 1;
                    end
                    % Compute Gaussian filtered speed
                    filteredSpeed = sum(obj.gaussianKernel .* obj.gaussianBuffer);
                otherwise
                    error('Unknown filter type: %s', obj.filterType);
            end
        end

        %% reset
        function reset(obj)
            obj.integral        = 0;
            obj.previousError   = 0;
            obj.previousTime    = 0;
            obj.controllerActive = true;
            obj.currentTargetSpeed = obj.desiredSpeed;

            % Reset buffers if needed
            if strcmp(obj.filterType, 'sma')
                obj.smaBuffer = zeros(obj.smaWindowSize,1);
                obj.smaIndex  = 1;
            end
            if strcmp(obj.filterType, 'gaussian')
                obj.gaussianBuffer = zeros(obj.gaussianWindowSize,1);
                obj.gaussianIndex  = 1;
            end

            if obj.verbose
                fprintf('[pid_SpeedController] Reset and re-enabled.\n');
            end
        end
    end
end