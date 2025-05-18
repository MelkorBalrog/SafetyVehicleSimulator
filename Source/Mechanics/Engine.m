classdef Engine < handle
    properties
        maxTorque
        torqueCurve
        maxRPM
        minRPM
        gearRatios
        differentialRatio
        efficiency
        currentRPM
        engineInertia    % Engine rotational inertia (kg·m²)
        clutch           % Clutch object
    end
    
    methods
        function obj = Engine(maxTorque, torqueCurve, maxRPM, minRPM, ...
                              gearRatios, differentialRatio, efficiency, ...
                              engineInertia, clutch)
            % Engine Constructor
            if nargin < 9
                error('Engine constructor requires all 9 parameters.');
            end
            obj.maxTorque = maxTorque;
            obj.torqueCurve = torqueCurve;
            obj.maxRPM = maxRPM;
            obj.minRPM = minRPM;
            obj.gearRatios = gearRatios;
            obj.differentialRatio = differentialRatio;
            obj.efficiency = efficiency;
            obj.currentRPM = minRPM;
            obj.engineInertia = engineInertia; % Initialize engine inertia
            obj.clutch = clutch; % Initialize clutch
        end
        
        function [engineTorque, wheelTorque] = getTorque(obj, throttlePosition, currentGear)
            % getTorque Retrieves engine and wheel torque based on current gear and throttle position
            % throttlePosition: 0 (no throttle) to 1 (full throttle)
            % currentGear: Current gear from Transmission
            
            % Ensure throttlePosition is within bounds
            throttlePosition = max(0, min(1, throttlePosition));
            
            % Calculate engine torque based on torque curve
            engineTorque = obj.torqueCurve(obj.currentRPM) * throttlePosition;
            
            % Limit engine torque to maxTorque
            engineTorque = min(engineTorque, obj.maxTorque);
            
            % If clutch is disengaged, no torque is transmitted to the wheels
            if ~obj.clutch.isEngaged
                wheelTorque = 0;
            else
                % Calculate wheel torque
                wheelTorque = engineTorque * obj.gearRatios(currentGear) * ...
                              obj.differentialRatio * obj.efficiency * ...
                              obj.clutch.getTorque();
            end
        end
        
        function rpm = getRPM(obj)
            % getRPM Retrieves the current RPM of the engine
            rpm = obj.currentRPM;
        end
        
        function obj = updateRPM(obj, throttlePosition, loadTorque, deltaTime, currentGear)
            % updateRPM Updates the engine RPM based on net torque and engine inertia
            % throttlePosition: Throttle input from 0 to 1
            % loadTorque: Torque opposing the engine's rotation (from drivetrain and load)
            % deltaTime: Time step for the simulation (seconds)
            % currentGear: Current gear from Transmission
            
            % Get engine torque from throttle position
            [engineTorque, ~] = obj.getTorque(throttlePosition, currentGear);
            
            % Calculate net torque
            netTorque = engineTorque - loadTorque;
            
            % Calculate angular acceleration (alpha = netTorque / inertia)
            angularAcceleration = netTorque / obj.engineInertia;
            
            % Update engine RPM
            % Convert angular acceleration (rad/s²) to RPM per second
            deltaRPM = angularAcceleration * deltaTime * (60 / (2 * pi));
            newRPM = obj.currentRPM + deltaRPM;
            
            % Clamp RPM within min and max
            obj.currentRPM = max(obj.minRPM, min(newRPM, obj.maxRPM));
        end
        
        % Additional methods to manually engage/disengage the clutch
        function engageClutch(obj, currentTime)
            obj.clutch.engage(currentTime);
        end
        
        function disengageClutch(obj, currentTime)
            obj.clutch.disengage(currentTime);
        end
    end
end
