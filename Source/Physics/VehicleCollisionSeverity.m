%{
% @file VehicleCollisionSeverity.m
% @brief Estimates crash severity using 3D collision dynamics.
% @author Miguel Marina
%}
%/**
% * @class VehicleCollisionSeverity
% * @brief Simulates partially elastic 3D collisions between two vehicles and estimates crash severities based on SAE J2980.
% *
% * The VehicleCollisionSeverity class models 3D collisions between two vehicles, accounting for linear dynamics.
% * It calculates crash severities based on collision parameters and SAE J2980 thresholds, evaluating collision
% * severity from both Vehicle 1 vs. Vehicle 2 and Vehicle 2 vs. Vehicle 1 perspectives.
% *
% * @author Miguel
% * @version 2.5
% * @date 2024-10-04
% */
classdef VehicleCollisionSeverity
    properties
        % Vehicle 1 Properties
        m1      % Mass of Vehicle 1 (kg)
        v1      % Velocity of Vehicle 1 before collision [vx; vy; vz] (m/s)
        r1      % Position vector from center of mass to collision point [x; y; z] (m)
        
        % Vehicle 2 Properties
        m2      % Mass of Vehicle 2 (kg)
        v2      % Velocity of Vehicle 2 before collision [vx; vy; vz] (m/s)
        r2      % Position vector from center of mass to collision point [x; y; z] (m)
        
        % Collision Parameters
        e       % Coefficient of restitution (0 <= e <= 1)
        n       % Collision normal unit vector [nx; ny; nz]
        
        % Severity Mapping Parameters
        J2980AssumedMaxMass = 0;
        VehicleUnderAnalysisMaxMass = 0;
        bound_type_str = 'Average'; % Default boundary type
        
        % Collision Types for Each Vehicle
        CollisionType_vehicle1 = ''; % Type of collision for Vehicle 1
        CollisionType_vehicle2 = ''; % Type of collision for Vehicle 2
        
        % Final Velocities After Collision
        v1_final    % Final velocity of Vehicle 1 after collision [vx; vy; vz] (m/s)
        v2_final    % Final velocity of Vehicle 2 after collision [vx; vy; vz] (m/s)
    end
    
    methods
        %% Constructor
        function obj = VehicleCollisionSeverity(m1, v1, r1, ...
                                                  m2, v2, r2, ...
                                                  e, n)
            % Constructor to initialize the VehicleCollisionSeverity object
            %
            % Inputs:
            %   m1, v1, r1 - Vehicle 1 parameters
            %   m2, v2, r2 - Vehicle 2 parameters
            %   e, n - Collision parameters

            if nargin ~= 8
                error('VehicleCollisionSeverity constructor requires eight input arguments.');
            end

            obj.m1 = m1;
            obj.v1 = v1;
            obj.r1 = r1;

            obj.m2 = m2;
            obj.v2 = v2;
            obj.r2 = r2;

            obj.e = e;
            obj.n = n;
            
            % Initialize CollisionType for both vehicles
            obj.CollisionType_vehicle1 = '';
            obj.CollisionType_vehicle2 = '';
            
            % Initialize final velocities to initial velocities
            obj.v1_final = v1;
            obj.v2_final = v2;
        end
        
        %% Perform Collision
        function obj = PerformCollision(obj)
            %/**
            % * @brief Performs the collision calculations to determine final velocities.
            % *
            % * This method calculates the final velocities of both vehicles post-collision based on
            % * the conservation of linear momentum and the coefficient of restitution.
            % *
            % * @return obj The updated VehicleCollisionSeverity object with final velocities.
            % */
            
            % Calculate relative velocity along the normal direction
            v_rel_normal = dot(obj.v2 - obj.v1, obj.n);
            
            % If relative velocity is positive, vehicles are moving apart; no collision
            % if v_rel_normal > 0
            %     fprintf('Vehicles are moving apart. No collision response applied.\n');
            %     obj.v1_final = obj.v1;
            %     obj.v2_final = obj.v2;
            %     return;
            % end
            
            % Calculate impulse scalar
            J = -(1 + obj.e) * v_rel_normal / (1/obj.m1 + 1/obj.m2);
            
            % Update velocities
            obj.v1_final = obj.v1 + (J / obj.m1) * obj.n;
            obj.v2_final = obj.v2 - (J / obj.m2) * obj.n;
        end
        
        %% Calculate Severity
        function [deltaV_vehicle1, deltaV_vehicle2, Vehicle1Severity, Vehicle2Severity] = CalculateSeverity(obj)
            %/**
            % * @brief Calculates the severity of the collision for each vehicle based on delta-V thresholds.
            % *
            % * This method computes the change in velocity (delta-V) for both vehicles resulting from the collision
            % * and determines their respective severity ratings based on predefined thresholds.
            % *
            % * @return deltaV_vehicle1 Change in velocity for Vehicle 1 (kph).
            % * @return deltaV_vehicle2 Change in velocity for Vehicle 2 (kph).
            % * @return Vehicle1Severity Severity rating for Vehicle 1 ('S0', 'S1', etc.).
            % * @return Vehicle2Severity Severity rating for Vehicle 2 ('S0', 'S1', etc.).
            % */
            
            % Calculate delta-V for both vehicles
            deltaV_vehicle1 = norm(obj.v1_final - obj.v1) * 3.6; % Convert from m/s to kph
            deltaV_vehicle2 = norm(obj.v2_final - obj.v2) * 3.6; % Convert from m/s to kph
            
            % Determine collision thresholds for Vehicle 1
            thresholds_vehicle1 = obj.get_thresholds(obj.bound_type_str, obj.CollisionType_vehicle1, obj.VehicleUnderAnalysisMaxMass, obj.J2980AssumedMaxMass);
            % Determine collision thresholds for Vehicle 2
            thresholds_vehicle2 = obj.get_thresholds(obj.bound_type_str, obj.CollisionType_vehicle2, obj.VehicleUnderAnalysisMaxMass, obj.J2980AssumedMaxMass);
            
            % Determine severity ratings
            Vehicle1Severity = obj.determine_severity(deltaV_vehicle1, thresholds_vehicle1);
            Vehicle2Severity = obj.determine_severity(deltaV_vehicle2, thresholds_vehicle2);
        end
    end
    
    methods (Access = private)
        %/**
        % * @brief Retrieves the severity thresholds based on boundary type, collision type, and vehicle mass.
        % *
        % * @param bound_type_str Boundary type string ('LowerBound', 'HigherBound', or 'Average').
        % * @param collision_type Type of collision ('Head-On Collision', 'Rear-End Collision', 'Side Collision', or 'Oblique Collision').
        % * @param vehicle_mass Mass of the vehicle (kg).
        % *
        % * @return thresholds Nx2 matrix of [min, max] delta-V values in kph.
        % *
        % * @throws Error if an invalid boundary type or collision type is provided.
        % */
        function thresholds = get_thresholds(~, bound_type_str, collision_type, vehicle_mass, J2980MaxAssumedMass)
            % Retrieve the severity thresholds based on bound type, collision type, and vehicle mass
            %
            % Parameters:
            %   bound_type_str - 'LowerBound', 'HigherBound', or 'Average'
            %   collision_type - 'Head-On Collision', 'Rear-End Collision', 'Side Collision', or 'Oblique Collision'
            %   vehicle_mass - Mass of the vehicle (kg)
            %
            % Returns:
            %   thresholds - Nx2 matrix of [min, max] delta-V values in kph

            % Define KE_J2980 thresholds per severity (in Joules)
            switch bound_type_str
                case 'LowerBound'
                    switch collision_type
                        case 'Head-On Collision'
                            KE_thresholds = [0, 4; 4, 20; 20, 40; 40, Inf];
                        case 'Rear-End Collision'
                            KE_thresholds = [0, 4; 4, 20; 20, 40; 40, Inf];
                        case 'Side Collision'
                            KE_thresholds = [0, 2; 2, 8; 8, 16; 16, Inf];
                        case 'Oblique Collision'
                            KE_thresholds = [0, 3; 3, 14; 14, 28; 28, Inf];
                        otherwise
                            error('Invalid collision type: %s.', collision_type);
                    end
                case 'HigherBound'
                    switch collision_type
                        case 'Head-On Collision'
                            KE_thresholds = [0, 10; 10, 50; 50, 65; 65, Inf];
                        case 'Rear-End Collision'
                            KE_thresholds = [0, 10; 10, 50; 50, 60; 60, Inf];
                        case 'Side Collision'
                            KE_thresholds = [0, 3; 10, 30; 30, 40; 40, Inf];
                        case 'Oblique Collision'
                            KE_thresholds = [0, 6.5; 10, 40; 40, 50; 50, Inf];
                        otherwise
                            error('Invalid collision type: %s.', collision_type);
                    end
                case 'Average'
                    switch collision_type
                        case 'Head-On Collision'
                            KE_thresholds = [0, 7; 7, 35; 35, 52.5; 52.5, Inf];
                        case 'Rear-End Collision'
                            KE_thresholds = [0, 7; 7, 35; 35, 52.5; 52.5, Inf];
                        case 'Side Collision'
                            KE_thresholds = [0, 2.5; 2.5, 6; 6, 39; 39, Inf];
                        case 'Oblique Collision'
                            KE_thresholds = [0, 4.75; 4.75, 20.5; 20.5, 47.5; 47.5, Inf];
                        otherwise
                            error('Invalid collision type: %s.', collision_type);
                    end
                otherwise
                    error('Invalid bound type: %s.', bound_type_str);
            end

            % Initialize thresholds matrix
            num_severities = size(KE_thresholds, 1);
            thresholds = zeros(num_severities, 2);

            for i = 1:num_severities
                % Calculate delta-V thresholds based on KE = 0.5 * m_vehicle * dv_vehicle^2
                KE_min = KE_thresholds(i, 1);
                KE_max = KE_thresholds(i, 2);
                KE_min = 0.5 * J2980MaxAssumedMass * (KE_min/3.6)^2; % in Joules
                KE_max = 0.5 * J2980MaxAssumedMass * (KE_max/3.6)^2; % in Joules

                % Therefore, dv_vehicle = sqrt(2 * KE / m_vehicle)
                dv_min_mps = sqrt(2 * KE_min / vehicle_mass); % in m/s
                dv_max_mps = sqrt(2 * KE_max / vehicle_mass); % in m/s

                % Convert delta-V from m/s to kph
                dv_min_kph = dv_min_mps * 3.6;
                dv_max_kph = dv_max_mps * 3.6;

                % Assign to thresholds matrix
                thresholds(i, :) = [dv_min_kph, dv_max_kph];
            end
        end

        %/**
        % * @brief Determines the severity rating based on delta-V and collision thresholds.
        % *
        % * Compares the delta-V value against the provided thresholds to assign a severity rating.
        % *
        % * @param delta_v Delta-V value in kph.
        % * @param collision_thresholds Nx2 matrix of [min, max] delta-V values in kph.
        % *
        % * @return severity Severity rating as 'S0', 'S1', etc.
        % */
        function severity = determine_severity(~, delta_v, collision_thresholds)
            % Determine severity rating based on delta-V and collision thresholds
            %
            % Parameters:
            %   delta_v - Delta-V in kph
            %   collision_thresholds - Thresholds for the collision type
            %
            % Returns:
            %   severity - Severity rating as 'S0', 'S1', etc.

            severity = 'S0'; % Default severity

            for i = 1:size(collision_thresholds, 1)
                min_val = collision_thresholds(i, 1);
                max_val = collision_thresholds(i, 2);
                if delta_v > min_val && delta_v <= max_val
                    severity = "S" + num2str(i - 1);
                    return;
                end
            end

            if delta_v > 0
                % If delta_v exceeds all thresholds, assign the highest severity
                severity = "S" + num2str(size(collision_thresholds, 1) - 1);
            end
        end
    end

    methods (Static, Access = public)
        %/**
        % * @brief Converts meters per second to kilometers per hour.
        % *
        % * @param vel Velocity in m/s.
        % *
        % * @return v_kph Velocity in km/h.
        % */
        function v_kph = ms_to_kph(vel)
            % Convert meters per second to kilometers per hour
            %
            % Parameters:
            %   vel - Velocity in m/s
            %
            % Returns:
            %   v_kph - Velocity in km/h
            v_kph = vel * 3.6;
        end

        %/**
        % * @brief Converts miles per hour to meters per second.
        % *
        % * @param vel Velocity in mph.
        % *
        % * @return v_ms Velocity in m/s.
        % */
        function v_ms = mph_to_ms(vel)
            % Convert miles per hour to meters per second
            %
            % Parameters:
            %   vel - Velocity in mph
            %
            % Returns:
            %   v_ms - Velocity in m/s
            v_ms = vel * 0.44704; % More accurate conversion
        end

        %/**
        % * @brief Calculates the coefficient of restitution based on impact velocity.
        % *
        % * Utilizes the Takeda correlation to determine the coefficient of restitution.
        % *
        % * @param impact_velocity Impact velocity in m/s.
        % *
        % * @return cor Coefficient of restitution (0 <= cor <= 1).
        % */
        function cor = get_cor_takeda(impact_velocity)
            % Calculate coefficient of restitution based on impact velocity
            %
            % Parameters:
            %   impact_velocity - Impact velocity in m/s
            %
            % Returns:
            %   cor - Coefficient of restitution (0 <= cor <= 1)
            cor = 0.574 * exp(-0.0396 * impact_velocity);
        end
    end
end
