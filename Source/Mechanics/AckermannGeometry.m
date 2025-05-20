%{
% @file AckermannGeometry.m
% @brief Calculates steering angles using Ackermann geometry.
% @author Miguel Marina
%}
%/**
% * @class AckermannGeometry
% * @brief Handles Ackermann steering geometry calculations for vehicle dynamics.
% *
% * The AckermannGeometry class provides methods to enforce mechanical limits on joint and
% * steering angles and to calculate the inner and outer steering angles based on the desired
% * steering angle, wheelbase, and track width. This ensures realistic and mechanically feasible
% * steering behavior in vehicle simulations.
% *
% * @author Miguel Marina
% * @version 1.0
% * @date 2024-10-04
% */
classdef AckermannGeometry
    properties (Constant)
        %/**
        % * @property jointLimits
        % * @brief Mechanical limits for joint angles (in radians).
        % *
        % * Structure containing the maximum and minimum allowable articulation angles between
        % * the tractor and trailer.
        % *
        % * Fields:
        % *   - maxAngle: Maximum articulation angle (45 degrees converted to radians).
        % *   - minAngle: Minimum articulation angle (-45 degrees converted to radians).
        % */
        jointLimits = struct(...
            'maxAngle', deg2rad(45), ...  % Maximum articulation angle between tractor and trailer in radians
            'minAngle', deg2rad(-45) ...  % Minimum articulation angle between tractor and trailer in radians
        );
        
        %/**
        % * @property steeringLimits
        % * @brief Mechanical limits for steering angles (in radians).
        % *
        % * Structure containing the maximum and minimum allowable steering angles for the front tires.
        % *
        % * Fields:
        % *   - maxSteerAngle: Maximum steering angle (30 degrees converted to radians).
        % *   - minSteerAngle: Minimum steering angle (-30 degrees converted to radians).
        % */
        steeringLimits = struct(...
            'maxSteerAngle', deg2rad(30), ...  % Maximum steering angle for front tires in radians
            'minSteerAngle', deg2rad(-30) ...  % Minimum steering angle for front tires in radians
        );
    end
    
    methods (Static)
        %/**
        % * @brief Enforces steering angle limits based on mechanical constraints.
        % *
        % * This method adjusts the input steering angle to ensure it stays within the predefined
        % * mechanical limits. If the input angle exceeds the maximum or minimum limits, it is
        % * clamped to the respective limit.
        % *
        % * @param steerAngle Desired steering angle (radians).
        % *
        % * @return adjustedSteerAngle The steering angle adjusted to lie within the allowed limits (radians).
        % */
        function adjustedSteerAngle = enforceSteeringLimits(steerAngle)
            if steerAngle > AckermannGeometry.steeringLimits.maxSteerAngle
                adjustedSteerAngle = AckermannGeometry.steeringLimits.maxSteerAngle;
            elseif steerAngle < AckermannGeometry.steeringLimits.minSteerAngle
                adjustedSteerAngle = AckermannGeometry.steeringLimits.minSteerAngle;
            else
                adjustedSteerAngle = steerAngle;
            end
        end
        
        %/**
        % * @brief Enforces joint angle limits based on mechanical constraints.
        % *
        % * This method adjusts the input joint angle to ensure it stays within the predefined
        % * mechanical limits. If the input angle exceeds the maximum or minimum limits, it is
        % * clamped to the respective limit.
        % *
        % * @param jointAngle Desired joint articulation angle (radians).
        % *
        % * @return adjustedJointAngle The joint angle adjusted to lie within the allowed limits (radians).
        % */
        function adjustedJointAngle = enforceJointLimits(jointAngle)
            if jointAngle > AckermannGeometry.jointLimits.maxAngle
                adjustedJointAngle = AckermannGeometry.jointLimits.maxAngle;
            elseif jointAngle < AckermannGeometry.jointLimits.minAngle
                adjustedJointAngle = AckermannGeometry.jointLimits.minAngle;
            else
                adjustedJointAngle = jointAngle;
            end
        end
        
        %/**
        % * @brief Calculates the inner and outer steering angles based on Ackermann geometry.
        % *
        % * Given a desired steering angle, wheelbase, and track width, this method computes the
        % * corresponding inner and outer steering angles for the front tires, as well as the turning
        % * radius.
        % *
        % * @param steerAngleRad Desired steering angle (radians).
        % * @param wheelbase Distance between front and rear wheels (meters).
        % * @param trackWidth Distance between left and right wheels (meters).
        % *
        % * @return innerSteerAngle The steering angle for the inner front tire (radians).
        % * @return outerSteerAngle The steering angle for the outer front tire (radians).
        % * @return R The turning radius of the vehicle (meters). Returns Inf if steerAngleRad is zero.
        % *
        % * @warning If steerAngleRad is zero, both inner and outer steering angles are set to zero,
        % * and the turning radius is set to infinity.
        % */
        function [innerSteerAngle, outerSteerAngle, R] = calculateAckermannSteeringAngles(steerAngleRad, wheelbase, trackWidth)
            if steerAngleRad == 0
                innerSteerAngle = 0;
                outerSteerAngle = 0;
                R = Inf;
            else
                R = wheelbase / tan(steerAngleRad);
                innerSteerAngle = atan(wheelbase / (R - trackWidth / 2));
                outerSteerAngle = atan(wheelbase / (R + trackWidth / 2));
            end
        end
    end
end
