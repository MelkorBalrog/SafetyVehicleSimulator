function [dydt, accelerations] = DynamicsUpdater_stateDerivative_wrapper(stateVec)
% DynamicsUpdater_stateDerivative_wrapper - wrapper to call DynamicsUpdater.stateDerivative for MEX
% Usage:
%   [dydt, accelerations] = DynamicsUpdater_stateDerivative_wrapper(stateVec)
% Inputs:
%   stateVec - vector [x; y; theta; p_x; p_y; L_z; phi; p]

    persistent dyn
    if isempty(dyn)
        % TODO: Initialize DynamicsUpdater with actual instances and initial state
        % Placeholder initialState struct
        initialState.position = [0;0];
        initialState.orientation = 0;
        initialState.velocity = 0;
        initialState.lateralVelocity = 0;
        initialState.yawRate = 0;
        initialState.rollAngle = 0;
        initialState.rollRate = 0;
        % Placeholder class instances and parameters
        fc = ForceCalculator('tractor', 10000, 0.8, [0;0;0], 0.3, 1.225, 10, 20, 1.0, Inf, [0,0,0,10000*9.81,1], [0;0;0], 1.0, [0;0;0], 0, 2.0, 3.5, struct(), [], 0, 0.01, 0, 0, 0, 'simple', [], [0;0;0], []);
        kin = KinematicsCalculator(fc, 1.0, 2.0, 200000, 5000, 5000, 150000, 4000, 8000, 0.01);
        transmission = Transmission();
        dyn = DynamicsUpdater(fc, kin, initialState, 10000, 3.5, 1.0, 2.0, 0.01, 'tractor', 2.0, 5000, 500, transmission, 0.1);
    end
    [dydt, accelerations] = dyn.stateDerivative(stateVec);
end