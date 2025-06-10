function [F_y_total, M_z] = ForceCalculator_computeTireForces_wrapper(loads, contactAreas, u, v, r)
% ForceCalculator_computeTireForces_wrapper - wrapper to call ForceCalculator.computeTireForces for MEX
% Usage:
%   [F_y_total, M_z] = ForceCalculator_computeTireForces_wrapper(loads, contactAreas, u, v, r)
% Inputs:
%   loads         - Nx1 vector of tire normal loads
%   contactAreas  - Nx1 vector of contact patch areas
%   u, v, r       - scalar longitudinal, lateral speed and yaw rate

    persistent fc
    if isempty(fc)
        % Basic initialization of ForceCalculator with fixed-size data for codegen
        vehicleType = 'tractor';
        mass = 10000; friction = 0.8; velocityVec = [0;0;0];
        dragCoeff = 0.3; airDensity = 1.225;
        frontalArea = 10; sideArea = 20; sideForceCoeff = 1.0;
        turnRadius = Inf; loadDist = [0,0,0,mass*9.81,1]; cog = [0;0;0];
        h_CoG = 1.0; angularVel = [0;0;0]; slopeAngle = 0;
        trackWidth = 2.0; wheelbase = 3.5;
        tireModel = struct('B',10,'C',1.9,'D',1.0,'E',0.97);
        suspensionModel = [];
        trailerInertia = 0; dt = 0.01; trailerMass = 0; trailerWheelbase = 0;
        numTrailerTires = 0; trailerBoxMasses = [];
        tireModelFlag = 'simple'; highFidelityModel = [];
        windVector = [0;0;0]; brakeSystem = BrakeSystem();
        varargin = {};
        fc = ForceCalculator(vehicleType, mass, friction, velocityVec, ...
            dragCoeff, airDensity, frontalArea, sideArea, sideForceCoeff, ...
            turnRadius, loadDist, cog, h_CoG, angularVel, slopeAngle, ...
            trackWidth, wheelbase, tireModel, suspensionModel, trailerInertia, ...
            dt, trailerMass, trailerWheelbase, numTrailerTires, trailerBoxMasses, ...
            tireModelFlag, highFidelityModel, windVector, brakeSystem, varargin{:});
    end
    idx = 1:numel(loads);
    [F_y_total, M_z] = fc.computeTireForces(loads, contactAreas, u, v, r, idx);
end
