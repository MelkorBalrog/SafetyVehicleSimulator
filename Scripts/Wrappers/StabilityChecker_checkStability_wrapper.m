function [isWiggling,isRollover,isSkidding,isJackknife] = StabilityChecker_checkStability_wrapper()
    % StabilityChecker_checkStability_wrapper - wrapper for StabilityChecker.checkStability
    % Initializes dummy vehicle objects and returns the stability flags

    persistent checker
    if isempty(checker)
        % Minimal initialization of dependencies
        vehicleType = 'tractor';
        mass = 10000; friction = 0.8; velocityVec = [0;0;0];
        dragCoeff = 0.3; airDensity = 1.225; frontalArea = 10; sideArea = 20; sideForceCoeff = 1.0;
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
        kin = KinematicsCalculator(fc, 1.0, 2.0, 200000, 5000, 5000, 150000, 4000, 8000, dt);
        transmission = Transmission();
        initialState.position = [0;0];
        initialState.orientation = 0;
        initialState.velocity = 0;
        initialState.lateralVelocity = 0;
        initialState.yawRate = 0;
        initialState.rollAngle = 0;
        initialState.rollRate = 0;
        dyn = DynamicsUpdater(fc, kin, initialState, mass, wheelbase, h_CoG, trackWidth, dt, vehicleType, trackWidth, 5000, 500, transmission, 0.1);
        checker = StabilityChecker(dyn, fc, kin, 0, 0, friction, h_CoG);
    end
    checker = checker.checkStability();
    [isWiggling,isRollover,isSkidding,isJackknife] = checker.getStabilityFlags();
end

