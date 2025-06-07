% generate_mex.m
% Script to generate MEX functions for performance-critical routines via MATLAB Coder

% Ensure MATLAB Coder is installed and paths include this folder and Source/Physics

fprintf('Generating MEX for CollisionDetector.checkCollision...\n');
codegen -config:mex CollisionDetector_mex_wrapper -args {zeros(4,2,'double'), zeros(4,2,'double')} -report;

% Add other wrappers as needed for additional hotspots
% Generate MEX for ForceCalculator.computeTireForces
fprintf('Generating MEX for ForceCalculator.computeTireForces...\n');
codegen -config:mex ForceCalculator_computeTireForces_wrapper -args {zeros(4,1,'double'), zeros(4,1,'double'), 0, 0, 0} -report;

% Generate MEX for DynamicsUpdater.stateDerivative
fprintf('Generating MEX for DynamicsUpdater.stateDerivative...\n');
codegen -config:mex DynamicsUpdater_stateDerivative_wrapper -args {zeros(8,1,'double')} -report;
