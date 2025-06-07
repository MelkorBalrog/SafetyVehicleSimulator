function collision = CollisionDetector_mex_wrapper(corners1, corners2)
% CollisionDetector_mex_wrapper - wrapper function for CollisionDetector.checkCollision for MEX generation
% Usage:
%   collision = CollisionDetector_mex_wrapper(corners1, corners2)
% Inputs:
%   corners1: 4x2 double matrix
%   corners2: 4x2 double matrix

    % Create detector instance
    detector = CollisionDetector();
    % Call method
    collision = detector.checkCollision(corners1, corners2);
end
