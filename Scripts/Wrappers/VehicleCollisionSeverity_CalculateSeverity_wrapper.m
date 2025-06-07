function [dV1,dV2,severity1,severity2] = VehicleCollisionSeverity_CalculateSeverity_wrapper()
    % VehicleCollisionSeverity_CalculateSeverity_wrapper - wrapper to call VehicleCollisionSeverity.CalculateSeverity for MEX generation

    % Simple placeholder initialization
    m1 = 1000; v1 = [10;0;0]; r1 = [0;0;0];
    m2 = 1500; v2 = [-10;0;0]; r2 = [0;0;0];
    e = 0.3; n = [1;0;0];
    vcs = VehicleCollisionSeverity(m1, v1, r1, m2, v2, r2, e, n);
    vcs.CollisionType_vehicle1 = 'Head-On Collision';
    vcs.CollisionType_vehicle2 = 'Head-On Collision';
    vcs.J2980AssumedMaxMass = 3000;
    vcs.VehicleUnderAnalysisMaxMass = 3000;
    vcs = vcs.PerformCollision();
    [dV1,dV2,severity1,severity2] = vcs.CalculateSeverity();
end

