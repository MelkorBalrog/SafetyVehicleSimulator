function tests = TractionLimitTest
    tests = functiontests(localfunctions);
end

function testTractionClamped(testCase)
    loadDist = [1 1 0 5000 0.1; -1 -1 0 5000 0.1];
    tireModel = PacejkaMagicFormula(10,1.9,8000,0.97);
    susp = LeafSpringSuspension(1000,100,0.3,1000,2,3);
    brake = BrakeSystem();
    fc = ForceCalculator('tractor', 1000, 0.8, [0;0;0], 0.3, 1.2, 1, 1, 0.8, Inf, ...
        loadDist, [0;0;0], 1, [0;0;0], 0, 2, 3, tireModel, susp, 0, 0.01, 0, 0, 0, [], [], 'simple', [], [0;0;0], brake);
    fc = fc.updateTractionForce(20000); % exceed mu*load -> should clamp
    forces = fc.getCalculatedForces();
    verifyLessThanOrEqual(testCase, forces.traction(1), 8000 + 1e-10);
end
