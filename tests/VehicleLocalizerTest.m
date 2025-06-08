function tests = VehicleLocalizerTest
    tests = functiontests(localfunctions);
end

function testLocalization(testCase)
    wps = [0 0; 1 0; 2 0; 3 0];
    loc = VehicleLocalizer(wps, 1.0);
    idx = loc.localize([2.2 0]);
    verifyEqual(testCase, idx, 3);
end

function testDistanceToCurve(testCase)
    wps = [0 0; 1 0; 2 0; 3 0];
    loc = VehicleLocalizer(wps, 1.0);
    upcoming = [Inf Inf 10];
    dist = loc.distanceToNextCurve(1, upcoming);
    verifyEqual(testCase, dist, 2, 'AbsTol', 1e-10);
end
