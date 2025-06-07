function tests = CurveSpeedLimiterTest
    tests = functiontests(localfunctions);
end

function testNoReductionWhenFar(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    [stopDist, ~] = limiter.stoppingDistance(tgtSpeed);
    dist = stopDist + 10; % Well beyond required distance
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, dist, false, 0.1);
    verifyEqual(testCase, limited, tgtSpeed, 'AbsTol', 1e-10);
end

function testFullReductionAtCurveStart(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, 0, true, 0.1);
    verifyEqual(testCase, limited, tgtSpeed * limiter.reductionFactor, 'AbsTol', 1e-10);
end

function testHalfwayReduction(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    [~, rampTime] = limiter.stoppingDistance(tgtSpeed);
    dist = curSpeed * rampTime / 2;
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, dist, false, 0.1);
    % Expected based on piecewise deceleration profile
    deltaV = 1 * 0.5 + 4.5 * (1.0 - 0.75);
    expectedFactor = 1 - deltaV / tgtSpeed;
    expectedFactor = max(limiter.reductionFactor, min(1, expectedFactor));
    expected = tgtSpeed * expectedFactor;
    verifyEqual(testCase, limited, expected, 'AbsTol', 1e-10);
end

function testRampUpAfterCurve(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20; tgtSpeed = 20;
    % Inside curve first
    limiter.limitSpeed(curSpeed, tgtSpeed, 0, true, 0.1);
    % Exit curve
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, Inf, false, 1.0);
    expectedFactor = min(1, limiter.reductionFactor + limiter.rampUpAccel * 1.0 / tgtSpeed);
    verifyEqual(testCase, limited, tgtSpeed * expectedFactor, 'AbsTol', 1e-10);
end

function testStoppingDistanceComputation(testCase)
    limiter = curveSpeed_Limiter();
    speed = 20;
    [dist, time] = limiter.stoppingDistance(speed);
    expTime = 1.5;
    expDist = 9.875 + 17.25;
    verifyEqual(testCase, time, expTime, 'AbsTol', 1e-10);
    verifyEqual(testCase, dist, expDist, 'AbsTol', 1e-10);
end
