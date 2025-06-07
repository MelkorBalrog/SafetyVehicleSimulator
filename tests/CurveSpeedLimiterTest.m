function tests = CurveSpeedLimiterTest
    tests = functiontests(localfunctions);
end

function testNoReductionWhenFar(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    dist = curSpeed * (limiter.rampDownTime + 1); % More than ramp time ahead
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
    dist = curSpeed * limiter.rampDownTime / 2;
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, dist, false, 0.1);
    expected = tgtSpeed * (limiter.reductionFactor + (1 - limiter.reductionFactor) * 0.5);
    verifyEqual(testCase, limited, expected, 'AbsTol', 1e-10);
end

function testRampUpAfterCurve(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20; tgtSpeed = 20;
    % Inside curve first
    limiter.limitSpeed(curSpeed, tgtSpeed, 0, true, 0.1);
    % Exit curve
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, Inf, false, 1.0);
    expectedFactor = limiter.reductionFactor + limiter.maxAccel * 1.0 / tgtSpeed;
    verifyEqual(testCase, limited, tgtSpeed * expectedFactor, 'AbsTol', 1e-10);
end
