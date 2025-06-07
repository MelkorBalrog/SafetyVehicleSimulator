function tests = CurveSpeedLimiterTest
    tests = functiontests(localfunctions);
end

function testNoRampBeforeStoppingDistance(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    stopDist = limiter.computeStoppingDistance(curSpeed, tgtSpeed * limiter.reductionFactor);
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, stopDist + 1);
    verifyEqual(testCase, limited, tgtSpeed, 'AbsTol', 1e-10);
end

function testFullReductionAtCurveStart(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, 0);
    verifyEqual(testCase, limited, tgtSpeed * limiter.reductionFactor, 'AbsTol', 1e-10);
end

function testHalfwayReduction(testCase)
    limiter = curveSpeed_Limiter();
    curSpeed = 20;
    tgtSpeed = 20;
    stopDist = limiter.computeStoppingDistance(curSpeed, tgtSpeed * limiter.reductionFactor);
    limited = limiter.limitSpeed(curSpeed, tgtSpeed, stopDist/2);
    expected = tgtSpeed * (limiter.reductionFactor + (1 - limiter.reductionFactor) * 0.5);
    verifyEqual(testCase, limited, expected, 'AbsTol', 1e-10);
end
