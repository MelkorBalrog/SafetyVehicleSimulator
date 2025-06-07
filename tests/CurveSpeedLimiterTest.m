function tests = CurveSpeedLimiterTest
    tests = functiontests(localfunctions);
end

function testNoLimitForLargeRadius(testCase)
    limiter = curveSpeed_Limiter(0.5);
    speed = 20;
    limited = limiter.limitSpeed(speed, 150); % radius larger than threshold
    verifyEqual(testCase, limited, speed);
end

function testLimitForSmallRadius(testCase)
    limiter = curveSpeed_Limiter(0.5);
    speed = 20;
    limited = limiter.limitSpeed(speed, 50); % radius below threshold
    verifyEqual(testCase, limited, speed * 0.5);
end
