function tests = ACCControllerTest
    tests = functiontests(localfunctions);
end

function setup(testCase)
    testCase.TestData.ctrl = acc_Controller(0.75, 2.0, 12.0, 3.0);
end

function testStartDecel(testCase)
    ctrl = testCase.TestData.ctrl;
    % Distance corresponds to less than 5.5 s lookahead at 20 m/s
    curSpeed = 20; pidAccel = 1; dist = 40; radius = 50; dt = 1;
    [accelOut, rot] = ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    verifyEqual(testCase, accelOut, -2, 'AbsTol', 1e-10);
    verifyGreaterThan(testCase, rot, 0);
end

function testNoDecelWhenFar(testCase)
    ctrl = testCase.TestData.ctrl;
    % Distance beyond 5.5 s lookahead should not trigger decel
    curSpeed = 20; pidAccel = 1; dist = 120; radius = 50; dt = 1;
    [accelOut, rot] = ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    verifyEqual(testCase, accelOut, pidAccel, 'AbsTol', 1e-10);
    verifyGreaterThan(testCase, rot, 0);
end

function testMaintainSpeedInCurve(testCase)
    ctrl = testCase.TestData.ctrl;
    % Trigger decel first
    curSpeed = 20; pidAccel = 1; dist = 40; radius = 50; dt = 1;
    ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    % Once speed reaches target maintain 75%
    curSpeed = 15; pidAccel = 2; dist = 0; radius = 20;
    [accelOut, ~] = ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    verifyEqual(testCase, accelOut, 0, 'AbsTol', 1e-10);
end

function testResumeAfterCurve(testCase)
    ctrl = testCase.TestData.ctrl;
    % Trigger decel first
    curSpeed = 20; pidAccel = 1; dist = 40; radius = 50; dt = 1;
    ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    % In curve maintain speed
    curSpeed = 15; pidAccel = 2; dist = 0; radius = 20;
    ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    % Exit curve and far from next one
    curSpeed = 15; pidAccel = 3; dist = 200; radius = Inf;
    [accelOut, ~] = ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    verifyEqual(testCase, accelOut, pidAccel, 'AbsTol', 1e-10);
end
