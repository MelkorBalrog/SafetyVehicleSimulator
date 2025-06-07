function tests = ACCControllerTest
    tests = functiontests(localfunctions);
end

function setup(testCase)
    testCase.TestData.ctrl = acc_Controller(0.75, 2.0, 12.0, 3.0);
end

function testStartDecel(testCase)
    ctrl = testCase.TestData.ctrl;
    curSpeed = 20; pidAccel = 1; dist = 40; radius = 50; dt = 1;
    [accelOut, rot] = ctrl.adjust(curSpeed, pidAccel, dist, radius, dt);
    verifyEqual(testCase, accelOut, -2, 'AbsTol', 1e-10);
    verifyGreaterThan(testCase, rot, 0);
end

function testNoDecelWhenFar(testCase)
    ctrl = testCase.TestData.ctrl;
    curSpeed = 20; pidAccel = 1; dist = 60; radius = 50; dt = 1;
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
