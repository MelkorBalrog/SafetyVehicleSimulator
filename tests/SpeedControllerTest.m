%{
% @file SpeedControllerTest.m
% @brief Unit tests for the SpeedController class.
% @author Miguel Marina
%}

%% SpeedControllerTest
% Entry point that returns the function-based tests structure.
%
% @retval tests  Structure containing local test functions.
function tests = SpeedControllerTest
    tests = functiontests(localfunctions);
end

%% setup
% Creates a SpeedController instance and stores test data.
%
% @param testCase  matlab.unittest.TestCase object for test context.
function setup(testCase)
    desiredSpeed = 20;      % m/s
    maxSpeed = 30;          % m/s
    Kp = 1; Ki = 0; Kd = 0;
    maxAccel = 5; minAccel = -5;
    mu = 0.8;
    g = 9.81;
    sf = 0.9;

    ctrl = SpeedController(desiredSpeed, maxSpeed, Kp, Ki, Kd, ...
        maxAccel, minAccel, 'FrictionCoeff', mu, 'Gravity', g, ...
        'SafetyFactor', sf);
    testCase.TestData.ctrl = ctrl;
    testCase.TestData.mu = mu;
    testCase.TestData.g = g;
    testCase.TestData.sf = sf;
    testCase.TestData.maxSpeed = maxSpeed;
end

%% testInfAndNegativeRadiusReturnsMax
% Ensures non-positive turn radius returns the max speed.
%
% @param testCase  matlab.unittest.TestCase object.
function testInfAndNegativeRadiusReturnsMax(testCase)
    ctrl = testCase.TestData.ctrl;
    ms = testCase.TestData.maxSpeed;
    verifyEqual(testCase, ctrl.computeCorneringSpeed(Inf), ms);
    verifyEqual(testCase, ctrl.computeCorneringSpeed(-5), ms);
end

%% testPositiveRadiusFormula
% Validates cornering speed formula for a positive radius.
%
% @param testCase  matlab.unittest.TestCase object.
function testPositiveRadiusFormula(testCase)
    ctrl = testCase.TestData.ctrl;
    mu = testCase.TestData.mu;
    g = testCase.TestData.g;
    sf = testCase.TestData.sf;
    ms = testCase.TestData.maxSpeed;

    R = 12;  % meters
    expected = min(sqrt(mu*g*R)*sf, ms);
    actual = ctrl.computeCorneringSpeed(R);
    verifyEqual(testCase, actual, expected, 'AbsTol', 1e-10);

    % also test a large radius that would exceed maxSpeed without clamping
    largeR = (ms/(sf*sqrt(mu*g)))^2 * 1.5;
    expectedLarge = min(sqrt(mu*g*largeR)*sf, ms);
    actualLarge = ctrl.computeCorneringSpeed(largeR);
    verifyEqual(testCase, actualLarge, expectedLarge, 'AbsTol', 1e-10);
end

