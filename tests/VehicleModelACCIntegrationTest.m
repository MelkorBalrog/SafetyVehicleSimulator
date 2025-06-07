function tests = VehicleModelACCIntegrationTest
    tests = functiontests(localfunctions);
end

function testACCIntegration(testCase)
    vm = VehicleModel([], [], false, 'sim', []);
    vm.initializeDefaultParameters();
    vm.initializeSim();
    verifyTrue(testCase, isprop(vm, 'accController'));
    verifyNotEmpty(testCase, vm.accController);
    verifyTrue(testCase, isprop(vm, 'localizer'));
    verifyNotEmpty(testCase, vm.localizer);
end
