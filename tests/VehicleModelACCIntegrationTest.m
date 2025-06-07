function tests = VehicleModelACCIntegrationTest
    tests = functiontests(localfunctions);
end

function testACCIntegration(testCase)
    vm = VehicleModel([], [], false, 'sim', []);
    vm.initializeDefaultParameters();
    vm.initializeSim();
    verifyTrue(testCase, isprop(vm, 'accController'));
    verifyNotEmpty(testCase, vm.accController);
end
