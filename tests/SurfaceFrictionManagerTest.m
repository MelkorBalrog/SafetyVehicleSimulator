function tests = SurfaceFrictionManagerTest
    tests = functiontests(localfunctions);
end

function testMuForPositions(testCase)
    map = LaneMap(10,10);
    map = map.setCellValue(5,5,1);
    loc = VehicleLocalizer([0 0],1.0);
    mgr = SurfaceFrictionManager(map, loc, 0.8, 0.4);
    mu = mgr.getMuForTirePositions([5 5; 1 1]);
    verifyEqual(testCase, mu, [0.8;0.4], 'AbsTol',1e-10);
end
