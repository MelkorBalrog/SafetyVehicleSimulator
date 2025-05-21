%/**
% * @class SimManager
% * @brief Handles running simulations, detecting collisions, and plotting results for Vehicles in parallel.
% *
% * This revised version launches collision detection in parallel to animating the trajectories.
% *
% * @version 2.6
% * @date 2024-10-31
% */
classdef SimManager < handle
    properties
        map
        % Simulation Components
        dataManager
        plotManager
        collisionDetector

        % Vehicle Simulations and Configurations
        vehicleSim1
        vehicleSim2
        vehicleSimConfig1
        vehicleSimConfig2

        % Managers
        uiManager
        configManager

        % Simulation Parameters
        dt

        % Collision point coordinates
        collisionX
        collisionY

        % Storage for simulation results
        sim1Results
        sim2Results
    end

    methods
        function obj = SimManager(map, dataManager, plotManager, collisionDetector, ...
                                   vehicleSimConfig1, vehicleSimConfig2, ...
                                   vehicleSim1, vehicleSim2, ...
                                   uiManager, configManager, dt)
            obj.map = map;
            obj.dataManager = dataManager;
            obj.plotManager = plotManager;
            obj.collisionDetector = collisionDetector;
            obj.vehicleSimConfig1 = vehicleSimConfig1;
            obj.vehicleSimConfig2 = vehicleSimConfig2;
            obj.vehicleSim1 = vehicleSim1;
            obj.vehicleSim2 = vehicleSim2;
            obj.uiManager = uiManager;
            obj.configManager = configManager;
            obj.dt = dt;

            % Initialize collision point coordinates
            obj.collisionX = NaN;
            obj.collisionY = NaN;
        end

        %/**
        % * @brief Executes the simulation workflow, launches collision checking in parallel,
        % *        and animates the vehicle trajectories.
        % */
        function runSimulations(obj)
            try
                %% 1. Clear Previous Plots
                disp('Clearing previous plots...');
                obj.plotManager.clearPlots();

                %% 2. Update Simulation Parameters for Both Vehicles
                disp('Updating simulation parameters for Vehicle 1...');
                obj.vehicleSim1.simParams = obj.vehicleSimConfig1.getSimulationParameters();
                disp('Updating simulation parameters for Vehicle 2...');
                obj.vehicleSim2.simParams = obj.vehicleSimConfig2.getSimulationParameters();

                % Set vehicleType fields dynamically
                obj.vehicleSim1.simParams.vehicleType = 'Truck';
                obj.vehicleSim2.simParams.vehicleType = 'Tractor';

                %% 3. Retrieve Offsets and Rotation Settings
                disp('Retrieving offsets and rotation settings...');
                offsetX = obj.uiManager.getOffsetX();
                offsetY = obj.uiManager.getOffsetY();
                vehicle1OffsetX = obj.uiManager.getvehicle1OffsetX();
                vehicle1OffsetY = obj.uiManager.getvehicle1OffsetY();
                rotateVehicle2 = obj.uiManager.getRotateVehicle2();
                rotateVehicle1 = obj.uiManager.getRotateVehicle1();
                rotationAngleDeg = obj.uiManager.getRotationAngleVehicle2(); % Degrees
                vehicle1rotationAngleDeg = obj.uiManager.getRotationAngleVehicle1(); % Degrees
                rotationAngleRad = deg2rad(-rotationAngleDeg); % Radians for Vehicle 2
                vehicle1rotationAngleRad = deg2rad(-vehicle1rotationAngleDeg); % Radians for Vehicle 1

                %% 4. Validate Offsets
                disp('Validating offsets...');
                if isempty(offsetX) || isempty(offsetY) || ~isnumeric(offsetX) || ~isnumeric(offsetY)
                    uialert(obj.uiManager.fig, 'Please enter valid numeric offset values for the Tractor.', 'Input Error');
                    return;
                end

                %% 5. Retrieve Wheel Dimensions
                disp('Retrieving wheel dimensions...');
                wheelHeightTruck1 = obj.vehicleSim1.simParams.tractorTireHeight;
                wheelWidthTruck1  = obj.vehicleSim1.simParams.tractorTireWidth;

                if obj.vehicleSim1.simParams.includeTrailer
                    wheelHeightTrailer1 = obj.vehicleSim1.simParams.trailerTireHeight;
                    wheelWidthTrailer1  = obj.vehicleSim1.simParams.trailerTireWidth;
                else
                    wheelHeightTrailer1 = [];
                    wheelWidthTrailer1  = [];
                end

                wheelHeightTractor2 = obj.vehicleSim2.simParams.tractorTireHeight;
                wheelWidthTractor2  = obj.vehicleSim2.simParams.tractorTireWidth;

                if obj.vehicleSim2.simParams.includeTrailer
                    wheelHeightTrailer2 = obj.vehicleSim2.simParams.trailerTireHeight;
                    wheelWidthTrailer2  = obj.vehicleSim2.simParams.trailerTireWidth;
                else
                    wheelHeightTrailer2 = [];
                    wheelWidthTrailer2  = [];
                end

                %% 6. Validate Wheel Dimensions
                disp('Validating wheel dimensions...');
                if isempty(wheelHeightTruck1) || wheelHeightTruck1 <= 0 || ...
                   isempty(wheelWidthTruck1)  || wheelWidthTruck1 <= 0
                    uialert(obj.uiManager.fig, 'Please ensure Tractor Wheel Height and Width are positive values in simulation parameters for Vehicle 1.', 'Input Error');
                    return;
                end

                if ~isempty(wheelHeightTrailer1) && (~isnumeric(wheelHeightTrailer1) || wheelHeightTrailer1 <= 0 || ...
                   ~isnumeric(wheelWidthTrailer1)  || wheelWidthTrailer1 <= 0)
                    uialert(obj.uiManager.fig, 'Please ensure Trailer Wheel Height and Width are positive values in simulation parameters for Vehicle 1.', 'Input Error');
                    return;
                end

                if isempty(wheelHeightTractor2) || wheelHeightTractor2 <= 0 || ...
                   isempty(wheelWidthTractor2)  || wheelWidthTractor2 <= 0
                    uialert(obj.uiManager.fig, 'Please ensure Tractor Wheel Height and Width are positive values in simulation parameters for Vehicle 2.', 'Input Error');
                    return;
                end

                if ~isempty(wheelHeightTrailer2) && (~isnumeric(wheelHeightTrailer2) || wheelHeightTrailer2 <= 0 || ...
                   ~isnumeric(wheelWidthTrailer2)  || wheelWidthTrailer2 <= 0)
                    uialert(obj.uiManager.fig, 'Please ensure Trailer Wheel Height and Width are positive values in simulation parameters for Vehicle 2.', 'Input Error');
                    return;
                end

                %% 7. Initialize the Progress Bar
                disp('Initializing progress bar...');
                hWaitbar = waitbar(0.15, 'Starting Simulations...', 'Name', 'Simulation Progress');
                cleanupObj = onCleanup(@() closeIfOpen(hWaitbar));

                %% 8. Instantiate Vehicle and Trailer Parameters for Plotting (for real-time animation)
                disp('Instantiating vehicle parameters for plotting...');
                vehicleParams1 = obj.createVehicleParams(obj.vehicleSim1.simParams, wheelHeightTruck1, wheelWidthTruck1);
                if obj.vehicleSim1.simParams.includeTrailer
                    trailerParams1 = obj.createTrailerParams(obj.vehicleSim1.simParams, wheelHeightTrailer1, wheelWidthTrailer1, 1);
                else
                    trailerParams1 = [];
                end
                vehicleParams2 = obj.createVehicleParams(obj.vehicleSim2.simParams, wheelHeightTractor2, wheelWidthTractor2);
                if obj.vehicleSim2.simParams.includeTrailer
                    trailerParams2 = obj.createTrailerParams(obj.vehicleSim2.simParams, wheelHeightTrailer2, wheelWidthTrailer2, 2);
                else
                    trailerParams2 = [];
                end

                %% Retrieve and validate mass values for collision detection
                disp('Retrieving and validating mass values for real-time animation...');
                VehicleUnderAnalysisMaxMass = obj.uiManager.getVehicle1Mass();
                J2980AssumedMaxMass        = obj.uiManager.getVehicle2Mass();
                if ~isnumeric(VehicleUnderAnalysisMaxMass) || VehicleUnderAnalysisMaxMass <= 0
                    uialert(obj.uiManager.fig, 'Invalid Tractor Vehicle Mass entered. Please enter a positive numeric value.', 'Input Error');
                    return;
                end
                if ~isnumeric(J2980AssumedMaxMass) || J2980AssumedMaxMass <= 0
                    uialert(obj.uiManager.fig, 'Invalid Vehicle Under Analysis Mass entered. Please enter a positive numeric value.', 'Input Error');
                    return;
                end

                %% Determine total steps for real-time animation
                totalSteps = length(obj.vehicleSim1.simParams.steeringCommands);
                fprintf('Total Simulation Steps set to: %d\n', totalSteps);
                
                %% 9. Run Vehicle Simulations
                Debug = 1;
                if Debug == 0
                    disp('Running simulations asynchronously...');
                    futures = cell(1, 2);

                    futures{1} = parfeval(@SimManager.runVehicleSim1, 1, obj.vehicleSim1);
                    futures{2} = parfeval(@SimManager.runVehicleSim2, 1, obj.vehicleSim2);

                    simResultsArray = cell(1, 2);
                    for idx = 1:2
                        future = futures{idx};
                        disp(['Fetching outputs for simulation ', num2str(idx), '...']);
                        result = fetchOutputs(future);
                        simResultsArray{idx} = result;

                        progress = 0.15 + (0.85 / 2) * (idx / 2);
                        waitbar(progress, hWaitbar, sprintf('Simulation Progress: %.2f%%', progress * 100));
                    end

                    obj.sim1Results = simResultsArray{1};
                    obj.sim2Results = simResultsArray{2};
                else
                    disp('Running simulations synchronously...');
                    obj.sim1Results = SimManager.runVehicleSim1(obj.vehicleSim1);
                    disp('VehicleSim1 simulation completed.');
                    obj.sim2Results = SimManager.runVehicleSim2(obj.vehicleSim2);
                    disp('VehicleSim2 simulation completed.');
                end

                %% 9. Transfer Simulation Results
                disp('Transferring simulation results to DataManager...');
                obj.transferSimulationResults();

                %% 10. Equalize Simulation Data Lengths
                disp('Equalizing simulation data lengths...');
                sim1Length = length(obj.dataManager.globalVehicle1Data.X);
                sim2Length = length(obj.dataManager.globalVehicle2Data.X);
                maxLength  = max(sim1Length, sim2Length);

                obj.dataManager.globalVehicle1Data.X = extendData(obj.dataManager.globalVehicle1Data.X, maxLength);
                obj.dataManager.globalVehicle1Data.Y = extendData(obj.dataManager.globalVehicle1Data.Y, maxLength);
                obj.dataManager.globalVehicle1Data.Theta = extendData(obj.dataManager.globalVehicle1Data.Theta, maxLength);
                obj.dataManager.globalVehicle1Data.Speed = extendData(obj.dataManager.globalVehicle1Data.Speed, maxLength);
                obj.dataManager.globalVehicle1Data.SteeringAngle = extendData(obj.dataManager.globalVehicle1Data.SteeringAngle, maxLength);

                if obj.vehicleSim1.simParams.includeTrailer
                    obj.dataManager.globalTrailer1Data.X = extendData(obj.dataManager.globalTrailer1Data.X, maxLength);
                    obj.dataManager.globalTrailer1Data.Y = extendData(obj.dataManager.globalTrailer1Data.Y, maxLength);
                    obj.dataManager.globalTrailer1Data.Theta = extendData(obj.dataManager.globalTrailer1Data.Theta, maxLength);
                    obj.dataManager.globalTrailer1Data.SteeringAngle = extendData(obj.dataManager.globalTrailer1Data.SteeringAngle, maxLength);
                end

                obj.dataManager.globalVehicle2Data.X = extendData(obj.dataManager.globalVehicle2Data.X, maxLength);
                obj.dataManager.globalVehicle2Data.Y = extendData(obj.dataManager.globalVehicle2Data.Y, maxLength);
                obj.dataManager.globalVehicle2Data.Theta = extendData(obj.dataManager.globalVehicle2Data.Theta, maxLength);
                obj.dataManager.globalVehicle2Data.Speed = extendData(obj.dataManager.globalVehicle2Data.Speed, maxLength);
                obj.dataManager.globalVehicle2Data.SteeringAngle = extendData(obj.dataManager.globalVehicle2Data.SteeringAngle, maxLength);

                if obj.vehicleSim2.simParams.includeTrailer
                    obj.dataManager.globalTrailer2Data.X = extendData(obj.dataManager.globalTrailer2Data.X, maxLength);
                    obj.dataManager.globalTrailer2Data.Y = extendData(obj.dataManager.globalTrailer2Data.Y, maxLength);
                    obj.dataManager.globalTrailer2Data.Theta = extendData(obj.dataManager.globalTrailer2Data.Theta, maxLength);
                    obj.dataManager.globalTrailer2Data.SteeringAngle = extendData(obj.dataManager.globalTrailer2Data.SteeringAngle, maxLength);
                end

                waitbar(0.25, hWaitbar, 'Simulation Data Equalized.');

                %% 11. Determine Total Steps
                disp('Determining total simulation steps...');
                totalSteps = maxLength;
                fprintf('Total Simulation Steps set to: %d\n', totalSteps);

                %% 12. Initialize Collision Data (will be filled later)
                disp('Initializing collision data...');
                obj.dataManager.collisionData.X = NaN(totalSteps, 1);
                obj.dataManager.collisionData.Y = NaN(totalSteps, 1);

                %% 13. Validate Vehicle Data
                disp('Validating vehicle data...');
                if isempty(obj.dataManager.globalVehicle1Data.X) || isempty(obj.dataManager.globalVehicle1Data.Y) || ...
                   isempty(obj.dataManager.globalVehicle2Data.X) || isempty(obj.dataManager.globalVehicle2Data.Y)
                    uialert(obj.uiManager.fig, 'One or both vehicle simulations returned empty data. Please check simulation parameters.', 'Data Error');
                    return;
                end

                %% 14. Apply Offsets (Vehicle 2)
                disp('Applying offsets to Tractor Vehicle positions...');
                obj.dataManager.globalVehicle2Data.X = obj.dataManager.globalVehicle2Data.X;
                obj.dataManager.globalVehicle2Data.Y = obj.dataManager.globalVehicle2Data.Y;
                if obj.vehicleSim2.simParams.includeTrailer
                    obj.dataManager.globalTrailer2Data.X = obj.dataManager.globalTrailer2Data.X;
                    obj.dataManager.globalTrailer2Data.Y = obj.dataManager.globalTrailer2Data.Y;
                end

                %% 15. Apply Rotation and Offset to Vehicle 1 if Enabled
                if rotateVehicle1
                    fprintf('Rotation enabled for Vehicle 1. Applying rotation of %.2f degrees.\n', vehicle1rotationAngleDeg);
                    R = [cos(vehicle1rotationAngleRad), -sin(vehicle1rotationAngleRad);
                         sin(vehicle1rotationAngleRad),  cos(vehicle1rotationAngleRad)];
                    rotatedPositionsV1 = (R * ([obj.dataManager.globalVehicle1Data.X, obj.dataManager.globalVehicle1Data.Y]).').';
                    vehicle1offsetP = [vehicle1OffsetX, vehicle1OffsetY];
                    rotatedPositionsV1 = (rotatedPositionsV1 + vehicle1offsetP)';
                    obj.dataManager.globalVehicle1Data.X = rotatedPositionsV1(1, :)';
                    obj.dataManager.globalVehicle1Data.Y = rotatedPositionsV1(2, :)';
                    obj.dataManager.globalVehicle1Data.Theta = ...
                        mod(obj.dataManager.globalVehicle1Data.Theta + vehicle1rotationAngleRad + pi, 2*pi) - pi;

                    if obj.vehicleSim1.simParams.includeTrailer
                        X_trailer = obj.dataManager.globalTrailer1Data.X;
                        Y_trailer = obj.dataManager.globalTrailer1Data.Y;
                        positionsTrailer = [X_trailer, Y_trailer];
                        rotatedPositionsTrailer = (R * positionsTrailer.').';
                        rotatedPositionsTrailer = (rotatedPositionsTrailer + vehicle1offsetP)';
                        obj.dataManager.globalTrailer1Data.X = rotatedPositionsTrailer(1, :)';
                        obj.dataManager.globalTrailer1Data.Y = rotatedPositionsTrailer(2, :)';
                        obj.dataManager.globalTrailer1Data.Theta = ...
                            mod(obj.dataManager.globalTrailer1Data.Theta + vehicle1rotationAngleRad + pi, 2*pi) - pi;
                    end
                else
                    obj.dataManager.globalVehicle1Data.X = obj.dataManager.globalVehicle1Data.X + vehicle1OffsetX;
                    obj.dataManager.globalVehicle1Data.Y = obj.dataManager.globalVehicle1Data.Y + vehicle1OffsetY;
                    if obj.vehicleSim1.simParams.includeTrailer
                        obj.dataManager.globalTrailer1Data.X = obj.dataManager.globalTrailer1Data.X + vehicle1OffsetX;
                        obj.dataManager.globalTrailer1Data.Y = obj.dataManager.globalTrailer1Data.Y + vehicle1OffsetY;
                    end
                end

                %% 16. Apply Rotation and Offset to Vehicle 2 if Enabled
                if rotateVehicle2
                    fprintf('Rotation enabled for Vehicle 2. Applying rotation of %.2f degrees.\n', rotationAngleDeg);
                    R = [cos(rotationAngleRad), -sin(rotationAngleRad);
                         sin(rotationAngleRad),  cos(rotationAngleRad)];
                    rotatedPositionsV2 = (R * ([obj.dataManager.globalVehicle2Data.X, obj.dataManager.globalVehicle2Data.Y]).').';
                    offsetP = [offsetX, offsetY];
                    rotatedPositionsV2 = (rotatedPositionsV2 + offsetP)';
                    obj.dataManager.globalVehicle2Data.X = rotatedPositionsV2(1, :)';
                    obj.dataManager.globalVehicle2Data.Y = rotatedPositionsV2(2, :)';
                    obj.dataManager.globalVehicle2Data.Theta = ...
                        mod(obj.dataManager.globalVehicle2Data.Theta + rotationAngleRad + pi, 2*pi) - pi;

                    if obj.vehicleSim2.simParams.includeTrailer
                        X_trailer = obj.dataManager.globalTrailer2Data.X;
                        Y_trailer = obj.dataManager.globalTrailer2Data.Y;
                        positionsTrailer = [X_trailer, Y_trailer];
                        rotatedPositionsTrailer = (R * positionsTrailer.').';
                        rotatedPositionsTrailer = (rotatedPositionsTrailer + offsetP)';
                        obj.dataManager.globalTrailer2Data.X = rotatedPositionsTrailer(1, :)';
                        obj.dataManager.globalTrailer2Data.Y = rotatedPositionsTrailer(2, :)';
                        obj.dataManager.globalTrailer2Data.Theta = ...
                            mod(obj.dataManager.globalTrailer2Data.Theta + rotationAngleRad + pi, 2*pi) - pi;
                    end
                else
                    obj.dataManager.globalVehicle2Data.X = obj.dataManager.globalVehicle2Data.X + offsetX;
                    obj.dataManager.globalVehicle2Data.Y = obj.dataManager.globalVehicle2Data.Y + offsetY;
                    if obj.vehicleSim2.simParams.includeTrailer
                        obj.dataManager.globalTrailer2Data.X = obj.dataManager.globalTrailer2Data.X + offsetX;
                        obj.dataManager.globalTrailer2Data.Y = obj.dataManager.globalTrailer2Data.Y + offsetY;
                    end
                end


                %% 19. Launch Collision Detection in Parallel
                disp('Launching collision detection in background...');
                % We pass in all the needed arguments. This will run the collision
                % detection logic in the background while we animate on the main thread.
                collisionFuture = parfeval(@obj.runCollisionCheck, 1, totalSteps, ...
                    vehicleParams1, trailerParams1, vehicleParams2, trailerParams2, ...
                    VehicleUnderAnalysisMaxMass, J2980AssumedMaxMass);

                %% 20. Animate Vehicles
                disp('Starting animation...');
                zoom(obj.plotManager.sharedAx, 'on');
                set(obj.plotManager.sharedAx, 'XLimMode', 'manual', 'YLimMode', 'manual');
                % Prepare lane map and initial markers once
                mapWidth  = 600;
                mapHeight = 600;
                mapObj = LaneMap(mapWidth, mapHeight);
                mapObj.LaneCommands = obj.map;
                mapObj.LaneColor    = [0.8275, 0.8275, 0.8275];
                mapObj.LaneWidth    = 5;

                % Plot the lane map without removing existing line objects
                hold(obj.plotManager.sharedAx, 'on');
                mapObj.plotLaneMapWithCommands(obj.plotManager.sharedAx, ...
                                               mapObj.LaneCommands, ...
                                               mapObj.LaneColor);
                hold(obj.plotManager.sharedAx, 'on');
                obj.plotManager.highlightInitialPositions(obj.dataManager);

                includeTrailer2 = isfield(obj.vehicleSim2.simParams, 'includeTrailer') && ...
                                      obj.vehicleSim2.simParams.includeTrailer;

                for iStep = 1:totalSteps
                    % Update trajectories and vehicle outlines
                    obj.plotManager.updateTrajectories(obj.dataManager, iStep, ...
                        obj.vehicleSim1.simParams, obj.vehicleSim2.simParams);

                    obj.plotManager.updateVehicleOutlines(obj.dataManager, iStep, ...
                        vehicleParams1, trailerParams1, ...
                        vehicleParams2, trailerParams2);

                    drawnow limitrate;
                    pause(0.01);
                end

                disp('Animation complete. Fetching collision results from the background...');
                collisionOutput = fetchOutputs(collisionFuture);
                collisionData   = collisionOutput{1};

                % You can handle collisionData as you wish (e.g., store it, re-plot).
                if collisionData.collisionDetected
                    fprintf('Collision was detected at step %d.\n', collisionData.collisionStep);
                    obj.collisionX = collisionData.collisionX;
                    obj.collisionY = collisionData.collisionY;
                    % Store collision data in DataManager
                    obj.dataManager.collisionData.X(collisionData.collisionStep) = collisionData.collisionX;
                    obj.dataManager.collisionData.Y(collisionData.collisionStep) = collisionData.collisionY;
                    % Optionally, you can re-plot the final or collision state, highlight collision, etc.
                else
                    disp('No collision detected in the entire simulation.');
                end

            catch ME
                disp(['An error occurred: ', ME.message]);
                obj.saveLogs({sprintf('Simulation Error: %s', ME.message)});
                rethrow(ME);
                waitbar(1, hWaitbar, 'Simulation Error Occurred.');
                pause(1);
                closeIfOpen(hWaitbar);
            end
        end

        %/**
        % * @brief This method runs the "for i=1:totalSteps" collision check in the background.
        % * @param totalSteps The total number of simulation steps.
        % */
        function collisionData = runCollisionCheck(obj, totalSteps, ...
                vehicleParams1, trailerParams1, vehicleParams2, trailerParams2, ...
                VehicleUnderAnalysisMaxMass, J2980AssumedMaxMass)

            collisionData.collisionDetected = false;
            collisionData.collisionStep     = NaN;
            collisionData.collisionX        = NaN;
            collisionData.collisionY        = NaN;

            fprintf('--- Starting Collision Detection in Parallel ---\n');
            for i = 2:totalSteps
                tractorCorners1 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalVehicle1Data.X(i), ...
                    obj.dataManager.globalVehicle1Data.Y(i), ...
                    obj.dataManager.globalVehicle1Data.Theta(i), ...
                    vehicleParams1, ...
                    true, ...
                    0, ...
                    vehicleParams1.numTiresPerAxle ...
                );

                tractorCorners2 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalVehicle2Data.X(i), ...
                    obj.dataManager.globalVehicle2Data.Y(i), ...
                    obj.dataManager.globalVehicle2Data.Theta(i), ...
                    vehicleParams2, ...
                    true, ...
                    0, ...
                    vehicleParams2.numTiresPerAxle ...
                );

                [collisionDetectedNow, ...
                 collisionXNow, ...
                 collisionYNow] = ...
                    obj.checkCornersForCollision(i, tractorCorners1, tractorCorners2, ...
                                                 trailerParams1, trailerParams2);

                if collisionDetectedNow
                    collisionData.collisionDetected = true;
                    collisionData.collisionStep     = i;
                    collisionData.collisionX        = collisionXNow;
                    collisionData.collisionY        = collisionYNow;
                    fprintf('Collision detected at Step %d.\n', i);
                    break;
                end
            end

            fprintf('--- Collision Detection in Parallel Completed ---\n');
        end

        %/**
        % * @brief Checks the corners for collision at step i, returning [didCollide, colX, colY].
        % */
        function [didCollide, collisionX, collisionY] = checkCornersForCollision(...
            obj, i, tractorCorners1, tractorCorners2, trailerParams1, trailerParams2)

            didCollide = false;
            collisionX = NaN;
            collisionY = NaN;

            % Trailer corners if needed
            trailerCorners1 = [];
            trailerCorners2 = [];
            if obj.vehicleSim1.simParams.includeTrailer && ~isempty(trailerParams1)
                trailerCorners1 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalTrailer1Data.X(i), ...
                    obj.dataManager.globalTrailer1Data.Y(i), ...
                    obj.dataManager.globalTrailer1Data.Theta(i), ...
                    trailerParams1, ...
                    false, ...
                    0, ...
                    trailerParams1.numTiresPerAxle ...
                );
            end
            if obj.vehicleSim2.simParams.includeTrailer && ~isempty(trailerParams2)
                trailerCorners2 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalTrailer2Data.X(i), ...
                    obj.dataManager.globalTrailer2Data.Y(i), ...
                    obj.dataManager.globalTrailer2Data.Theta(i), ...
                    trailerParams2, ...
                    false, ...
                    0, ...
                    trailerParams2.numTiresPerAxle ...
                );
            end

            % Check collisions
            collisionTT = obj.collisionDetector.checkCollision(tractorCorners1, tractorCorners2);
            collisionTrailers = false;

            if ~isempty(trailerCorners1) && ~isempty(trailerCorners2)
                collisionTrailers = obj.collisionDetector.checkCollision(trailerCorners1, trailerCorners2);
            end

            collisionTractor1Trailer2 = false;
            collisionTractor2Trailer1 = false;
            if ~isempty(trailerCorners1)
                collisionTractor2Trailer1 = obj.collisionDetector.checkCollision(tractorCorners2, trailerCorners1);
            end
            if ~isempty(trailerCorners2)
                collisionTractor1Trailer2 = obj.collisionDetector.checkCollision(tractorCorners1, trailerCorners2);
            end

            if collisionTT || collisionTrailers || collisionTractor1Trailer2 || collisionTractor2Trailer1
                didCollide = true;

                % Attempt to compute intersection center
                poly1 = polyshape(tractorCorners1(:,1), tractorCorners1(:,2));
                poly2 = polyshape(tractorCorners2(:,1), tractorCorners2(:,2));
                if ~isempty(trailerCorners1)
                    poly1 = union(poly1, polyshape(trailerCorners1(:,1), trailerCorners1(:,2)));
                end
                if ~isempty(trailerCorners2)
                    poly2 = union(poly2, polyshape(trailerCorners2(:,1), trailerCorners2(:,2)));
                end
                intersection = intersect(poly1, poly2);
                if ~isempty(intersection.Vertices)
                    collisionX = mean(intersection.Vertices(:,1));
                    collisionY = mean(intersection.Vertices(:,2));
                else
                    % Fallback approximate
                    collisionX = 0.5 * (obj.dataManager.globalVehicle1Data.X(i) + ...
                                        obj.dataManager.globalVehicle2Data.X(i));
                    collisionY = 0.5 * (obj.dataManager.globalVehicle1Data.Y(i) + ...
                                        obj.dataManager.globalVehicle2Data.Y(i));
                end
            end
        end

        % (Remaining methods unchanged except for removing the old in-line collision loop)

        function transferSimulationResults(obj)
            requiredFields = {'X', 'Y', 'Theta', 'speedData', 'steeringAngles'};
            if isstruct(obj.sim1Results)
                missingFields = setdiff(requiredFields, fieldnames(obj.sim1Results));
                if ~isempty(missingFields)
                    error('sim1Results is missing required fields: %s', strjoin(missingFields, ', '));
                end
                obj.dataManager.globalVehicle1Data.X = obj.sim1Results.X(:);
                obj.dataManager.globalVehicle1Data.Y = obj.sim1Results.Y(:);
                obj.dataManager.globalVehicle1Data.Theta = obj.sim1Results.Theta(:);
                obj.dataManager.globalVehicle1Data.Speed = obj.sim1Results.speedData(:);
                obj.dataManager.globalVehicle1Data.SteeringAngle = obj.sim1Results.steeringAngles(:);
            else
                error('sim1Results is not a structure. Expected a structure.');
            end

            if obj.vehicleSim1.simParams.includeTrailer
                if isfield(obj.sim1Results, 'trailerX') && isfield(obj.sim1Results, 'trailerY') && isfield(obj.sim1Results, 'trailerTheta')
                    obj.dataManager.globalTrailer1Data.X = obj.sim1Results.trailerX(:);
                    obj.dataManager.globalTrailer1Data.Y = obj.sim1Results.trailerY(:);
                    obj.dataManager.globalTrailer1Data.Theta = obj.sim1Results.trailerTheta(:);
                    if isfield(obj.sim1Results, 'trailerSteeringAngles') && ~isempty(obj.sim1Results.trailerSteeringAngles)
                        obj.dataManager.globalTrailer1Data.SteeringAngle = obj.sim1Results.trailerSteeringAngles(:);
                    else
                        obj.dataManager.globalTrailer1Data.SteeringAngle = zeros(length(obj.dataManager.globalTrailer1Data.X), 1);
                        warning('No trailerSteeringAngles in sim1Results. Assigning zeros.');
                    end
                else
                    error('sim1Results is missing trailer fields for Vehicle 1.');
                end
            else
                obj.dataManager.globalTrailer1Data.X = [];
                obj.dataManager.globalTrailer1Data.Y = [];
                obj.dataManager.globalTrailer1Data.Theta = [];
                obj.dataManager.globalTrailer1Data.SteeringAngle = [];
            end

            if isstruct(obj.sim2Results)
                missingFields = setdiff(requiredFields, fieldnames(obj.sim2Results));
                if ~isempty(missingFields)
                    error('sim2Results is missing required fields: %s', strjoin(missingFields, ', '));
                end
                obj.dataManager.globalVehicle2Data.X = obj.sim2Results.X(:);
                obj.dataManager.globalVehicle2Data.Y = obj.sim2Results.Y(:);
                obj.dataManager.globalVehicle2Data.Theta = obj.sim2Results.Theta(:);
                obj.dataManager.globalVehicle2Data.Speed = obj.sim2Results.speedData(:);
                obj.dataManager.globalVehicle2Data.SteeringAngle = obj.sim2Results.steeringAngles(:);
            else
                error('sim2Results is not a structure. Expected a structure.');
            end

            if obj.vehicleSim2.simParams.includeTrailer
                if isfield(obj.sim2Results, 'trailerX') && isfield(obj.sim2Results, 'trailerY') && isfield(obj.sim2Results, 'trailerTheta')
                    obj.dataManager.globalTrailer2Data.X = obj.sim2Results.trailerX(:);
                    obj.dataManager.globalTrailer2Data.Y = obj.sim2Results.trailerY(:);
                    obj.dataManager.globalTrailer2Data.Theta = obj.sim2Results.trailerTheta(:);
                    if isfield(obj.sim2Results, 'trailerSteeringAngles') && ~isempty(obj.sim2Results.trailerSteeringAngles)
                        obj.dataManager.globalTrailer2Data.SteeringAngle = obj.sim2Results.trailerSteeringAngles(:);
                    else
                        obj.dataManager.globalTrailer2Data.SteeringAngle = zeros(length(obj.dataManager.globalTrailer2Data.X), 1);
                        warning('No trailerSteeringAngles in sim2Results. Assigning zeros.');
                    end
                else
                    error('sim2Results is missing trailer fields for Vehicle 2.');
                end
            else
                obj.dataManager.globalTrailer2Data.X = [];
                obj.dataManager.globalTrailer2Data.Y = [];
                obj.dataManager.globalTrailer2Data.Theta = [];
                obj.dataManager.globalTrailer2Data.SteeringAngle = [];
            end
        end

        function saveLogs(obj, logMessages)
            txtFilename = obj.getUniqueFilename('simulation_Vehicle1', '.txt');
            csvFilename = obj.getUniqueFilename('simulation_Vehicle1', '.csv');

            fid = fopen(txtFilename, 'w');
            if fid ~= -1
                fprintf(fid, '%s\n', logMessages{:});
                fclose(fid);
                disp(['Log data saved to ', txtFilename]);
            else
                warning('Failed to write simulation log to text file.');
            end

            logTable = table(logMessages', 'VariableNames', {'Logs'});
            try
                writetable(logTable, csvFilename);
                disp(['Log data saved to ', csvFilename]);
            catch ME
                warning('Failed to write simulation log to CSV file: %s', ME.message);
            end
        end

        function uniqueName = getUniqueFilename(~, baseName, extension)
            uniqueName = [baseName, extension];
            if exist(uniqueName, 'file') == 2
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                uniqueName = [baseName, '_', timestamp, extension];
            end
        end

        function params = createVehicleParams(obj, simParams, wheelHeight, wheelWidth)
            if isfield(simParams, 'numTiresPerAxleTractor')
                numTiresPerAxle = simParams.numTiresPerAxleTractor;
            else
                numTiresPerAxle = 4;
                warning('Missing numTiresPerAxleTractor. Using default: 4');
            end

            params = struct(...
                'isTractor', true, ...
                'length', simParams.tractorLength, ...
                'width', simParams.tractorWidth, ...
                'height', simParams.tractorHeight, ...
                'CoGHeight', simParams.tractorCoGHeight, ...
                'wheelbase', simParams.tractorWheelbase, ...
                'trackWidth', simParams.tractorTrackWidth, ...
                'numAxles', simParams.tractorNumAxles, ...
                'axleSpacing', simParams.tractorAxleSpacing, ...
                'mass', simParams.tractorMass, ...
                'wheelWidth', wheelWidth, ...
                'wheelHeight', wheelHeight, ...
                'numTiresPerAxle', numTiresPerAxle ...
            );
        end

        function params = createTrailerParams(obj, simParams, wheelHeight, wheelWidth, trailerNumber)
            requiredFields = {'trailerLength', 'trailerWidth', 'trailerHeight', 'trailerCoGHeight', ...
                              'trailerWheelbase', 'trailerTrackWidth', 'trailerNumAxles', ...
                              'trailerAxleSpacing', 'trailerMass', 'numTiresPerAxleTrailer', ...
                              'W_FrontLeft', 'W_FrontRight', 'W_RearLeft', 'W_RearRight'};
            for i = 1:length(requiredFields)
                if ~isfield(simParams, requiredFields{i})
                    error('simParams missing %s', requiredFields{i});
                end
            end

            numTiresPerAxle = simParams.numTiresPerAxleTrailer;
            if isempty(numTiresPerAxle) || ~isnumeric(numTiresPerAxle) || numTiresPerAxle <= 0
                numTiresPerAxle = 4;
                warning('Invalid numTiresPerAxleTrailer. Using default: 4');
            end

            numAxlesTractor   = simParams.tractorNumAxles;
            wheelbaseTractor  = simParams.tractorWheelbase;
            if numAxlesTractor > 1
                axlePositionsTractor = linspace(wheelbaseTractor / 2, -wheelbaseTractor / 2, numAxlesTractor);
            else
                axlePositionsTractor = 0;
            end

            frontAxlePosition = axlePositionsTractor(1);
            rearAxlePosition  = axlePositionsTractor(end);
            middleAxleIndex   = ceil(numAxlesTractor / 2);
            middleAxlePosition = axlePositionsTractor(middleAxleIndex);

            params = struct(...
                'isTractor', false, ...
                'length', simParams.trailerLength, ...
                'width', simParams.trailerWidth, ...
                'height', simParams.trailerHeight, ...
                'CoGHeight', simParams.trailerCoGHeight, ...
                'wheelbase', simParams.trailerWheelbase, ...
                'trackWidth', simParams.trailerTrackWidth, ...
                'numAxles', simParams.trailerNumAxles, ...
                'axleSpacing', simParams.trailerAxleSpacing, ...
                'mass', simParams.trailerMass, ...
                'wheelWidth', wheelWidth, ...
                'wheelHeight', wheelHeight, ...
                'numTiresPerAxle', numTiresPerAxle, ...
                'W_Total', simParams.W_FrontLeft + simParams.W_FrontRight + ...
                           simParams.W_RearLeft + simParams.W_RearRight, ...
                'includeTrailer', simParams.includeTrailer, ...
                'HitchDistance', simParams.trailerHitchDistance + ...
                                 ((middleAxlePosition + rearAxlePosition) / 2) ...
            );

            if trailerNumber == 1
                obj.dataManager.TrailerLength1 = simParams.trailerLength;
            elseif trailerNumber == 2
                obj.dataManager.TrailerLength2 = simParams.trailerLength;
            end
        end
    end

    methods(Static)
        function simResults = runVehicleSim1(vehicleSim, stepCallback)
            if nargin < 2 || isempty(stepCallback)
                stepCallback = [];
            end
            [X, Y, Theta, trailerX, trailerY, trailerTheta, flags, ...
                steeringAngles, speedData] = vehicleSim.runSimulation();
            simResults = struct(...
                'X', X, ...
                'Y', Y, ...
                'Theta', Theta, ...
                'trailerX', trailerX, ...
                'trailerY', trailerY, ...
                'trailerTheta', trailerTheta, ...
                'flags', flags, ...
                'steeringAngles', steeringAngles, ...
                'speedData', speedData ...
            );
        end

        function simResults = runVehicleSim2(vehicleSim, stepCallback)
            if nargin < 2 || isempty(stepCallback)
                stepCallback = [];
            end
            [X, Y, Theta, trailerX, trailerY, trailerTheta, flags, ...
                steeringAngles, speedData] = vehicleSim.runSimulation();
            simResults = struct(...
                'X', X, ...
                'Y', Y, ...
                'Theta', Theta, ...
                'trailerX', trailerX, ...
                'trailerY', trailerY, ...
                'trailerTheta', trailerTheta, ...
                'flags', flags, ...
                'steeringAngles', steeringAngles, ...
                'speedData', speedData ...
            );
        end
    end
end

function extendedData = extendData(data, targetLength)
    currentLength = length(data);
    if currentLength == 0
        extendedData = zeros(targetLength, 1);
    elseif currentLength < targetLength
        lastValue  = data(end);
        extension  = repmat(lastValue, targetLength - currentLength, 1);
        extendedData = [data; extension];
    else
        extendedData = data;
    end
end

function closeIfOpen(h)
    if isvalid(h)
        close(h);
    end
end
