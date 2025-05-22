%{
% @file PlotManager.m
% @brief Generates figures and manages plot updates for the simulation.
%        Handles different views for vehicle trajectories.
% @author Miguel Marina
%}

classdef PlotManager < handle
    % PlotManager class that manages and visualizes plots for vehicle simulations.

    properties
        sharedAx          % Shared axes for plotting vehicle trajectories
        uiManager         % Reference to the UIManager
        collisionDetector % Reference to the CollisionDetector

        % Handles to line objects for updating data each frame:
        veh1Line
        trl1Line
        trl1RearLine
        veh2Line
        trl2Line
        trl2RearLine

        % Handles to vehicle outlines for animation
        veh1Outline
        trl1Outline
        veh2Outline
        trl2Outline

        % Handles to full vehicle graphics (body and wheels)
        veh1Graphics
        trl1Graphics
        veh2Graphics
        trl2Graphics


        % Handles to initial-position markers
        veh1StartMarker
        trl1StartMarker
        veh2StartMarker
        trl2StartMarker
    end

    methods
        %% Constructor
        function obj = PlotManager(sharedAx, uiManager, collisionDetector)
            % Constructor to initialize the PlotManager with shared axes and managers.
            if nargin < 3
                error('PlotManager requires three input arguments: sharedAx, uiManager, collisionDetector.');
            end
            obj.sharedAx = sharedAx;
            obj.uiManager = uiManager;
            obj.collisionDetector = collisionDetector;

            % Initialize the plotting area (once)
            cla(obj.sharedAx); 
            hold(obj.sharedAx, 'on');
            grid(obj.sharedAx, 'on');

            % Basic labeling (only do it once)
            xlabel(obj.sharedAx, 'Longitudinal Distance (m)');
            ylabel(obj.sharedAx, 'Lateral Distance (m)');
            title(obj.sharedAx, 'Vehicle Simulation Trajectories');
            axis(obj.sharedAx, 'equal');

            % Example fixed axis limits or manual. You can set these once:
            xlim(obj.sharedAx, [0, 1000]);
            ylim(obj.sharedAx, [-10, 1010]);
            % Turn on or off as needed:
            % set(obj.sharedAx, 'XLimMode', 'manual', 'YLimMode', 'manual');

            % Pre-create empty line objects (so we can just set data later)
            obj.veh1Line = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 1.5, ...
                'DisplayName', 'Vehicle 1 Trajectory');
            obj.trl1Line = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 1 Trajectory');
            obj.trl1RearLine = plot(obj.sharedAx, NaN, NaN, 'b--', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 1 Rear');

            obj.veh2Line = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 1.5, ...
                'DisplayName', 'Vehicle 2 Trajectory');
            obj.trl2Line = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 2 Trajectory');
            obj.trl2RearLine = plot(obj.sharedAx, NaN, NaN, 'c--', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 2 Rear');

            % Pre-create initial position markers
            obj.veh1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'r', 'filled', ...
                'DisplayName', 'Vehicle 1 Start');
            obj.trl1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'b', 'filled', ...
                'DisplayName', 'Trailer 1 Start');
            obj.veh2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'm', 'filled', ...
                'DisplayName', 'Vehicle 2 Start');
            obj.trl2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'c', 'filled', ...
                'DisplayName', 'Trailer 2 Start');

            % Create vehicle outline handles for animation
            obj.veh1Outline = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 2);
            obj.trl1Outline = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 2);
            obj.veh2Outline = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 2);
            obj.trl2Outline = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 2);

            % Reset full vehicle graphic handles
            obj.veh1Graphics = gobjects(0);
            obj.trl1Graphics = gobjects(0);
            obj.veh2Graphics = gobjects(0);
            obj.trl2Graphics = gobjects(0);

            % Initialize full vehicle graphic handles
            obj.veh1Graphics = gobjects(0);
            obj.trl1Graphics = gobjects(0);
            obj.veh2Graphics = gobjects(0);
            obj.trl2Graphics = gobjects(0);
        end

        %% Clear Plots
        function clearPlots(obj)
            % Clears all plotted data from the shared axes
            cla(obj.sharedAx);
            hold(obj.sharedAx, 'on');
            grid(obj.sharedAx, 'on');

            % Re-init lines so we don't create duplicates
            obj.veh1Line = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 1.5);
            obj.trl1Line = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 1.5);
            obj.trl1RearLine = plot(obj.sharedAx, NaN, NaN, 'b--', 'LineWidth', 1.5);

            obj.veh2Line = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 1.5);
            obj.trl2Line = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 1.5);
            obj.trl2RearLine = plot(obj.sharedAx, NaN, NaN, 'c--', 'LineWidth', 1.5);

            % And the markers
            obj.veh1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'r', 'filled');
            obj.trl1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'b', 'filled');
            obj.veh2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'm', 'filled');
            obj.trl2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'c', 'filled');

            % Recreate vehicle outline handles
            obj.veh1Outline = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 2);
            obj.trl1Outline = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 2);
            obj.veh2Outline = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 2);
            obj.trl2Outline = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 2);


            % Keep existing axes settings
            xlabel(obj.sharedAx, 'Longitudinal Distance (m)');
            ylabel(obj.sharedAx, 'Lateral Distance (m)');
            title(obj.sharedAx, 'Vehicle Simulation Trajectories');
            axis(obj.sharedAx, 'equal');
        end

        %% Update Trajectories (Fast Approach)
        function updateTrajectories(obj, dataManager, iStep, simParams1, simParams2)
            % Instead of plotting the entire path each time,
            % let's only plot up to iStep by updating XData/YData of the line objects.

            % Vehicle 1 path up to iStep
            Xv1 = dataManager.globalVehicle1Data.X(1:iStep);
            Yv1 = dataManager.globalVehicle1Data.Y(1:iStep);
            set(obj.veh1Line, 'XData', Xv1, 'YData', Yv1);

            % If trailer 1 is present
            if ~isempty(dataManager.globalTrailer1Data.X)
                Xtrl1 = dataManager.globalTrailer1Data.X(1:iStep);
                Ytrl1 = dataManager.globalTrailer1Data.Y(1:iStep);
                set(obj.trl1Line, 'XData', Xtrl1, 'YData', Ytrl1);

                rearX_Trailer1 = Xtrl1 - (simParams1.trailerLength/2).*cos(dataManager.globalTrailer1Data.Theta(1:iStep));
                rearY_Trailer1 = Ytrl1 - (simParams1.trailerLength/2).*sin(dataManager.globalTrailer1Data.Theta(1:iStep));
                set(obj.trl1RearLine, 'XData', rearX_Trailer1, 'YData', rearY_Trailer1);
            else
                % If no trailer, set them NaN to keep them hidden
                set(obj.trl1Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl1RearLine, 'XData', NaN, 'YData', NaN);
            end

            % Vehicle 2 path up to iStep
            Xv2 = dataManager.globalVehicle2Data.X(1:iStep);
            Yv2 = dataManager.globalVehicle2Data.Y(1:iStep);
            set(obj.veh2Line, 'XData', Xv2, 'YData', Yv2);

            % If trailer 2 is present
            if ~isempty(dataManager.globalTrailer2Data.X)
                Xtrl2 = dataManager.globalTrailer2Data.X(1:iStep);
                Ytrl2 = dataManager.globalTrailer2Data.Y(1:iStep);
                set(obj.trl2Line, 'XData', Xtrl2, 'YData', Ytrl2);

                rearX_Trailer2 = Xtrl2 - (simParams2.trailerLength/2).*cos(dataManager.globalTrailer2Data.Theta(1:iStep));
                rearY_Trailer2 = Ytrl2 - (simParams2.trailerLength/2).*sin(dataManager.globalTrailer2Data.Theta(1:iStep));
                set(obj.trl2RearLine, 'XData', rearX_Trailer2, 'YData', rearY_Trailer2);
            else
                % If no trailer, set them NaN to keep them hidden
                set(obj.trl2Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl2RearLine, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Highlight Initial Positions
        function highlightInitialPositions(obj, dataManager)
            % Update the initial position markers to the actual initial positions

            set(obj.veh1StartMarker, 'XData', dataManager.globalVehicle1Data.X(1), ...
                                     'YData', dataManager.globalVehicle1Data.Y(1));
            if ~isempty(dataManager.globalTrailer1Data.X)
                set(obj.trl1StartMarker, 'XData', dataManager.globalTrailer1Data.X(1), ...
                                         'YData', dataManager.globalTrailer1Data.Y(1));
            else
                set(obj.trl1StartMarker, 'XData', NaN, 'YData', NaN);
            end

            set(obj.veh2StartMarker, 'XData', dataManager.globalVehicle2Data.X(1), ...
                                     'YData', dataManager.globalVehicle2Data.Y(1));
            if ~isempty(dataManager.globalTrailer2Data.X)
                set(obj.trl2StartMarker, 'XData', dataManager.globalTrailer2Data.X(1), ...
                                         'YData', dataManager.globalTrailer2Data.Y(1));
            else
                set(obj.trl2StartMarker, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Update Vehicle Outlines
        function updateVehicleOutlines(obj, dataManager, iStep, vehicleParams1, trailerParams1, vehicleParams2, trailerParams2)
            % Delete previous vehicle graphics
            if ~isempty(obj.veh1Graphics); delete(obj.veh1Graphics(ishandle(obj.veh1Graphics))); end
            if ~isempty(obj.trl1Graphics); delete(obj.trl1Graphics(ishandle(obj.trl1Graphics))); end
            if ~isempty(obj.veh2Graphics); delete(obj.veh2Graphics(ishandle(obj.veh2Graphics))); end
            if ~isempty(obj.trl2Graphics); delete(obj.trl2Graphics(ishandle(obj.trl2Graphics))); end

            % Plot updated vehicles with wheels
            sa1 = rad2deg(dataManager.globalVehicle1Data.SteeringAngle(iStep));
            obj.veh1Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle1Data.X(iStep), ...
                dataManager.globalVehicle1Data.Y(iStep), ...
                dataManager.globalVehicle1Data.Theta(iStep), ...
                vehicleParams1, 'r', true, false, sa1, ...
                vehicleParams1.numTiresPerAxle, vehicleParams1.numAxles);

            if ~isempty(trailerParams1)
                obj.trl1Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    dataManager.globalTrailer1Data.X(iStep), ...
                    dataManager.globalTrailer1Data.Y(iStep), ...
                    dataManager.globalTrailer1Data.Theta(iStep), ...
                    trailerParams1, 'b', false, false, 0, ...
                    trailerParams1.numTiresPerAxle, trailerParams1.numAxles);
            else
                obj.trl1Graphics = gobjects(0);
            end

            sa2 = rad2deg(dataManager.globalVehicle2Data.SteeringAngle(iStep));
            obj.veh2Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle2Data.X(iStep), ...
                dataManager.globalVehicle2Data.Y(iStep), ...
                dataManager.globalVehicle2Data.Theta(iStep), ...
                vehicleParams2, 'm', true, false, sa2, ...
                vehicleParams2.numTiresPerAxle, vehicleParams2.numAxles);

            if ~isempty(trailerParams2)
                obj.trl2Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    dataManager.globalTrailer2Data.X(iStep), ...
                    dataManager.globalTrailer2Data.Y(iStep), ...
                    dataManager.globalTrailer2Data.Theta(iStep), ...
                    trailerParams2, 'c', false, false, 0, ...
                    trailerParams2.numTiresPerAxle, trailerParams2.numAxles);
            else
                obj.trl2Graphics = gobjects(0);
            end
        end


        %% Set Axis Limits (Optional to call once if you want)
        function setAxisLimits(obj, dataManager, stepsToPlot, collisionDetected)
            % You can still set axis limits once based on data
            % ...
            % For faster animation, consider not calling this at every frame.
            relevantX = [];
            relevantY = [];
            if collisionDetected
                relevantX = [relevantX; dataManager.globalVehicle1Data.X(1:stepsToPlot)];
                relevantX = [relevantX; dataManager.globalVehicle2Data.X(1:stepsToPlot)];
                relevantY = [relevantY; dataManager.globalVehicle1Data.Y(1:stepsToPlot)];
                relevantY = [relevantY; dataManager.globalVehicle2Data.Y(1:stepsToPlot)];
                if ~isempty(dataManager.globalTrailer1Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer1Data.X(1:stepsToPlot)];
                    relevantY = [relevantY; dataManager.globalTrailer1Data.Y(1:stepsToPlot)];
                end
                if ~isempty(dataManager.globalTrailer2Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer2Data.X(1:stepsToPlot)];
                    relevantY = [relevantY; dataManager.globalTrailer2Data.Y(1:stepsToPlot)];
                end
            else
                relevantX = [relevantX; dataManager.globalVehicle1Data.X(:)];
                relevantX = [relevantX; dataManager.globalVehicle2Data.X(:)];
                relevantY = [relevantY; dataManager.globalVehicle1Data.Y(:)];
                relevantY = [relevantY; dataManager.globalVehicle2Data.Y(:)];
                if ~isempty(dataManager.globalTrailer1Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer1Data.X(:)];
                    relevantY = [relevantY; dataManager.globalTrailer1Data.Y(:)];
                end
                if ~isempty(dataManager.globalTrailer2Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer2Data.X(:)];
                    relevantY = [relevantY; dataManager.globalTrailer2Data.Y(:)];
                end
            end

            if ~isempty(relevantX)
                minX = min(relevantX) - 10;
                maxX = max(relevantX) + 10;
                minY = min(relevantY) - 10;
                maxY = max(relevantY) + 10;

                if minX == maxX, minX = minX - 1; maxX = maxX + 1; end
                if minY == maxY, minY = minY - 1; maxY = maxY + 1; end

                xlim(obj.sharedAx, [minX, maxX]);
                ylim(obj.sharedAx, [minY, maxY]);
            end

            axis(obj.sharedAx, 'equal');
        end

        %% Plot Vehicles at a Single Step (Unchanged, but removed repeated axis calls)
        function plotVehicles(obj, dataManager, plotStep, vehicleParams1, trailerParams1, ...
                              vehicleParams2, trailerParams2, steeringAnglesSim1, steeringAnglesSim2)
            % Optionally plot the tractor/trailer polygons at this step

            % Convert steering angles from radians to degrees
            steeringAngleTractor1 = rad2deg(steeringAnglesSim1(plotStep));
            steeringAngleTractor2 = rad2deg(steeringAnglesSim2(plotStep));

            % Plot Trailers First
            if ~isempty(dataManager.globalTrailer1Data.X)
                VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    dataManager.globalTrailer1Data.X(plotStep), ...
                    dataManager.globalTrailer1Data.Y(plotStep), ...
                    dataManager.globalTrailer1Data.Theta(plotStep), ...
                    trailerParams1, ...
                    'b', ...
                    false, ...
                    false, ...
                    0, ...
                    trailerParams1.numTiresPerAxle, ...
                    trailerParams1.numAxles);
            end
            if ~isempty(dataManager.globalTrailer2Data.X)
                VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    dataManager.globalTrailer2Data.X(plotStep), ...
                    dataManager.globalTrailer2Data.Y(plotStep), ...
                    dataManager.globalTrailer2Data.Theta(plotStep), ...
                    trailerParams2, ...
                    'c', ...
                    false, ...
                    false, ...
                    0, ...
                    trailerParams2.numTiresPerAxle, ...
                    trailerParams2.numAxles);
            end

            % Then plot the tractors
            VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle1Data.X(plotStep), ...
                dataManager.globalVehicle1Data.Y(plotStep), ...
                dataManager.globalVehicle1Data.Theta(plotStep), ...
                vehicleParams1, ...
                'r', ...
                true, ...
                false, ...
                steeringAngleTractor1, ...
                vehicleParams1.numTiresPerAxle, ...
                vehicleParams1.numAxles);

            VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle2Data.X(plotStep), ...
                dataManager.globalVehicle2Data.Y(plotStep), ...
                dataManager.globalVehicle2Data.Theta(plotStep), ...
                vehicleParams2, ...
                'm', ...
                true, ...
                false, ...
                steeringAngleTractor2, ...
                vehicleParams2.numTiresPerAxle, ...
                vehicleParams2.numAxles);
        end

        %% Plot Collision Point
        function plotCollisionPoint(obj, dataManager, plotStep, dt, collisionSeverity, ...
                                    vehicleParams1, trailerParams1, collisionX, collisionY)
            % (Same logic, but remove repeated calls to axis/ylim for performance)
            elapsedTime = ((plotStep - 1) * dt);

            if isnan(collisionX) || isnan(collisionY)
                uialert(obj.uiManager.fig, 'Computed collision coordinates are NaN.', 'Collision Plot Error');
                fprintf('Error: Collision coordinates are NaN.\n');
                return;
            end
            scatter(obj.sharedAx, collisionX, collisionY, 150, 'k', 'filled', 'DisplayName', 'Collision Point');
            uialert(obj.uiManager.fig, ...
                sprintf('Collision at step %d (Time: %.2f s).', plotStep, elapsedTime), ...
                'Collision Alert');

            % Then do the collision severity, etc.
            obj.integrateCollisionSeverity(dataManager, plotStep, dt, collisionSeverity, ...
                                           vehicleParams1, trailerParams1, collisionX, collisionY);
        end

        %% Integrate Collision Severity
        function integrateCollisionSeverity(obj, dataManager, plotStep, dt, collisionSeverity, vehicleParams1, trailerParams1, collisionX, collisionY)
            % Integrates collision severity calculations and displays results.

            % Verify that required fields exist in collisionSeverity
            requiredFields = {'collisionPT', 'collisionPTrailer', 'vehicle2Params', 'vehicle1Params', ...
                              'AverageVehicle1Mass', 'AverageVehicle2Mass', 'bound_type_str', ...
                              'includeTrailer1', 'includeTrailer2'};
            missingFields = setdiff(requiredFields, fieldnames(collisionSeverity));
            if ~isempty(missingFields)
                uialert(obj.uiManager.fig, sprintf('Missing fields in collisionSeverity: %s', strjoin(missingFields, ', ')), ...
                        'Collision Severity Error');
                return;
            end

            % Determine Collision Direction and Set Vehicle Parameters Accordingly
            % **Added checks for trailer presence to prevent inversion when trailers are absent**
            if collisionSeverity.collisionPT
                % Collision from J2980 Vehicle to Truck
                % Vehicle 1: J2980 Vehicle (Target)
                % Vehicle 2: Truck (Bullet)
                m1 = collisionSeverity.vehicle2Params.mass;
                vehicle1_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle1Params.mass; % Assuming 'vehicle1Params' holds tractor mass
                vehicle2_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end

            elseif collisionSeverity.collisionPTrailer && collisionSeverity.includeTrailer1 && collisionSeverity.includeTrailer2
                % Collision from Truck to J2980 Vehicle
                % Vehicle 1: Truck (Target)
                % Vehicle 2: J2980 Vehicle (Bullet)
                m1 = collisionSeverity.vehicle1Params.mass;
                vehicle1_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle2Params.mass;
                vehicle2_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end

            else
                % **Handle cases where trailers are not present to prevent inversion**
                % Default to collisionPT logic without inversion
                m1 = collisionSeverity.vehicle2Params.mass;
                vehicle1_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle1Params.mass;
                vehicle2_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end
            end

            % Create initial velocity vectors
            collisionSeverity.v1_initial = [v1_x; v1_y; 0]; % Target's velocity
            collisionSeverity.v2_initial = [v2_x; v2_y; 0]; % Bullet's velocity

            % Compute Position Vectors from COM to Collision Point
            r1 = [collisionX - vehicle1_X; collisionY - vehicle1_Y; 0]; % Vector from Vehicle 1 COM to collision point
            r2 = [collisionX - vehicle2_X; collisionY - vehicle2_Y; 0]; % Vector from Vehicle 2 COM to collision point

            % Define Collision Normal Unit Vector
            n_vector = [vehicle2_X - vehicle1_X; vehicle2_Y - vehicle1_Y; 0];
            if norm(n_vector) == 0
                uialert(obj.uiManager.fig, 'Collision normal vector has zero magnitude.', 'Collision Error');
                return;
            end
            n_vector = n_vector / norm(n_vector); % Normalize to get a unit vector

            % Compute Heading Vectors for Both Vehicles
            h1 = [cos(vehicle1_Theta); sin(vehicle1_Theta); 0]; % Heading vector of Vehicle 1
            if norm(h1) == 0
                uialert(obj.uiManager.fig, 'Vehicle 1 heading vector has zero magnitude.', 'Collision Error');
                return;
            end
            h1 = h1 / norm(h1); % Ensure it's a unit vector

            h2 = [cos(vehicle2_Theta); sin(vehicle2_Theta); 0]; % Heading vector of Vehicle 2
            if norm(h2) == 0
                uialert(obj.uiManager.fig, 'Vehicle 2 heading vector has zero magnitude.', 'Collision Error');
                return;
            end
            h2 = h2 / norm(h2); % Ensure it's a unit vector

            % Determine Collision Types Separately for Each Vehicle
            collisionType_vehicle1 = obj.determineCollisionType(n_vector, h1);
            collisionType_vehicle2 = obj.determineCollisionType(-n_vector, h2);

            % Compute Relative Velocity Vector before collision
            v_relative_vec = collisionSeverity.v2_initial - collisionSeverity.v1_initial; % Correct relative velocity vector

            % Compute Coefficient of Restitution using get_cor_takeda
            impact_velocity = norm(v_relative_vec(1:2)); % Only horizontal components
            e = VehicleCollisionSeverity.get_cor_takeda(impact_velocity);
            fprintf('Computed Coefficient of Restitution (e): %.4f\n', e);

            % Instantiate VehicleCollisionSeverity Object with Current Velocities
            collisionSeverityObj = VehicleCollisionSeverity(m1, collisionSeverity.v1_initial, r1, ...
                                                           m2, collisionSeverity.v2_initial, r2, ...
                                                           e, n_vector);

            % Configure Severity Mapping Parameters from UI Inputs
            collisionSeverityObj.J2980AssumedMaxMass = collisionSeverity.J2980AssumedMaxMass;
            collisionSeverityObj.VehicleUnderAnalysisMaxMass = collisionSeverity.VehicleUnderAnalysisMaxMass;

            % Set Bound Type (retrieved from UI selection)
            collisionSeverityObj.bound_type_str = collisionSeverity.bound_type_str;

            % Set Collision Types in collisionSeverity object
            collisionSeverityObj.CollisionType_vehicle1 = collisionType_vehicle1;
            collisionSeverityObj.CollisionType_vehicle2 = collisionType_vehicle2;

            % Calculate Initial Speeds BEFORE Performing Collision
            initialSpeed_target = norm(collisionSeverity.v1_initial(1:2)) * 3.6;         % Target speed in kph
            initialSpeed_bullet = norm(collisionSeverity.v2_initial(1:2)) * 3.6;         % Bullet speed in kph

            % Perform collision to update velocities
            collisionSeverityObj = collisionSeverityObj.PerformCollision();

            % Perform Collision Severity Calculation
            [deltaV_target, deltaV_bullet, TargetSeverity, BulletSeverity] = collisionSeverityObj.CalculateSeverity();

            % Display Results and Inputs in a Message Box with Enhanced Formatting
            msgbox(sprintf(['\nCollision Severity Inputs:\n', ...
                            '- Collision Type for Vehicle 1: %s\n', ...
                            '- Collision Type for Vehicle 2: %s\n', ...
                            '- Initial Speed of Target: %.2f kph\n', ...
                            '- Initial Speed of Bullet: %.2f kph\n', ...
                            '- Average J2980 Vehicle Mass: %.2f kg\n', ...
                            '- Average Vehicle Under Analysis Mass: %.2f kg\n', ...
                            '- Bound Type: %s\n\n', ...
                            'Collision Severity Results:\n', ...
                            '- Target Severity: %s\n', ...
                            '- Bullet Severity: %s\n', ...
                            '- Target Delta-V: %.2f kph\n', ...
                            '- Bullet Delta-V: %.2f kph\n'], ...
                           collisionType_vehicle1, ...
                           collisionType_vehicle2, ...
                           initialSpeed_target, ...
                           initialSpeed_bullet, ...
                           collisionSeverity.J2980AssumedMaxMass, ...
                           collisionSeverity.VehicleUnderAnalysisMaxMass, ...
                           collisionSeverity.bound_type_str, ...
                           TargetSeverity, ...
                           BulletSeverity, ...
                           deltaV_target, ...
                           deltaV_bullet), ...
                   'Collision Severity');

            % Create the formatted report text
            reportText = sprintf(['\nCollision Severity Inputs:\n', ...
                                 '- Collision Type for Vehicle 1: %s\n', ...
                                 '- Collision Type for Vehicle 2: %s\n', ...
                                 '- Initial Speed of Target: %.2f kph\n', ...
                                 '- Initial Speed of Bullet: %.2f kph\n', ...
                                 '- Average J2980 Vehicle Mass: %.2f kg\n', ...
                                 '- Average Vehicle Under Analysis Mass: %.2f kg\n', ...
                                 '- Bound Type: %s\n\n', ...
                                 'Collision Severity Results:\n', ...
                                 '- Target Severity: %s\n', ...
                                 '- Bullet Severity: %s\n', ...
                                 '- Target Delta-V: %.2f kph\n', ...
                                 '- Bullet Delta-V: %.2f kph\n'], ...
                                collisionType_vehicle1, ...
                                collisionType_vehicle2, ...
                                initialSpeed_target, ...
                                initialSpeed_bullet, ...
                                collisionSeverity.J2980AssumedMaxMass, ...
                                collisionSeverity.VehicleUnderAnalysisMaxMass, ...
                                collisionSeverity.bound_type_str, ...
                                TargetSeverity, ...
                                BulletSeverity, ...
                                deltaV_target, ...
                                deltaV_bullet);
        
            % Define the filename with a timestamp to ensure uniqueness
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            filename = sprintf('collision_severity_report_%s.txt', timestamp);
        
            % Attempt to open the file for writing
            fileID = fopen(filename, 'w');
            if fileID == -1
                uialert(obj.uiManager.fig, 'Failed to open file for writing.', 'File Error');
                return;
            end
        
            % Write the report text to the file
            fprintf(fileID, '%s', reportText);
        
            % Close the file
            fclose(fileID);
        
            % Optional: Notify the user that the report was saved successfully
            uialert(obj.uiManager.fig, sprintf('Collision severity report saved to "%s".', filename), 'Report Saved');

            % Equal axis scaling
            axis(obj.sharedAx, 'equal');

            % Add Padding to Y-Axis
            padding = 0.8; % 20% padding
            ylim(obj.sharedAx, [0 - padding, 1000 + padding]);
        end

        %% Plot Trajectories (Up to stepsToPlot)
        function plotTrajectories(obj, dataManager, stepsToPlot, simParams1, simParams2)
            % Updates the line data for both vehicles and their trailers up to stepsToPlot

            % Vehicle 1:
            Xv1 = dataManager.globalVehicle1Data.X(1:stepsToPlot);
            Yv1 = dataManager.globalVehicle1Data.Y(1:stepsToPlot);
            set(obj.veh1Line, 'XData', Xv1, 'YData', Yv1);

            if ~isempty(dataManager.globalTrailer1Data.X)
                Xtrl1 = dataManager.globalTrailer1Data.X(1:stepsToPlot);
                Ytrl1 = dataManager.globalTrailer1Data.Y(1:stepsToPlot);
                set(obj.trl1Line, 'XData', Xtrl1, 'YData', Ytrl1);

                rearX_Trailer1 = Xtrl1 - (simParams1.trailerLength / 2) .* ...
                    cos(dataManager.globalTrailer1Data.Theta(1:stepsToPlot));
                rearY_Trailer1 = Ytrl1 - (simParams1.trailerLength / 2) .* ...
                    sin(dataManager.globalTrailer1Data.Theta(1:stepsToPlot));
                set(obj.trl1RearLine, 'XData', rearX_Trailer1, 'YData', rearY_Trailer1);
            else
                % If no trailer, hide lines
                set(obj.trl1Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl1RearLine, 'XData', NaN, 'YData', NaN);
            end

            % Vehicle 2:
            Xv2 = dataManager.globalVehicle2Data.X(1:stepsToPlot);
            Yv2 = dataManager.globalVehicle2Data.Y(1:stepsToPlot);
            set(obj.veh2Line, 'XData', Xv2, 'YData', Yv2);

            if ~isempty(dataManager.globalTrailer2Data.X)
                Xtrl2 = dataManager.globalTrailer2Data.X(1:stepsToPlot);
                Ytrl2 = dataManager.globalTrailer2Data.Y(1:stepsToPlot);
                set(obj.trl2Line, 'XData', Xtrl2, 'YData', Ytrl2);

                rearX_Trailer2 = Xtrl2 - (simParams2.trailerLength / 2) .* ...
                    cos(dataManager.globalTrailer2Data.Theta(1:stepsToPlot));
                rearY_Trailer2 = Ytrl2 - (simParams2.trailerLength / 2) .* ...
                    sin(dataManager.globalTrailer2Data.Theta(1:stepsToPlot));
                set(obj.trl2RearLine, 'XData', rearX_Trailer2, 'YData', rearY_Trailer2);
            else
                % If no trailer, hide lines
                set(obj.trl2Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl2RearLine, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Determine Collision Type
        function collisionType = determineCollisionType(~, n_vector, headingVector)
            % (No change to logic)
            dotProduct = dot(n_vector(1:2), headingVector(1:2));
            dotProduct = max(min(dotProduct, 1), -1);
            angle = acosd(dotProduct);

            if angle < 30
                collisionType = 'Rear-End Collision';
            elseif angle > 150
                collisionType = 'Head-On Collision';
            elseif angle >= 60 && angle <= 120
                collisionType = 'Side Collision';
            else
                collisionType = 'Oblique Collision';
            end
        end
    end
end
