%{
% @file VehiclePlotter.m
% @brief Static utilities for drawing vehicle shapes in plots.
%        Provides helper functions for geometry calculations.
% @author Miguel Marina
%}
%/**
% * @class VehiclePlotter
% * @brief Provides static methods for plotting vehicle components in simulations.
% *
% * The VehiclePlotter class offers static utility functions to compute vehicle geometries and render them
% * within simulation plots. It handles the calculation of vehicle corners, plotting of vehicles, axles, wheels,
% * hitch joints, and centers of gravity (CoG). This class is instrumental in visualizing the spatial
% * configurations and movements of different vehicle types within the simulation environment.
% *
% * @author Miguel Marina
% * @version 1.0
% * @date 2024-10-04
% */
classdef VehiclePlotter
    methods (Static)
        %/**
        % * @brief Computes the four corners of a vehicle based on its position and orientation.
        % *
        % * This method calculates the (X, Y) coordinates of the vehicle's four corners, taking into account
        % * whether the vehicle is a tractor, passenger vehicle, or trailer. It considers the vehicle's
        % * dimensions, steering angle, and number of tires per axle to accurately represent its geometry.
        % *
        % * @param x Position of the vehicle along the X-axis (center for tractor/passenger, hitch point for trailer).
        % * @param y Position of the vehicle along the Y-axis (center for tractor/passenger, hitch point for trailer).
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param vehicleParams Structure containing vehicle parameters such as length, width, wheel dimensions, wheelbase, and number of axles.
        % * @param isTractor Boolean indicating if the vehicle is a tractor.
        % * @param isPassengerVehicle Boolean indicating if the vehicle is a passenger vehicle.
        % * @param steeringWheelAngle Steering angle of the vehicle's wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % *
        % * @return corners 4x2 matrix containing the (X, Y) coordinates of the vehicle's rectangle corners.
        % *
        % * @throws Error if "trailerHitchDistance" is not less than "length" for trailers.
        % */
        function corners = getVehicleCorners(x, y, theta, vehicleParams, isTractor, steeringWheelAngle, numTiresPerAxle)
            % getVehicleCorners computes the four corners of a vehicle.
            %
            % Inputs:
            %   x, y - Position of the vehicle (center for tractor/passenger, hitch point for trailer)
            %   theta - Orientation angle in radians
            %   vehicleParams - Struct containing vehicle parameters
            %   isTrailer - Boolean indicating if the vehicle is a tractor
            %   isTractor - Boolean indicating if the vehicle is a passenger vehicle
            %   steeringWheelAngle - Steering angle in degrees
            %   numTiresPerAxle - Number of tires per axle
            %
            % Output:
            %   corners - 4x2 matrix containing the (X, Y) coordinates of the rectangle corners

            % Extract parameters from vehicleParams
            length = vehicleParams.length;
            width = vehicleParams.width;
            wheelWidth = vehicleParams.wheelWidth;
            wheelHeight = vehicleParams.wheelHeight;
            wheelbase = vehicleParams.wheelbase;

            % === Hardcode trailerHitchDistance to 2.620 meters ===
            trailerHitchDistance = 1.310 * 2;
            % Calculate the remaining length of the trailer
            remainingLength = length - trailerHitchDistance;

            % Initialize localCorners based on vehicle type
            if ~isTractor
                % === Trailer Corner Calculation ===

                % Validate remainingLength
                if remainingLength <= 0
                    error('For trailers, "trailerHitchDistance" must be less than "length".');
                end

                % Define the rectangle corners in local trailer coordinates
                % Starting from hitch position (0,0), extend backward along X-axis by remainingLength
                % Order: front-left (hitch), front-right, back-right, back-left
                localCorners = [trailerHitchDistance, -remainingLength, -remainingLength, trailerHitchDistance;
                                -width/2, -width/2, width/2, width/2];
            else
                % === Tractor or Passenger Vehicle Corner Calculation ===
                % Front of the vehicle is at (0, 0), extend back by 'length' along X-axis
                % Order: front-left, front-right, back-right, back-left
                localCorners = [0, -length, -length, 0;
                                -width/2, -width/2, width/2, width/2];
            end

            % Rotate the corners
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedCorners = R * localCorners;

            % Translate the corners to the vehicle position
            translatedCornersX = rotatedCorners(1, :) + x;
            translatedCornersY = rotatedCorners(2, :) + y;

            % Combine X and Y into a single matrix
            corners = [translatedCornersX; translatedCornersY]';

            % === Input Validation ===
            if size(corners, 1) ~= 4 || size(corners, 2) ~= 2
                error('getVehicleCorners:InvalidOutput', 'The corners matrix must be a 4x2 matrix of (X, Y) coordinates.');
            end
        end

        %/**
        % * @brief Plots the vehicle on the given axes.
        % *
        % * This method renders the vehicle's body, axles, wheels, and other components based on its position,
        % * orientation, and parameters. It differentiates between tractors, passenger vehicles, and trailers
        % * to accurately depict their respective geometries.
        % *
        % * @param ax Handle to the axes where the vehicle will be plotted.
        % * @param x Position of the vehicle along the X-axis.
        % * @param y Position of the vehicle along the Y-axis.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param vehicleParams Structure containing vehicle parameters such as length, width, wheel dimensions, wheelbase, and number of axles.
        % * @param color Color code for the vehicle plot (e.g., 'r' for red).
        % * @param isTractor Boolean indicating if the vehicle is a tractor.
        % * @param isPassengerVehicle Boolean indicating if the vehicle is a passenger vehicle.
        % * @param steeringWheelAngle Steering angle of the vehicle's wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % * @param numAxles Number of axles on the vehicle.
        % *
        % * @return None
        % */
        function graphics = plotVehicle(ax, x, y, theta, vehicleParams, color, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles)
            % Compute geometry for all components
            geom = VehiclePlotter.computeVehicleGeometry(x, y, theta, vehicleParams, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles);

            % Plot body
            graphics.body = plot(ax, geom.body(1, :), geom.body(2, :), color, 'LineWidth', 2);

            % Plot axles and wheels
            nAxles = numel(geom.axles);
            graphics.axles = gobjects(1, nAxles);
            graphics.wheels = gobjects(1, numel(geom.wheels));

            wheelIdx = 1;
            for a = 1:nAxles
                graphics.axles(a) = plot(ax, geom.axles{a}(1, :), geom.axles{a}(2, :), 'k', 'LineWidth', 2);
                wheelPolys = geom.wheels{a};
                for w = 1:numel(wheelPolys)
                    graphics.wheels(wheelIdx) = fill(ax, wheelPolys{w}(1, :), wheelPolys{w}(2, :), 'k');
                    wheelIdx = wheelIdx + 1;
                end
            end
        end

        %% Update an existing vehicle graphic with new pose
        function updateVehicle(graphics, x, y, theta, vehicleParams, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles)
            if isempty(graphics) || ~isgraphics(graphics.body)
                return;
            end

            geom = VehiclePlotter.computeVehicleGeometry(x, y, theta, vehicleParams, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles);
            set(graphics.body, 'XData', geom.body(1, :), 'YData', geom.body(2, :));

            wheelIdx = 1;
            for a = 1:numel(geom.axles)
                if isgraphics(graphics.axles(a))
                    set(graphics.axles(a), 'XData', geom.axles{a}(1, :), 'YData', geom.axles{a}(2, :));
                end
                wheelPolys = geom.wheels{a};
                for w = 1:numel(wheelPolys)
                    if isgraphics(graphics.wheels(wheelIdx))
                        set(graphics.wheels(wheelIdx), 'XData', wheelPolys{w}(1, :), 'YData', wheelPolys{w}(2, :));
                    end
                    wheelIdx = wheelIdx + 1;
                end
            end
        end

        %% Compute geometry for body, axles, and wheels without plotting
        function geom = computeVehicleGeometry(x, y, theta, vehicleParams, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles)
            length = vehicleParams.length;
            width = vehicleParams.width;
            wheelWidth = vehicleParams.wheelHeight;
            wheelHeight = vehicleParams.wheelWidth;
            wheelbase = vehicleParams.wheelbase;

            if (~isTractor && ~isPassengerVehicle)
                trailerHitchDistance = vehicleParams.HitchDistance;
                remainingLength = length - trailerHitchDistance;
                localCorners = [trailerHitchDistance, -remainingLength, -remainingLength, trailerHitchDistance, trailerHitchDistance;
                                -width/2, -width/2, width/2, width/2, -width/2];
                baseOffset = -remainingLength/2;
            else
                localCorners = [0, -length, -length, 0, 0;
                                -width/2, -width/2, width/2, width/2, -width/2];
                baseOffset = -length/2;
            end

            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotated = R * localCorners;
            geom.body = [rotated(1, :) + x; rotated(2, :) + y];

            geom.axles = cell(1, numAxles + (isTractor || isPassengerVehicle));
            geom.wheels = cell(1, numAxles + (isTractor || isPassengerVehicle));

            axleIdx = 1;
            function [axleXY, wheelPolys] = axleGeom(axlePos, steer)
                axleX = [-width/2, width/2];
                axleY = [0, 0];
                axlePosX = x + axlePos * cos(theta);
                axlePosY = y + axlePos * sin(theta);
                Rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                axleEnds = Rot * [axleX; axleY] + [axlePosX; axlePosY];
                cX = mean(axleEnds(1, :));
                cY = mean(axleEnds(2, :));
                rot90 = [0 -1; 1 0];
                axleXY = rot90 * ([axleEnds(1, :) - cX; axleEnds(2, :) - cY]) + [cX; cY];

                M = 0.5;
                phi = deg2rad(steer);
                offsetAngle = theta + phi + pi/2;
                d = ((width - vehicleParams.trackWidth) / 2);
                leftBase = axleXY(:,1) + d * [cos(offsetAngle); sin(offsetAngle)];
                rightBase = axleXY(:,2) - d * [cos(offsetAngle); sin(offsetAngle)];

                rect = [-wheelWidth/2, wheelWidth/2, wheelWidth/2, -wheelWidth/2, -wheelWidth/2;
                        -wheelHeight/2, -wheelHeight/2, wheelHeight/2, wheelHeight/2, -wheelHeight/2];
                Rw = [cos(theta + phi), -sin(theta + phi); sin(theta + phi), cos(theta + phi)];

                wheelPolys = {};
                wheelPolys{end+1} = Rw * rect + leftBase;
                wheelPolys{end+1} = Rw * rect + rightBase;
                if numTiresPerAxle > 2
                    wheelPolys{end+1} = Rw * rect + leftBase + M * [cos(offsetAngle); sin(offsetAngle)];
                    wheelPolys{end+1} = Rw * rect + rightBase - M * [cos(offsetAngle); sin(offsetAngle)];
                end
            end

            if isTractor
                if numAxles >= 1
                    [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + vehicleParams.axleSpacing, 0);
                    axleIdx = axleIdx + 1;
                end
                if numAxles >= 2
                    [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + 2*vehicleParams.axleSpacing, 0);
                    axleIdx = axleIdx + 1;
                end
                [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + vehicleParams.axleSpacing + wheelbase, steeringWheelAngle);
                axleIdx = axleIdx + 1;
            elseif isPassengerVehicle
                [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + vehicleParams.axleSpacing, 0);
                axleIdx = axleIdx + 1;
                [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + vehicleParams.axleSpacing + wheelbase, steeringWheelAngle);
                axleIdx = axleIdx + 1;
            else
                for ax = 1:numAxles
                    [geom.axles{axleIdx}, geom.wheels{axleIdx}] = axleGeom(baseOffset + ax*vehicleParams.axleSpacing, 0);
                    axleIdx = axleIdx + 1;
                end
            end

            geom.axles = geom.axles(1:axleIdx-1);
            geom.wheels = geom.wheels(1:axleIdx-1);
        end

        %/**
        % * @brief Plots the axle and associated wheels on the given axes.
        % *
        % * This helper method renders an axle line and the corresponding wheels based on the vehicle's orientation
        % * and position. It supports vehicles with different numbers of tires per axle.
        % *
        % * @param ax Handle to the axes where the axle and wheels will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param axlePos Position of the axle relative to the vehicle's reference point.
        % * @param width Width of the vehicle.
        % * @param wheelHeight Height of the wheels.
        % * @param wheelWidth Width of the wheels.
        % * @param steeringWheelAngle Steering angle of the wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % *
        % * @return None
        % */
        function h = plotAxleAndWheels(ax, x, y, theta, axlePos, width, wheelWidth, wheelHeight, steeringWheelAngle, numTiresPerAxle, trackWidth)
            % Define the axle line
            axleX = [-width/2, width/2];
            axleY = [0, 0];

            % Position the axle
            axlePosX = x + axlePos * cos(theta);
            axlePosY = y + axlePos * sin(theta);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            axleEnds = R * [axleX; axleY] + [axlePosX; axlePosY];

            % Rotate the entire axle 90 degrees counterclockwise around its center
            centerX = mean(axleEnds(1, :));
            centerY = mean(axleEnds(2, :));
            rotationMatrix = [cosd(90), -sind(90); sind(90), cosd(90)];
            rotatedAxleEnds = rotationMatrix * ([axleEnds(1, :) - centerX; axleEnds(2, :) - centerY]) + [centerX; centerY];

            % Plot the axle and store the handle
            hAxle = plot(ax, rotatedAxleEnds(1, :), rotatedAxleEnds(2, :), 'k', 'LineWidth', 2);
            wheelHandles = [];

            % Define the offset distance M
            M = 0.5; % Example offset distance, adjust as needed

            % Convert steering angle from degrees to radians
            phi = deg2rad(steeringWheelAngle);

            % Calculate the combined angle for offset
            offsetAngle = theta + phi + pi/2; % Adding 90 degrees in radians

            % Calculate the offset components for both sides
            offsetX_positive = M * cos(offsetAngle);
            offsetY_positive = M * sin(offsetAngle);

            offsetX_negative = -M * cos(offsetAngle);
            offsetY_negative = -M * sin(offsetAngle);

            if numTiresPerAxle == 2
                % Plot left tire with positive offset
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 1) + (((width - trackWidth) / 2) * cos(offsetAngle)), ...
                    rotatedAxleEnds(2, 1) + (((width - trackWidth) / 2) * sin(offsetAngle)), ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');

                % Plot right tire with negative offset
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 2) - (((width - trackWidth) / 2) * cos(offsetAngle)), ...
                    rotatedAxleEnds(2, 2) - (((width - trackWidth) / 2) * sin(offsetAngle)), ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');
            else
                % Handle cases with more than 2 tires per axle if necessary
                % For example, duplicate the above two plots or adjust as needed
                % Example for 4 tires:
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 1) + (((width - trackWidth) / 2) * cos(offsetAngle)),...
                    rotatedAxleEnds(2, 1) + (((width - trackWidth) / 2) * sin(offsetAngle)), ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 2) - (((width - trackWidth) / 2) * cos(offsetAngle)), ...
                    rotatedAxleEnds(2, 2) - (((width - trackWidth) / 2) * sin(offsetAngle)), ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 1) + (((width - trackWidth) / 2) * cos(offsetAngle)) + offsetX_positive, ...
                    rotatedAxleEnds(2, 1) + (((width - trackWidth) / 2) * sin(offsetAngle)) + offsetY_positive, ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rotatedAxleEnds(1, 2) - (((width - trackWidth) / 2) * cos(offsetAngle)) + offsetX_negative, ...
                    rotatedAxleEnds(2, 2) - (((width - trackWidth) / 2) * sin(offsetAngle)) + offsetY_negative, ...
                    wheelWidth, ...
                    wheelHeight, ...
                    theta + phi, ...
                    'k');
            end

            % Set axis equal for proper visualization
            axis equal;
            h = [hAxle, wheelHandles];
        end

        %/**
        % * @brief Plots an individual wheel on the given axes.
        % *
        % * This helper method renders a wheel as a filled rectangle based on its position, dimensions, and orientation.
        % *
        % * @param ax Handle to the axes where the wheel will be plotted.
        % * @param x X-coordinate of the wheel's center.
        % * @param y Y-coordinate of the wheel's center.
        % * @param wheelWidth Width of the wheel.
        % * @param wheelHeight Height of the wheel.
        % * @param theta Orientation angle of the wheel in radians.
        % * @param color Color code for the wheel fill.
        % *
        % * @return None
        % */
        function h = plotWheel(ax, x, y, wheelWidth, wheelHeight, theta, color)
            % Define a rectangle for the wheel
            wheelX = [-wheelWidth/2, wheelWidth/2, wheelWidth/2, -wheelWidth/2, -wheelWidth/2];
            wheelY = [-wheelHeight/2, -wheelHeight/2, wheelHeight/2, wheelHeight/2, -wheelHeight/2];

            % Rotate the wheel
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedWheel = R * [wheelX; wheelY];

            % Translate the wheel
            translatedWheelX = rotatedWheel(1, :) + x;
            translatedWheelY = rotatedWheel(2, :) + y;

            % Plot the wheel and return the handle for easier updates
            h = fill(ax, translatedWheelX, translatedWheelY, color);
        end

        %/**
        % * @brief Plots the hitch joint on the given axes.
        % *
        % * Marks the hitch joint point where a trailer is attached to a tractor or another vehicle.
        % *
        % * @param ax Handle to the axes where the hitch joint will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param length Length of the vehicle.
        % * @param color Color code for the hitch joint marker.
        % *
        % * @return None
        % */
        function plotHitchJoint(ax, x, y, theta, length, color)
            % Calculate the hitch joint position
            hitchX = x - length/2 * cos(theta);
            hitchY = y - length/2 * sin(theta);

            % Plot the hitch joint
            plot(ax, hitchX, hitchY, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
        end

        %/**
        % * @brief Plots the center of gravity (CoG) on the given axes.
        % *
        % * This method marks the CoG of the vehicle with distinct markers for better visualization.
        % *
        % * @param ax Handle to the axes where the CoG will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param cogX X-offset of the CoG from the vehicle's reference point.
        % * @param cogY Y-offset of the CoG from the vehicle's reference point.
        % * @param color Color code for the CoG markers.
        % *
        % * @return None
        % */
        function plotCoG(ax, x, y, cogX, cogY, color)
            % Calculate the global position of the CoG
            globalX = x + cogX;
            globalY = y + cogY;

            % Plot the CoG as a distinct marker
            plot(ax, globalX, globalY, 'x', 'Color', color, 'MarkerSize', 10, 'LineWidth', 2);
            % hold(ax, 'on');
            plot(ax, globalX, globalY, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
            % hold(ax, 'off');
        end
    end
end
