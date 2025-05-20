%/**
% * @class UIManager
% * @brief Handles the creation and management of the simulation UI.
% *
% * The UIManager class is responsible for constructing and managing the user interface of the simulation.
% * It creates various UI panels, tabs, buttons, and menu items, and assigns callback functions to handle
% * user interactions. The class ensures that all UI components are properly initialized and interact seamlessly
% * with other modules such as ConfigurationManager, DataManager, and PlotManager.
% *
% * @author Miguel Marina
% * @version 1.5
% * @date 2024-12-21
% */
classdef UIManager < handle
    properties
        fig

        % New figure for vehicle configurations
        configFig

        % Configuration Tab Group (Now in a separate figure)
        tabGroup

        % Vehicle Configuration Tabs
        vehicleTab1
        vehicleTab2

        plotPanel
        sharedAx

        offsetPanel
        vehicle1OffsetXField
        vehicle1OffsetYField
        rotateVehicle1Checkbox
        vehicle1RotationAngleField
        offsetXField
        offsetYField
        rotateVehicle2Checkbox
        rotationAngleField

        severityPanel
        boundTypeDropdown
        vehicle2MassField
        vehicle1MassField

        buttonPanel
        loadVehicleButton
        loadVehicle1Button
        startSimulationButton

        saveConfigMenu
        loadConfigMenu
        saveResultsMenu
        savePlotMenu
        saveManualControlMenu

        Callbacks

        % UI Components for Vehicle 1
        vehicle1LengthField
        vehicle1WidthField
        vehicle1HeightField

        % UI Components for Vehicle 2
        vehicle2LengthField
        vehicle2WidthField
        vehicle2HeightField

        % Lane Map Commands
        laneCommandsLabel
        laneCommandsField
        buildMapButton

        pathCommandsPanel
        pathCommandsField
        generateWaypointsButton
        waypointsPanel
        waypointsTable
    end

    methods
        function obj = UIManager(callbacks)
            if nargin < 1
                callbacks = struct();
            end
            obj.Callbacks = callbacks;
            obj.createUI();
        end

        function createUI(obj)
            % Create the main UI figure with a modern color scheme and responsive layout
            obj.fig = uifigure('Name', 'Vehicle Simulations', ...
                'Position', [100, 100, 1600, 900], ...
                'Color', [0.95, 0.95, 0.95], ... % Light grey background
                'DefaultUicontrolFontName', 'Segoe UI', ...
                'DefaultUicontrolFontSize', 12, ...
                'KeyPressFcn', @(src, event) obj.keyPressHandler(src, event));

            % Create Menu Bar in the main figure
            obj.createMenu();

            % Create a separate figure for vehicle configurations
            obj.createVehicleConfigGUI();

            % Create Main Layout using Grid
            mainLayout = uigridlayout(obj.fig, [2, 2], ...
                'ColumnWidth', {'1x', 300}, 'RowHeight', {'1x', 200});
            mainLayout.Padding = 10;
            mainLayout.RowSpacing = 10;
            mainLayout.ColumnSpacing = 10;

            % Create Plotting Area
            obj.createPlotPanel(mainLayout);

            % Create Offset Configuration Panel
            obj.createOffsetPanel(mainLayout);

            % Create Severity Configuration Panel
            obj.createSeverityPanel(mainLayout);

            % Create Button Panel in the main figure
            obj.createButtonPanel(mainLayout);
        end

        function createVehicleConfigGUI(obj)
            % Create a separate figure for Vehicle Configurations with modern styling
            obj.configFig = uifigure('Name','Vehicles Configuration',...
                'Position',[800,100,400,700], ...
                'Color', [0.95, 0.95, 0.95], ...
                'DefaultUicontrolFontName', 'Segoe UI', ...
                'DefaultUicontrolFontSize', 12);

            % Create Tab Group in the separate configuration figure
            obj.tabGroup = uitabgroup(obj.configFig, 'Position', [10, 10, 380, 680]);

            % Vehicle 1 Tab
            obj.vehicleTab1 = uitab(obj.tabGroup, 'Title', 'Vehicle 1 Configuration');
            obj.createVehicle1Config(obj.vehicleTab1);

            % Vehicle 2 Tab
            obj.vehicleTab2 = uitab(obj.tabGroup, 'Title', 'Vehicle 2 Configuration');
            obj.createVehicle2VehicleConfig(obj.vehicleTab2);
        end

        function createVehicle1Config(obj, parent)
            % Create a grid layout for better alignment
            grid = uigridlayout(parent, [5, 2], ...
                'ColumnWidth', {150, '1x'}, 'RowHeight', {30, 30, 30, 30, 30});
            grid.Padding = [10, 10, 10, 10];
            grid.RowSpacing = 10;
            grid.ColumnSpacing = 10;

            % % Vehicle 1 Length
            % uilabel(grid, 'Text', 'Length (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle1LengthField = uieditfield(grid, 'numeric', 'Value', 5.0);
            % 
            % % Vehicle 1 Width
            % uilabel(grid, 'Text', 'Width (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle1WidthField = uieditfield(grid, 'numeric', 'Value', 2.5);
            % 
            % % Vehicle 1 Height
            % uilabel(grid, 'Text', 'Height (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle1HeightField = uieditfield(grid, 'numeric', 'Value', 3.0);
            % 
            % % Initial Position X
            % uilabel(grid, 'Text', 'Initial Position X (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle1OffsetXField = uieditfield(grid, 'numeric', 'Value', 0);
            % 
            % % Initial Position Y
            % uilabel(grid, 'Text', 'Initial Position Y (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle1OffsetYField = uieditfield(grid, 'numeric', 'Value', 0);

            % Rotate Vehicle 1 Checkbox
            obj.rotateVehicle1Checkbox = uicheckbox(parent, ...
                'Position', [10, 100, 200, 30], ...
                'Text', 'Rotate Vehicle 1', ...
                'Value', false, ...
                'FontWeight', 'bold', ...
                'ValueChangedFcn', @(src, event) obj.toggleVehicle1RotationOptions(src));

            % Rotation Angle Field
            uilabel(parent, 'Position', [10, 60, 150, 30], ...
                'Text', 'Rotation Angle (°):', ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            obj.vehicle1RotationAngleField = uieditfield(parent, 'numeric', ...
                'Position', [160, 60, 120, 30], ...
                'Limits', [-360, 360], ...
                'Value', 0, ...
                'Enable', 'off');
        end

        function createVehicle2VehicleConfig(obj, parent)
            % Create a grid layout for better alignment
            grid = uigridlayout(parent, [5, 2], ...
                'ColumnWidth', {150, '1x'}, 'RowHeight', {30, 30, 30, 30, 30});
            grid.Padding = [10, 10, 10, 10];
            grid.RowSpacing = 10;
            grid.ColumnSpacing = 10;

            % % Vehicle 2 Length
            % uilabel(grid, 'Text', 'Length (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle2LengthField = uieditfield(grid, 'numeric', 'Value', 4.5);
            % 
            % % Vehicle 2 Width
            % uilabel(grid, 'Text', 'Width (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle2WidthField = uieditfield(grid, 'numeric', 'Value', 2.0);
            % 
            % % Vehicle 2 Height
            % uilabel(grid, 'Text', 'Height (m):', 'HorizontalAlignment', 'right');
            % obj.vehicle2HeightField = uieditfield(grid, 'numeric', 'Value', 2.8);
            % 
            % % Initial Position X
            % uilabel(grid, 'Text', 'Initial Position X (m):', 'HorizontalAlignment', 'right');
            % obj.offsetXField = uieditfield(grid, 'numeric', 'Value', -370);
            % 
            % % Initial Position Y
            % uilabel(grid, 'Text', 'Initial Position Y (m):', 'HorizontalAlignment', 'right');
            % obj.offsetYField = uieditfield(grid, 'numeric', 'Value', -3);

            % Rotate Vehicle 2 Checkbox
            obj.rotateVehicle2Checkbox = uicheckbox(parent, ...
                'Position', [10, 100, 200, 30], ...
                'Text', 'Rotate Vehicle 2', ...
                'Value', false, ...
                'FontWeight', 'bold', ...
                'ValueChangedFcn', @(src, event) obj.toggleVehicle2RotationOptions(src));

            % Rotation Angle Field
            uilabel(parent, 'Position', [10, 60, 150, 30], ...
                'Text', 'Rotation Angle (°):', ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            obj.rotationAngleField = uieditfield(parent, 'numeric', ...
                'Position', [160, 60, 120, 30], ...
                'Limits', [-360, 360], ...
                'Value', 0, ...
                'Enable', 'off');
        end

        function createWaypointGUI(obj)
            % Create Waypoint Generation GUI as a separate figure with modern styling
            f = uifigure('Name','Waypoint Generation','Position',[100,100,800,600], ...
                'Color', [0.95, 0.95, 0.95], ...
                'DefaultUicontrolFontName', 'Segoe UI', ...
                'DefaultUicontrolFontSize', 12);

            % Main Grid Layout
            mainGrid = uigridlayout(f, [2, 1], ...
                'RowHeight', {'1x', '1x'}, 'ColumnWidth', '1x');
            mainGrid.Padding = 10;
            mainGrid.RowSpacing = 20;

            % Commands Panel
            commandsPanel = uipanel('Parent', mainGrid, 'Title', 'Path Commands', ...
                'BackgroundColor', [1 1 1], 'Padding', 10);

            commandsGrid = uigridlayout(commandsPanel, [3, 2], ...
                'ColumnWidth', {150, '1x'}, 'RowHeight', {30, 30, 40});
            commandsGrid.ColumnSpacing = 10;
            commandsGrid.RowSpacing = 10;

            uilabel(commandsGrid, 'Text', 'Enter Path String:', 'HorizontalAlignment', 'right');
            commandsEdit = uieditfield(commandsGrid, 'text', ...
                'Value', 'straight(100,200,300,200)|curve(300,250,50,270,90,ccw)|straight(300,300,100,300)|curve(100,250,50,90,270,ccw)', ...
                'Layout', [1, 2]);

            uilabel(commandsGrid, 'Text', 'Resolution:', 'HorizontalAlignment', 'right');
            resolutionEdit = uieditfield(commandsGrid, 'numeric', ...
                'Value', 0.05, ...
                'Limits', [0.01, Inf]);

            % Generate Waypoints Button without Icon to prevent errors
            genButton = uibutton(commandsGrid, 'push', ...
                'Text', 'Generate Waypoints', ...
                'ButtonPushedFcn', @(btn, event)generateWaypointsCallback(), ...
                'Layout', [3, 2]);

            % Waypoints Panel
            waypointsPanel = uipanel('Parent', mainGrid, 'Title', 'Generated Waypoints', ...
                'BackgroundColor', [1 1 1], 'Padding', 10);

            obj.waypointsTable = uitable(waypointsPanel, ...
                'ColumnName', {'X', 'Y'}, ...
                'RowName', [], ...
                'Position', [10, 10, 760, 400]);

            % Callback Function for Generating Waypoints
            function generateWaypointsCallback()
                pathString = commandsEdit.Value;
                resolution = resolutionEdit.Value;
                if isempty(pathString)
                    uialert(f, 'Path string cannot be empty.', 'Input Error');
                    return;
                end
                if isnan(resolution) || resolution <= 0
                    uialert(f, 'Please enter a positive numeric value for resolution.', 'Invalid Resolution');
                    return;
                end

                try
                    wp = generateWaypoints(pathString, resolution);
                    if isempty(wp)
                        uialert(f, 'No waypoints generated. Check the path commands.', 'No Data');
                    else
                        obj.waypointsTable.Data = wp;

                        [rows, cols] = size(wp);
                        formatSpec = [repmat('%g, ', 1, cols-1), '%g\n'];
                        waypointStr = sprintf(formatSpec, wp.');

                        waypointDisplayFig = uifigure('Name', 'Waypoints', ...
                                   'NumberTitle', 'off', ...
                                   'MenuBar', 'none', ...
                                   'ToolBar', 'none', ...
                                   'Position', [300 300 400 300], ...
                                   'Color', [1 1 1]);
                        uieditfield(waypointDisplayFig, 'text', ...
                                  'Value', waypointStr, ...
                                  'Editable', 'off', ...
                                  'Position', [20 20 360 260], ...
                                  'HorizontalAlignment', 'left');
                    end
                catch ME
                    uialert(f, ['Error generating waypoints: ' ME.message], 'Error');
                end
            end
        end

        function createMenu(obj)
            mainMenu = uimenu(obj.fig, 'Text', 'File'); % Removed 'FontWeight'

            % Save Configuration Menu
            obj.saveConfigMenu = uimenu(mainMenu, 'Text', 'Save Configuration', ...
                'Accelerator', 'S', ...
                'MenuSelectedFcn', @(src, event) obj.Callbacks.SaveConfiguration());

            % Load Configuration Menu
            obj.loadConfigMenu = uimenu(mainMenu, 'Text', 'Load Configuration', ...
                'Accelerator', 'L', ...
                'MenuSelectedFcn', @(src, event) obj.Callbacks.LoadConfiguration());

            % Save Simulation Results
            obj.saveResultsMenu = uimenu(mainMenu, 'Text', 'Save Simulation Results', ...
                'Accelerator', 'R', ...
                'MenuSelectedFcn', @(src, event) obj.Callbacks.SaveSimulationResults());

            % Save Plot as PNG
            obj.savePlotMenu = uimenu(mainMenu, 'Text', 'Save Plot as PNG', ...
                'Accelerator', 'P', ...
                'MenuSelectedFcn', @(src, event) obj.Callbacks.SavePlotAsPNG());

            % Save Manual Control Data
            obj.saveManualControlMenu = uimenu(mainMenu, 'Text', 'Save Manual Control Data', ...
                'Accelerator', 'M', ...
                'MenuSelectedFcn', @(src, event) obj.Callbacks.SaveManualControlData());
        end

        %/**
        % * @brief Creates the plotting area panel with shared axes.
        % */
        function createPlotPanel(obj, parent)
            % Create Plot Panel within the specified parent layout
            obj.plotPanel = uipanel(parent, 'Title', 'Plotting Area', ...
                'BackgroundColor', [1 1 1]);

            plotGrid = uigridlayout(obj.plotPanel, [1,1], ...
                'RowHeight', {'1x'}, 'ColumnWidth', {'1x'}); % Changed to cell arrays
            plotGrid.Padding = 10;

            % Add plotting axes
            obj.sharedAx = uiaxes('Parent', plotGrid, 'Color', [1 1 1], 'Box', 'on');
            hold(obj.sharedAx, 'on');
            xlabel(obj.sharedAx, 'X Position (meters)', 'FontWeight', 'bold');
            ylabel(obj.sharedAx, 'Y Position (meters)', 'FontWeight', 'bold');
            title(obj.sharedAx, 'Simulation Plot', 'FontWeight', 'bold');
            axis(obj.sharedAx, 'equal');
            grid(obj.sharedAx, 'on');
            obj.sharedAx.FontName = 'Segoe UI';
            obj.sharedAx.FontSize = 12;
        end

        function createOffsetPanel(obj, parent)
            % Create Offset Configuration Panel within the specified parent layout
            obj.offsetPanel = uipanel(parent, 'Title', 'Initial Position & Rotation Configurations', ...
                'BackgroundColor', [1 1 1]);

            offsetGrid = uigridlayout(obj.offsetPanel, [4, 2], ...
                'ColumnWidth', {150, '1x'}, 'RowHeight', {30, 30, 30, 30});
            offsetGrid.Padding = [10, 10, 10, 10];
            offsetGrid.RowSpacing = 10;
            offsetGrid.ColumnSpacing = 10;

            % Vehicle 1 Initial Position
            uilabel(offsetGrid, 'Text', 'Vehicle 1 Initial X (m):', 'HorizontalAlignment', 'right');
            obj.vehicle1OffsetXField = uieditfield(offsetGrid, 'numeric', 'Value', -75);

            uilabel(offsetGrid, 'Text', 'Vehicle 1 Initial Y (m):', 'HorizontalAlignment', 'right');
            obj.vehicle1OffsetYField = uieditfield(offsetGrid, 'numeric', 'Value', 200);

            % Rotate Vehicle 1
            obj.rotateVehicle1Checkbox = uicheckbox(obj.offsetPanel, ...
                'Text', 'Rotate Vehicle 1', ...
                'Position', [10, 150, 200, 30], ...
                'Value', false, ...
                'FontWeight', 'bold', ...
                'ValueChangedFcn', @(src, event) obj.toggleVehicle1RotationOptions(src));

            uilabel(obj.offsetPanel, 'Position', [10, 110, 150, 30], ...
                'Text', 'Rotation Angle (°):', ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            obj.vehicle1RotationAngleField = uieditfield(obj.offsetPanel, 'numeric', ...
                'Position', [160, 110, 120, 30], ...
                'Limits', [-360, 360], ...
                'Value', 0, ...
                'Enable', 'off');

            % Vehicle 2 Initial Position
            uilabel(offsetGrid, 'Text', 'Vehicle 2 Initial X (m):', 'HorizontalAlignment', 'right');
            obj.offsetXField = uieditfield(offsetGrid, 'numeric', 'Value', 100);

            uilabel(offsetGrid, 'Text', 'Vehicle 2 Initial Y (m):', 'HorizontalAlignment', 'right');
            obj.offsetYField = uieditfield(offsetGrid, 'numeric', 'Value', 205);

            % Rotate Vehicle 2
            obj.rotateVehicle2Checkbox = uicheckbox(obj.offsetPanel, ...
                'Text', 'Rotate Vehicle 2', ...
                'Position', [10, 70, 200, 30], ...
                'Value', false, ...
                'FontWeight', 'bold', ...
                'ValueChangedFcn', @(src, event) obj.toggleVehicle2RotationOptions(src));

            uilabel(obj.offsetPanel, 'Position', [10, 30, 150, 30], ...
                'Text', 'Rotation Angle (°):', ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            obj.rotationAngleField = uieditfield(obj.offsetPanel, 'numeric', ...
                'Position', [160, 30, 120, 30], ...
                'Limits', [-360, 360], ...
                'Value', 0, ...
                'Enable', 'off');
        end

        function createSeverityPanel(obj, parent)
            % Create Severity Configuration Panel within the specified parent layout
            obj.severityPanel = uipanel(parent, 'Title', 'Severity Configurations', ...
                'BackgroundColor', [1 1 1]);

            severityGrid = uigridlayout(obj.severityPanel, [3, 2], ...
                'ColumnWidth', {200, '1x'}, 'RowHeight', {30, 30, 30});
            severityGrid.Padding = [10, 10, 10, 10];
            severityGrid.RowSpacing = 10;
            severityGrid.ColumnSpacing = 10;

            % Bound Type Dropdown
            uilabel(severityGrid, 'Text', 'Bound Type:', 'HorizontalAlignment', 'right');
            obj.boundTypeDropdown = uidropdown(severityGrid, ...
                'Items', {'HigherBound', 'LowerBound', 'Average'}, ...
                'Value', 'Average');

            % Vehicle 2 Mass
            uilabel(severityGrid, 'Text', 'J2980 Assumed Max Vehicle Mass (kg):', ...
                'HorizontalAlignment', 'right');
            obj.vehicle2MassField = uieditfield(severityGrid, 'numeric', 'Value', 4500);

            % Vehicle 1 Mass
            uilabel(severityGrid, 'Text', 'Max Mass of Vehicles Under Analysis (kg):', ...
                'HorizontalAlignment', 'right');
            obj.vehicle1MassField = uieditfield(severityGrid, 'numeric', 'Value', 36500);
        end

        %/**
        % * @brief Creates the button panel with action buttons and map commands.
        % */
        function createButtonPanel(obj, parent)
            % Create Button Panel within the specified parent layout
            obj.buttonPanel = uipanel(parent, 'Title', '', 'BackgroundColor', [1 1 1]);
        
            % Adjusted layout to have 3 rows and 2 columns
            buttonGrid = uigridlayout(obj.buttonPanel, [3, 2], ...
                'ColumnWidth', {'1x', '1x'}, ...
                'RowHeight', {30, '1x', 50}, ...
                'Padding', [10, 10, 10, 10], ...
                'RowSpacing', 10, ...
                'ColumnSpacing', 20);
        
            % Create the label and set its layout afterwards
            obj.laneCommandsLabel = uilabel(buttonGrid, ...
                'Text', 'Map Commands:', ...
                'HorizontalAlignment', 'left', ...
                'FontWeight', 'bold');
            obj.laneCommandsLabel.Layout.Row = 1;
            obj.laneCommandsLabel.Layout.Column = [1 2];
        
            % Create the textarea and set its layout afterwards
            obj.laneCommandsField = uitextarea(buttonGrid, ...
                'Value', {
                'straight(100,200,300,200)|curve(300,250,50,270,90,ccw)|straight(300,300,100,300)|curve(100,250,50,90,270,ccw)|straight(100,205,300,205)|curve(300,250,45,270,90,ccw)|straight(300,295,100,295)|curve(100,250,45,90,270,ccw)'
                }, ...
                'Editable', 'on');
            obj.laneCommandsField.Layout.Row = 2;
            obj.laneCommandsField.Layout.Column = [1 2];
        
            % Create the buttons and set their layout afterwards
            obj.buildMapButton = uibutton(buttonGrid, 'push', ...
                'Text', 'Build Map', ...
                'ButtonPushedFcn', @(btn, event)obj.buildMapCallback());
            obj.buildMapButton.Layout.Row = 3;
            obj.buildMapButton.Layout.Column = 1;
        
            obj.startSimulationButton = uibutton(buttonGrid, 'push', ...
                'Text', 'Start Simulation', ...
                'ButtonPushedFcn', @(btn, event) obj.Callbacks.StartSimulation());
            obj.startSimulationButton.Layout.Row = 3;
            obj.startSimulationButton.Layout.Column = 2;
        end

        function buildMapCallback(obj)
            cla(obj.sharedAx);
            mapWidth = 600;
            mapHeight = 600;
            map = LaneMap(mapWidth, mapHeight);
            map.LaneCommands = obj.laneCommandsField.Value;
            map.LaneColor = [0 0.4470 0.7410]; % Modern blue color
            map.LaneWidth = 5;
            map.plotLaneMapWithCommands(obj.sharedAx, map.LaneCommands, map.LaneColor);
        end

        function keyPressHandler(obj, src, event)
            % Handle key press events if necessary
        end

        function toggleVehicle1RotationOptions(obj, src)
            if src.Value
                obj.vehicle1RotationAngleField.Enable = 'on';
            else
                obj.vehicle1RotationAngleField.Enable = 'off';
                obj.vehicle1RotationAngleField.Value = 0;
            end
        end

        function toggleVehicle2RotationOptions(obj, src)
            if src.Value
                obj.rotationAngleField.Enable = 'on';
            else
                obj.rotationAngleField.Enable = 'off';
                obj.rotationAngleField.Value = 0;
            end
        end

        % Getter methods remain unchanged...
        function offsetX = getOffsetX(obj)
            offsetX = obj.offsetXField.Value;
        end

        function offsetY = getOffsetY(obj)
            offsetY = obj.offsetYField.Value;
        end

        function offsetX = getvehicle1OffsetX(obj)
            offsetX = obj.vehicle1OffsetXField.Value;
        end

        function offsetY = getvehicle1OffsetY(obj)
            offsetY = obj.vehicle1OffsetYField.Value;
        end

        function boundType = getBoundType(obj)
            boundType = obj.boundTypeDropdown.Value;
        end

        function vehicle2Mass = getVehicle2Mass(obj)
            vehicle2Mass = obj.vehicle2MassField.Value;
        end

        function vehicle1Mass = getVehicle1Mass(obj)
            vehicle1Mass = obj.vehicle1MassField.Value;
        end

        function rotateVehicle2 = getRotateVehicle2(obj)
            rotateVehicle2 = obj.rotateVehicle2Checkbox.Value;
        end

        function rotationAngle = getRotationAngleVehicle2(obj)
            rotationAngle = obj.rotationAngleField.Value;
        end

        function rotateVehicle1 = getRotateVehicle1(obj)
            rotateVehicle1 = obj.rotateVehicle1Checkbox.Value;
        end

        function rotationAngle = getRotationAngleVehicle1(obj)
            rotationAngle = obj.vehicle1RotationAngleField.Value;
        end
    end
end

%% Helper Functions
function waypoints = generateWaypoints(pathString, resolution)
    if nargin < 2
        resolution = 1;
    end
    segments = split(pathString, '|');
    waypoints = [];
    for i = 1:length(segments)
        segment = strtrim(segments{i});
        if startsWith(segment, 'straight')
            params = parseStraight(segment);
            wp = generateStraightWaypoints(params, resolution);
        elseif startsWith(segment, 'curve')
            params = parseCurve(segment);
            wp = generateCurveWaypoints(params, resolution);
        else
            error('Unknown segment type: %s', segment);
        end
        waypoints = [waypoints; wp]; %#ok<AGROW>
    end
end

function params = parseStraight(segment)
    tokens = regexp(segment, '^straight\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)\)$', 'tokens');
    if isempty(tokens)
        error('Invalid straight segment format: %s', segment);
    end
    tokens = tokens{1};
    params.startX = str2double(tokens{1});
    params.startY = str2double(tokens{2});
    params.endX = str2double(tokens{3});
    params.endY = str2double(tokens{4});
end

function params = parseCurve(segment)
    tokens = regexp(segment, '^curve\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),\s*(cw|ccw)\)$', 'tokens', 'ignorecase');
    if isempty(tokens)
        error('Invalid curve segment format: %s', segment);
    end
    tokens = tokens{1};
    params.centerX = str2double(tokens{1});
    params.centerY = str2double(tokens{2});
    params.radius = str2double(tokens{3});
    params.startAngle = str2double(tokens{4});
    params.endAngle = str2double(tokens{5});
    params.direction = lower(strtrim(tokens{6}));
end

function wp = generateStraightWaypoints(params, resolution)
    dx = params.endX - params.startX;
    dy = params.endY - params.startY;
    distance = sqrt(dx^2 + dy^2);
    numPoints = max(2, ceil(distance * resolution));
    x = linspace(params.startX, params.endX, numPoints)';
    y = linspace(params.startY, params.endY, numPoints)';
    wp = [x, y];
end

function wp = generateCurveWaypoints(params, resolution)
    offset = 2.5; 
    startRad = deg2rad(params.startAngle);
    endRad = deg2rad(params.endAngle);

    if strcmp(params.direction, 'cw')
        if endRad > startRad
            endRad = endRad - 2*pi;
        end
        theta = linspace(startRad, endRad, max(2, ceil((params.radius)*abs(startRad - endRad))*resolution));
        outwardNormalFactor = -1;
    else
        if endRad < startRad
            endRad = endRad + 2*pi;
        end
        theta = linspace(startRad, endRad, max(2, ceil((params.radius)*abs(endRad - startRad))*resolution));
        outwardNormalFactor = 1;
    end

    x = params.centerX + (params.radius)*cos(theta)';
    y = params.centerY + (params.radius)*sin(theta)';
    nx = cos(theta)';
    ny = sin(theta)';
    x_outer = x + outwardNormalFactor * nx;
    y_outer = y + outwardNormalFactor * ny;
    wp = [x_outer, y_outer];
end
