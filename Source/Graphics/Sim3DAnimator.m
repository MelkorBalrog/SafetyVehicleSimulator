%{
@file Sim3DAnimator.m
@brief Animates simulation results using the 3D graphics framework.
%}

classdef Sim3DAnimator < handle
    properties
        DataManager
        GraphicsWindow
        Vehicle1
        Vehicle2
        Steps
    end

    methods
        function obj = Sim3DAnimator(dataManager, graphicsWindow)
            if nargin < 2 || isempty(graphicsWindow)
                graphicsWindow = GraphicsWindow();
            end
            obj.DataManager = dataManager;
            obj.GraphicsWindow = graphicsWindow;
            obj.Steps = numel(dataManager.globalVehicle1Data.X);

            % Simple vehicle representations using vehicle parts
            obj.Vehicle1 = Tractor3D();
            obj.Vehicle2 = Tractor3D();
            obj.GraphicsWindow.World.addObject(obj.Vehicle1);
            obj.GraphicsWindow.World.addObject(obj.Vehicle2);
        end

        function run(obj, dt)
            if nargin < 2
                dt = obj.DataManager.dt;
            end
            for k = 1:obj.Steps
                obj.updateVehicles(k);
                obj.GraphicsWindow.render();
                pause(dt);
            end
        end

        function updateVehicles(obj, k)
            if k > obj.Steps
                return;
            end
            obj.Vehicle1.Position = [obj.DataManager.globalVehicle1Data.X(k), ...
                                     obj.DataManager.globalVehicle1Data.Y(k), 0];
            obj.Vehicle1.setOrientation(obj.DataManager.globalVehicle1Data.Theta(k));

            obj.Vehicle2.Position = [obj.DataManager.globalVehicle2Data.X(k), ...
                                     obj.DataManager.globalVehicle2Data.Y(k), 0];
            obj.Vehicle2.setOrientation(obj.DataManager.globalVehicle2Data.Theta(k));
        end
    end
end
