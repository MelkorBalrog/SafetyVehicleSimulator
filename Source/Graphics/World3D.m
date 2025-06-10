%{
@file World3D.m
@brief Manages and draws a collection of Object3D instances to form a scene.
%}

classdef World3D < handle
    properties
        Objects = Object3D.empty
        CameraPosition = [10 10 10]
        CameraTarget = [0 0 0]
        UseGPU = false
    end

    methods
        function addObject(obj, object3d)
            if obj.UseGPU
                object3d.UseGPU = true;
            end
            obj.Objects(end+1) = object3d;
        end

        function clear(obj)
            obj.Objects = Object3D.empty;
        end

        function draw(obj, ax)
            % draw Renders all objects in the world.
            if nargin < 2 || isempty(ax)
                ax = gca;
            end
            view(ax, 3);
            grid(ax, 'on');
            holdState = ishold(ax);
            hold(ax, 'on');
            for i = 1:numel(obj.Objects)
                obj.Objects(i).draw(ax);
            end
            if ~holdState
                hold(ax, 'off');
            end
            xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
            camproj(ax,'perspective');
            campos(ax, obj.CameraPosition);
            camtarget(ax, obj.CameraTarget);
        end
    end
end
