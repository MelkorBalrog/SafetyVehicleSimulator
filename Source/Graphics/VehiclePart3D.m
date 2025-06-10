%{
@file VehiclePart3D.m
@brief Base class for vehicle parts composed of Boxel objects.
%}

classdef VehiclePart3D < Object3D
    properties
        Name
    end
    methods
        function obj = VehiclePart3D(name, boxels, useGPU)
            if nargin < 1
                name = '';
            end
            if nargin < 2
                boxels = Boxel.empty;
            end
            if nargin < 3
                useGPU = false;
            end
            obj@Object3D(boxels, useGPU);
            obj.Name = name;
        end
    end
end
