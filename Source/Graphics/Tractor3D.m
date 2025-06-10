%{
@file Tractor3D.m
@brief Simple tractor body built from Boxels.
%}

classdef Tractor3D < VehiclePart3D
    methods
        function obj = Tractor3D(length, width, height, color)
            if nargin < 4
                color = [1 0 0];
            end
            if nargin < 3
                height = 1.5;
            end
            if nargin < 2
                width = 1.0;
            end
            if nargin < 1
                length = 3.0;
            end
            obj@VehiclePart3D('Tractor');
            b = Boxel([length/2 0 height/2], [length width height], color);
            obj.addBoxel(b);
        end
    end
end
