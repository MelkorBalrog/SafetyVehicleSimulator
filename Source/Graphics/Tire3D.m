%{
@file Tire3D.m
@brief Vehicle tire composed of multiple Boxels forming a ring.
%}

classdef Tire3D < VehiclePart3D
    methods
        function obj = Tire3D(radius, width, color, nSegments)
            if nargin < 4
                nSegments = 12;
            end
            if nargin < 3
                color = [0 0 0];
            end
            if nargin < 2
                width = 0.3;
            end
            if nargin < 1
                radius = 0.5;
            end
            obj@VehiclePart3D('Tire');
            for k = 1:nSegments
                ang = 2*pi*(k-1)/nSegments;
                pos = [radius*cos(ang) radius*sin(ang) 0];
                segSize = [width radius*2*sin(pi/nSegments) radius*2*sin(pi/nSegments)];
                b = Boxel(pos + [0 0 segSize(3)/2], segSize, color);
                obj.addBoxel(b);
            end
        end
    end
end
