%{
@file Object3D.m
@brief Combines multiple Boxel objects into a single drawable graphic.
%}

classdef Object3D
    properties
        Boxels = Boxel.empty
        Position = [0 0 0]
        Orientation = eye(3)
        UseGPU = false
    end

    methods
        function obj = Object3D(boxels, useGPU)
            if nargin < 1
                boxels = Boxel.empty;
            end
            if nargin < 2
                useGPU = false;
            end
            obj.Boxels = boxels;
            obj.UseGPU = useGPU;
        end

        function addBoxel(obj, boxel)
            if obj.UseGPU
                boxel.UseGPU = true;
            end
            obj.Boxels(end+1) = boxel;
        end

        function setOrientation(obj, yaw, pitch, roll)
            % setOrientation Sets the object orientation from yaw, pitch, roll (rad).
            if nargin == 2 && ismatrix(yaw)
                obj.Orientation = yaw;
                return;
            end
            if nargin < 4, roll = 0; end
            if nargin < 3, pitch = 0; end
            if nargin < 2, yaw = 0; end
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            obj.Orientation = Rz*Ry*Rx;
        end

        function h = draw(obj, ax)
            % draw Renders the composed object by drawing each boxel.
            if nargin < 2 || isempty(ax)
                ax = gca;
            end
            holdState = ishold(ax);
            hold(ax, 'on');
            for i = 1:numel(obj.Boxels)
                b = obj.Boxels(i);
                verts = b.localVertices();
                if obj.UseGPU && gpuDeviceCount > 0
                    verts = gpuArray(verts); %#ok<GPUARRAY>
                    R = gpuArray(obj.Orientation); %#ok<GPUARRAY>
                    verts = (R * verts.').';
                    verts = verts + gpuArray(obj.Position);
                    verts = gather(verts);
                else
                    verts = (obj.Orientation * verts.').';
                    verts = verts + obj.Position;
                end
                patch(ax, 'Vertices', verts, 'Faces', b.faces(), ...
                    'FaceColor', b.Color, 'EdgeColor', 'none');
            end
            if ~holdState
                hold(ax, 'off');
            end
            h = []; % return handle array in future
        end

        function h = smoothSurface(obj, ax, alpha)
            % smoothSurface Draws a smoothed version of the composed object.
            %   This function computes an alpha shape of all vertices
            %   from the contained boxels to produce a smoother looking
            %   surface. A patch handle is returned.

            if nargin < 2 || isempty(ax)
                ax = gca;
            end
            if nargin < 3
                alpha = 1.5;
            end

            verts = obj.collectMesh();
            shp = alphaShape(verts, alpha);
            [facesS, vertsS] = boundaryFacets(shp);

            h = patch(ax, 'Vertices', vertsS, 'Faces', facesS, ...
                'FaceColor', 'interp', 'EdgeColor', 'none');
        end
    end

    methods (Access = private)
        function verts = collectMesh(obj)
            % collectMesh Collects transformed vertices from all boxels
            verts = [];
            idx = 0;
            for i = 1:numel(obj.Boxels)
                b = obj.Boxels(i);
                v = b.localVertices();
                v = (obj.Orientation * v.').';
                v = v + obj.Position;
                verts = [verts; v]; %#ok<AGROW>
                idx = idx + size(v,1);
            end
        end
    end
end
