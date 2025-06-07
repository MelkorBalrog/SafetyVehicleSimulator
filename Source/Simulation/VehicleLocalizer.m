classdef VehicleLocalizer < handle
    % VehicleLocalizer Localizes vehicle position on a map and provides
    % distance to the next curve along the path.

    properties
        waypoints
        waypointSpacing (1,1) double = 1.0
    end

    methods
        function obj = VehicleLocalizer(waypoints, spacing)
            if nargin >= 1 && ~isempty(waypoints)
                if iscell(waypoints)
                    waypoints = cell2mat(waypoints);
                end
                obj.waypoints = double(waypoints);
            else
                obj.waypoints = zeros(0,2);
            end
            if nargin >= 2 && ~isempty(spacing)
                obj.waypointSpacing = spacing;
            end
        end

        function idx = localize(obj, position)
            %LOCALIZE Returns the closest waypoint index to the given position.
            if isempty(obj.waypoints)
                idx = 1;
                return;
            end
            posVec = double(position(:)');
            if numel(posVec) > 2
                posVec = posVec(1:2);
            end
            diffs = obj.waypoints(:,1:2) - posVec;
            [~, idx] = min(sum(diffs.^2, 2));
        end

        function dist = distanceToNextCurve(obj, currentIdx, upcomingRadii)
            %DISTANCETONEXTCURVE Compute distance to first upcoming non-infinite radius
            curveIdx = find(~isinf(upcomingRadii), 1, 'first');
            if isempty(curveIdx)
                dist = Inf;
            else
                dist = (curveIdx-1) * obj.waypointSpacing;
            end
        end
    end
end
