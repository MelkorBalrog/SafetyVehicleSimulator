%{
% @file StabilityChecker.m
% @brief Monitors vehicle stability and detects rollover risk.
% @author Miguel Marina
%}
%/**
% * @class StabilityChecker
% * @brief Monitors and detects various stability conditions in a vehicle-trailer system,
% *        and recommends updates to Pacejka/hitch parameters to mitigate instabilities.
% *
% * This class includes additional ODE-based adaptation formulas in `recommendPacejkaUpdates(dt)`
% * and `recommendHitchUpdates(dt)` that adjust selected Pacejka tire parameters and hitch stiffness
% * to reduce rollover or jackknife risk, respectively.
% *
% * @version 3.4
% * @date 2024-12-28
% */
classdef StabilityChecker
    properties
        % ----------------------------------------------------------------
        %  Instances
        % ----------------------------------------------------------------
        dynamicsUpdater        % (Optional) Instance of DynamicsUpdater
        forceCalculator        % Instance of ForceCalculator
        kinematicsCalculator   % Instance of KinematicsCalculator
        
        % ----------------------------------------------------------------
        %  Stability Flags
        % ----------------------------------------------------------------
        isWiggling
        isRollover
        isSkidding
        isJackknife
        
        % ----------------------------------------------------------------
        %  Physical / Trailer Parameters
        % ----------------------------------------------------------------
        m_trailer
        L_trailer
        mu
        h_CoG
        
        % ----------------------------------------------------------------
        %  Internally Calculated Thresholds
        % ----------------------------------------------------------------
        rollAngleThreshold
        hitchInstabilityThreshold
        maxArticulationAngle
        currentRollAngle
        currentHitchAngle
        
        % ----------------------------------------------------------------
        %  Wiggling Detection
        % ----------------------------------------------------------------
        yawMomentThreshold
        
        % ----------------------------------------------------------------
        %  Dynamic Parameters
        % ----------------------------------------------------------------
        velocity
        angularVelocity
        turnRadius
        vehicleMass
        orientation

        % ----------------------------------------------------------------
        %  Wiggling Detection
        % ----------------------------------------------------------------

        % A short-term buffer of trailer yaw angles for sign-change detection
        trailerYawBuffer
        wiggleWindowSize = 20     % how many past samples to store
        wigglingThreshold = 0.4   % if wiggleMeasure > 0.4 => we say "exceed"

        % Use a minimal or minimal-larger threshold if you want to filter out small angles
        minYawAmplitude = 0.005   % optional: ignore tiny differences if < 0.005 rad
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Confidence Score Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        % Wiggling
        wigglingScore = 0
        wigglingScoreHigh = 10
        wigglingScoreLow  = 9
        wigglingIncrement = 1.2
        wigglingDecrement = 0.8

        % Rollover
        rolloverScore = 0
        rolloverScoreHigh = 5
        rolloverScoreLow  = 2
        rolloverIncrement = 1.2
        rolloverDecrement = 0.8

        % Skidding
        skiddingScore = 0
        skiddingScoreHigh = 5
        skiddingScoreLow  = 2
        skiddingIncrement = 1.2
        skiddingDecrement = 0.8

        % Jackknife
        jackknifeScore = 0
        jackknifeScoreHigh = 5
        jackknifeScoreLow  = 2
        jackknifeIncrement = 1.2
        jackknifeDecrement = 0.8

        % Keep track of filtered risk indices for wiggling, rollover, etc.
        filteredWigglingRisk = 0
        filteredRolloverRisk = 0
        filteredSkiddingRisk = 0
        filteredJackknifeRisk = 0
    
        % We can also store time of last detection if we want time-based latching
        lastWigglingOn = 0
        lastRolloverOn = 0
        lastSkiddingOn = 0
        lastJackknifeOn = 0
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NEW: Store Current & Nominal Values for Pacejka/Hitch
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        % Example: we adapt pDy1 in the lateral Pacejka model to reduce rollover risk
        pDy1_nominal  = 1.0   % Nominal pDy1 (example)
        pDy1_current  = 1.0   % Current adaptation value

        % Example: we adapt Hitch yaw stiffness to reduce jackknife risk
        K_hitchYaw_nominal = 1000.0 % Nominal yaw stiffness (N·m/rad)
        K_hitchYaw_current = 1000.0 % Current adaptation value
    end
    
    methods
        % ================================================================
        % Constructor
        % ================================================================
        function obj = StabilityChecker(dynamicsUpdater, forceCalc, kinematicsCalc, ...
                                        m_trailer, L_trailer, mu, h_CoG)
            fprintf('Initializing StabilityChecker (auto-threshold calculation)...\n');

            obj.dynamicsUpdater      = dynamicsUpdater;
            obj.forceCalculator      = forceCalc;
            obj.kinematicsCalculator = kinematicsCalc;

            obj.m_trailer = m_trailer;
            obj.L_trailer = L_trailer;
            obj.mu        = mu;
            obj.h_CoG     = h_CoG;

            obj.isWiggling  = false;
            obj.isRollover  = false;
            obj.isSkidding  = false;
            obj.isJackknife = false;

            obj.currentHitchAngle = 0;
            obj.currentRollAngle  = 0;

            obj.velocity        = [0; 0; 0];
            obj.angularVelocity = [0; 0; 0];
            obj.turnRadius      = Inf; 
            obj.vehicleMass     = 0;  
            obj.orientation     = 0;

            % Compute thresholds
            trackWidth = 1.8;
            SSF_critical = 0.4;
            obj.rollAngleThreshold = atan(SSF_critical);

            g = 9.81;
            obj.hitchInstabilityThreshold = obj.mu * (obj.m_trailer * g) * (obj.L_trailer / 2);
            obj.maxArticulationAngle      = atan(obj.L_trailer / (2 * obj.h_CoG));
            obj.yawMomentThreshold        = obj.mu * obj.m_trailer * g * (obj.L_trailer / 2);

            fprintf('Auto-threshold => rollAngle=%.4f, hitch=%.2f N, maxArtic=%.4f, yawMoment=%.2f\n',...
                obj.rollAngleThreshold, obj.hitchInstabilityThreshold, ...
                obj.maxArticulationAngle, obj.yawMomentThreshold);
        end

        % ================================================================
        % Update Dynamics
        % ================================================================
        function obj = updateDynamics(obj, newVelocity, newAngularVelocity, ...
                                      newTurnRadius, newVehicleMass, orientation)
            obj.velocity        = newVelocity;
            obj.angularVelocity = newAngularVelocity;
            obj.turnRadius      = newTurnRadius;
            obj.vehicleMass     = newVehicleMass;
            obj.orientation     = orientation;
        end

        % ================================================================
        % DETECT WIGGLING via Sign-Change in Trailer Yaw Angle
        % ================================================================
        function obj = detectWiggling(obj)
            % 1) Get trailer yaw angle from forceCalculator (or kinematicsCalculator)
            forces = obj.forceCalculator.getCalculatedForces();
            if isKey(forces, 'trailerPsi')
                currentTrailerYaw = forces('trailerPsi');
            else
                currentTrailerYaw = 0; % fallback
            end

            % 2) Append to trailerYawBuffer
            obj.trailerYawBuffer(end+1) = currentTrailerYaw;
            if length(obj.trailerYawBuffer) > obj.wiggleWindowSize
                obj.trailerYawBuffer(1) = [];  % remove oldest
            end

            % If we haven't reached the window size yet, skip
            if length(obj.trailerYawBuffer) < obj.wiggleWindowSize
                return;
            end

            % 3) Compute discrete differences of yaw angles
            diffs = diff(obj.trailerYawBuffer);

            % Optionally ignore small diffs < minYawAmplitude
            diffs(abs(diffs) < obj.minYawAmplitude) = 0;

            % 4) Count sign changes
            signChanges = 0;
            for i = 1:length(diffs)-1
                if diffs(i)*diffs(i+1) < 0
                    % That means sign(diffs(i)) != sign(diffs(i+1))
                    signChanges = signChanges + 1;
                end
            end

            % 5) Calculate "wiggleMeasure" = signChanges / (N-1)
            % where N = length(diffs). Actually, diffs has length = wiggleWindowSize-1
            N = length(diffs);
            if N > 1
                wiggleMeasure = signChanges / (N-1);
            else
                wiggleMeasure = 0;
            end

            % 6) Compare wiggleMeasure to threshold
            rawIsExceed = (wiggleMeasure > obj.wigglingThreshold);

            % 7) Convert to score update
            if rawIsExceed
                obj.wigglingScore = obj.wigglingScore + obj.wigglingIncrement;
            else
                obj.wigglingScore = max(0, obj.wigglingScore - obj.wigglingDecrement);
            end

            % 8) Apply hysteresis in the final flag
            if obj.wigglingScore >= obj.wigglingScoreHigh
                obj.isWiggling = true;
            elseif obj.wigglingScore <= obj.wigglingScoreLow
                obj.isWiggling = false;
            end
        end

        % ================================================================
        % DETECT ROLLOVER
        % ================================================================
        function obj = detectRollover(obj)
            % 1) Get raw rollover risk index from ForceCalculator
            forces = obj.forceCalculator.getCalculatedForces();
            rolloverRiskRaw = 0;
            if isKey(forces, 'rolloverRiskIndex')
                rolloverRiskRaw = forces('rolloverRiskIndex');
            end
        
            % 2) Increase threshold to avoid small spurious triggers
            biggerThreshold = 1.2; % we might treat "risk>1.2" as exceed
        
            % 3) Filter the raw risk
            alphaFilter = 0.9;
            obj.filteredRolloverRisk = alphaFilter * obj.filteredRolloverRisk + ...
                                       (1 - alphaFilter) * rolloverRiskRaw;
        
            % 4) Check if filtered risk is above bigger threshold
            rawIsExceed = (obj.filteredRolloverRisk > biggerThreshold);
        
            % 5) Score + hysteresis
            if rawIsExceed
                obj.rolloverScore = obj.rolloverScore + obj.rolloverIncrement;
            else
                obj.rolloverScore = max(0, obj.rolloverScore - obj.rolloverDecrement);
            end
        
            if obj.rolloverScore >= obj.rolloverScoreHigh
                obj.isRollover = true;
            elseif obj.rolloverScore <= obj.rolloverScoreLow
                obj.isRollover = false;
            end
        end

        % ================================================================
        % DETECT SKIDDING
        % ================================================================
        function obj = detectSkidding(obj)
            forces = obj.forceCalculator.getCalculatedForces();
        
            % 1) Check throttle or engine torque from your 'dynamicsUpdater' or 'forceCalculator'
            %    Suppose we have obj.dynamicsUpdater.getThrottle() returning [0..1]
            throttle = 0;
            if ~isempty(obj.dynamicsUpdater) && ismethod(obj.dynamicsUpdater, 'getThrottle')
                throttle = obj.dynamicsUpdater.getThrottle();
            end
        
            % If throttle < small threshold (say 0.1), skip skidding detection
            if throttle < 0.1
                % treat as NOT skidding
                obj.skiddingScore = max(0, obj.skiddingScore - obj.skiddingDecrement);
                if obj.skiddingScore <= obj.skiddingScoreLow
                    obj.isSkidding = false;
                end
                return;
            end
        
            % 2) If we do want to check, then do your normal skidding logic:
            Fc = 0;
            if (obj.turnRadius > 0) && (obj.vehicleMass > 0)
                Fc = obj.vehicleMass * (obj.velocity(1)^2) / obj.turnRadius;
            end
        
            F_traction = 0;
            if isKey(forces, 'traction')
                F_traction_vec = forces('traction');
                F_traction     = F_traction_vec(1);
            end
        
            % Example: less sensitive => multiply Fc by 1.2 (or 0.3, etc.)
            rawIsExceed = (F_traction < 1.3 * Fc);
        
            if rawIsExceed
                obj.skiddingScore = obj.skiddingScore + obj.skiddingIncrement;
            else
                obj.skiddingScore = max(0, obj.skiddingScore - obj.skiddingDecrement);
            end
        
            if obj.skiddingScore >= obj.skiddingScoreHigh
                obj.isSkidding = true;
            elseif obj.skiddingScore <= obj.skiddingScoreLow
                obj.isSkidding = false;
            end
        end

        % ================================================================
        % DETECT JACKKNIFE (unchanged example)
        % ================================================================
        function obj = detectJackknife(obj)
            forces = obj.forceCalculator.getCalculatedForces();
            F_hitch_lateral = 0;
            if isKey(forces, 'hitchLateralForce')
                F_hitch_lateral = forces('hitchLateralForce');
            end

            % Suppose we store trailer yaw minus tractor yaw in currentHitchAngle
            trailerYaw = obj.currentHitchAngle; 
            tractorYaw = obj.orientation;      
            hitchAngleRad = trailerYaw - tractorYaw;
            obj.currentHitchAngle = hitchAngleRad;

            angleDeg = abs(rad2deg(hitchAngleRad));
            angleThrDeg = 1.01 * rad2deg(obj.maxArticulationAngle);
            forceThr    = 1.01 * obj.hitchInstabilityThreshold;

            rawIsExceed = (angleDeg > angleThrDeg) && (abs(F_hitch_lateral) > forceThr);

            if rawIsExceed
                obj.jackknifeScore = obj.jackknifeScore + obj.jackknifeIncrement;
            else
                obj.jackknifeScore = max(0, obj.jackknifeScore - obj.jackknifeDecrement);
            end

            if obj.jackknifeScore >= obj.jackknifeScoreHigh
                obj.isJackknife = true;
            elseif obj.jackknifeScore <= obj.jackknifeScoreLow
                obj.isJackknife = false;
            end
        end

        % ================================================================
        % CHECK STABILITY
        % ================================================================
        function obj = checkStability(obj)
            % ------------------------------------------
            % 1) If speed is below a certain threshold,
            %    reset all flags to false & scores to 0.
            % ------------------------------------------
            speedMag = norm(obj.velocity(1:2));  % only horizontal?
            lowSpeedThreshold = 2.0;  % e.g., 2 m/s
            if speedMag < lowSpeedThreshold
                % Reset everything
                obj.isWiggling  = false; obj.wigglingScore  = 0;
                obj.isRollover  = false; obj.rolloverScore  = 0;
                obj.isSkidding  = false; obj.skiddingScore  = 0;
                obj.isJackknife = false; obj.jackknifeScore = 0;
                return;
            end

            % If above threshold, do normal checks:
            obj = obj.detectWiggling();
            obj = obj.detectRollover();
            obj = obj.detectSkidding();
            obj = obj.detectJackknife();
        end

        % ================================================================
        % GET STABILITY FLAGS
        % ================================================================
        function [isWiggling, isRollover, isSkidding, isJackknife] = getStabilityFlags(obj)
            isWiggling  = obj.isWiggling;
            isRollover  = obj.isRollover;
            isSkidding  = obj.isSkidding;
            isJackknife = obj.isJackknife;
        end

        % ================================================================
        % GET UNSTABLE FLAG
        % ================================================================
        function isUnstable = getUnstableFlag(obj)
            isUnstable = (obj.isWiggling || obj.isRollover || ...
                          obj.isSkidding || obj.isJackknife);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% NEW METHODS: Recommend ODE-Based Adaptations (Pacejka/Hitch)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % ================================================================
        % RECOMMEND PACEJKA UPDATES
        % ================================================================
        function [obj, pDy1_updated] = recommendPacejkaUpdates(obj, dt)
            forces = obj.forceCalculator.getCalculatedForces();
            rolloverRisk = 0;
            if isKey(forces, 'rolloverRiskIndex')
                rolloverRisk = forces('rolloverRiskIndex');
            end

            alpha_roll = 0.05;  
            beta_relax = 0.02;  

            reduceTerm = -alpha_roll * max(0, rolloverRisk - 1); 
            relaxTerm  =  beta_relax * (obj.pDy1_nominal - obj.pDy1_current);

            dpDy1_dt = reduceTerm + relaxTerm;
            pDy1_next = obj.pDy1_current + dpDy1_dt * dt;
            pDy1_next = max(min(pDy1_next, 2.0), 0.3);

            obj.pDy1_current = pDy1_next;
            pDy1_updated     = pDy1_next;
        end

        % ================================================================
        % RECOMMEND HITCH UPDATES
        % ================================================================
        function [obj, K_hitch_updated] = recommendHitchUpdates(obj, dt)
            forces = obj.forceCalculator.getCalculatedForces();
            jackRisk = 0;
            if isKey(forces, 'jackknifeRiskIndex')
                jackRisk = forces('jackknifeRiskIndex');
            end

            gamma_jack  = 100.0;  
            gamma_relax =  10.0;  

            stiffenTerm =  gamma_jack * max(0, jackRisk - 1);
            relaxTerm   = -gamma_relax * (obj.K_hitchYaw_current - obj.K_hitchYaw_nominal);

            dK_dt = stiffenTerm + relaxTerm;
            K_hitch_next = obj.K_hitchYaw_current + dK_dt * dt;
            K_hitch_next = max(min(K_hitch_next, 5000), 500);

            obj.K_hitchYaw_current = K_hitch_next;
            K_hitch_updated        = K_hitch_next;
        end
    end
end
