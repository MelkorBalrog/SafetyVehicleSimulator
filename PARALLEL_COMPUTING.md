# Parallel Computing Implementation Suggestions

This document outlines recommended improvements to leverage parallel computing capabilities across the SafetyVehicleSimulator codebase, with a focus on the Vehicle Model and Simulation Manager modules.

## 1. Entry Point: Launching a Parallel Pool
- Before running any parallel operations, ensure a parallel pool is started:
  ```matlab
  if isempty(gcp('nocreate'))
      parpool('local');
  end
  ```

## 2. SimManager-Level Parallelization
- **Concurrent Vehicle Simulations**: Encapsulate each vehicle’s full simulation in the static methods `runVehicleSim1` and `runVehicleSim2`.
- Use `parfeval` to execute both vehicle simulations asynchronously:
  ```matlab
  futures = [
      parfeval(@SimManager.runVehicleSim1, 1, obj.vehicleSim1),
      parfeval(@SimManager.runVehicleSim2, 1, obj.vehicleSim2)
  ];
  results = fetchOutputs(futures);
  ```
- **Batch Scenarios / Monte Carlo**: Use the new `runBatchVehicleSimulations(vehicleSimArray)` method to explore multiple scenarios in parallel via `parfor`.

## 3. VehicleModel-Level Parallelization
- **Decouple Time Loop**: Extract the per-step update logic in `computeNextFrame` into a pure function that accepts state vectors and returns next state. This enables vectorized evaluation.
- **Vectorize Tire Force Computations**: In `ForceCalculator.computeTireForces`, simple Pacejka models use array operations, and high-fidelity paths now leverage `parfor` in the computation loop.
- **Vectorize Inertia Calculation**: The `calculateInertiaPassenger` method now uses vectorized array operations for computing inertia instead of loops.
- **Batch Road Disturbances**: If multiple road segments or friction maps are evaluated, use `spmd` or `parfor` to parallelize across road profiles.

## 4. GPU Acceleration
- Identify large matrix operations in `DynamicsUpdater` and `KinematicsCalculator`.  Cast arrays to `gpuArray` and perform computations on the GPU:
  ```matlab
  state_gpu = gpuArray(state);
  next_gpu = someDynamicsFunction(state_gpu, params);
  state = gather(next_gpu);
  ```

## 5. Physics & Mechanics Modules
- **Parallel Tire Models**: In `Pacejka96TireModel` and `PacejkaMagicFormula`, wrap independent evaluations in `parfor`.
- **Collision Detection**: For a large number of collision checks (e.g., Monte Carlo or multi-agent), distribute pairwise collision checks across workers.

## 6. Implementation Roadmap
1. Refactor `computeNextFrame` into a stateless helper function.
2. Use the existing methods `runVehicleSim1` / `runVehicleSim2` to execute per-vehicle simulations in isolated calls.
   - Initializes state
   - Loops (or vectorizes) to compute full trajectory
   - Returns trajectory and diagnostics
3. Update `SimManager.runSimulations` to launch two or more vehicle simulations in parallel, then merge results for collision detection and plotting.
4. Benchmark serial vs. parallel performance; tune number of workers.
5. Add unit tests for the new parallel functions.

---
*This is a living document; please update with profiling data and new patterns as the codebase evolves.*