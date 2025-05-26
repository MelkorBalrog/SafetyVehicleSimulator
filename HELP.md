# SafetyVehicleSimulator Manual

This manual explains how to install, configure, and run simulations in the SafetyVehicleSimulator MATLAB toolbox.

## 1. Overview

SafetyVehicleSimulator models the dynamics of two vehicles and their trailers, detects collisions, and visualizes results through a graphical interface. It combines modules for mapping, control, physics, and plotting.

## 2. Installation

1. Install **MATLAB R2022b** or later with the Parallel Computing Toolbox.
2. Clone this repository.
3. Open MATLAB and add the repository folder to your MATLAB path:
   ```matlab
   addpath(genpath('path/to/SafetyVehicleSimulator'));
   ```

## 3. Running a Simulation

The entry point is `RunSim.m`. Execute it in MATLAB to launch the UI:
```matlab
RunSim
```
The interface allows you to load vehicle data from Excel, start and pause simulations, and save results.

## 4. User Interface

- **File Menu** – Load or save vehicle data, simulation configurations, and results.
- **Control Panel** – Start, pause, or stop the simulation. Adjust vehicle offsets and rotations.
- **Plot Area** – Displays vehicle trajectories and collision events in real time.

## 5. Configuration and Data Management

`ConfigurationManager` saves or loads simulation settings to XML files, while `DataManager` handles Excel import/export and saving MAT files for playback. Use these features through the UI menu options.

## 6. Parallel Computing

Several modules support parallel execution. See `PARALLEL_COMPUTING.md` for implementation details. Ensure a parallel pool is active to accelerate simulations:
```matlab
if isempty(gcp('nocreate'))
    parpool;
end
```

## 7. Testing

Unit tests reside in the `tests` folder. Run them with:
```matlab
results = runtests('tests');
assertSuccess(results);
```

## 8. Troubleshooting

- **No Display** – Check MATLAB path and ensure all dependencies are added.
- **Slow Performance** – Verify that the Parallel Computing Toolbox is installed and a parallel pool is running.

## 9. Further Resources

- `Scripts` folder: helper scripts for collision detection and plotting.
- `Source` folder: core classes for UI, physics, and simulation management.
- `Simulations` folder: sample scenarios and result data.

---
*For more advanced usage, refer to the inline documentation within each MATLAB class.*
