# VDSS - Vehicle Dynamics Safety Simulator

## Overview
VDSS sets up the simulation environment and user interface for vehicle dynamics experiments. The main `VDSS` function initializes the UI, loads vehicle configurations and runs simulations using the `SimManager`.

## Key Modules
- **SimManager**: Handles running simulations, detecting collisions, and plotting results in parallel.
- **UIManager**: Manages the graphical interface and connects callbacks for user interactions.
- **ConfigurationManager**: Saves and loads simulation configurations from XML files.
- **plotVehicleMotionResults**: Visualizes logged motion data.
- **checkMaxDistanceAtMaxSpeed**: Compares maximum travel distances for two vehicles.

## License
This project is licensed under the GNU General Public License v3. See the [LICENSE](LICENSE) file for details.
