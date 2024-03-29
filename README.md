# MRS UAV Unreal Engine 5 (UE5) Simulator

## Requirements

* A dedicated NVidia GPU (at least MX450, RTX is recommended) or a comparable AMD GPU
* 3 GB of HDD space

## The maps

| Map Name  |                         |
|-----------|-------------------------|
| Valley    | ![](.fig/valley.jpg)    |
| Forest    | ![](.fig/forest.jpg)    |
| Warehouse | ![](.fig/warehouse.jpg) |

## Installation

### Linux

1. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)
2. Install the MRS UAV System UE5 endpoint
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
3. Download the and untar [Unreal Engine Drone Simulator](https://nasmrs.felk.cvut.cz/index.php/s/kfuWK1KS9h7oJ0I)
4. Launch the UE5 Simulator by `./ueds.sh`. Alternatively, start it in a headless mode via `./ueds.sh -renderOffScreen`.
5. Start the MRS UAV System UE5 endpoint
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```

### Windows

Requirements:
* Windows 11 + updates
* WSL 2.0

1. Install Ubuntu 20.04 into the WSL 2.0 using the Microsoft Store
2. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) into WSL 2.0
3. Create `.wslconfig` file in `C:/Users/<User>/.wslconfig`
4. Place the following content into `.wslconfig`
```
[wsl2]
firewall=false
networkingMode=mirrored
debugConsole=false
[experimental]
hostAddressLoopback=true
```
5. Restart the WSL by issuing `wsl --shutdown` into a comand line.
6. Install the MRS UAV System UE5 endpoint
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
7. Download and unzip the [Unreal Engine Drone Simulator](https://nasmrs.felk.cvut.cz/index.php/s/qLVIlBxtk8VCj5q)
8. Start the Unreal Engine Drone Simulator
9. Start the MRS UAV System UE5 endpoint
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```
