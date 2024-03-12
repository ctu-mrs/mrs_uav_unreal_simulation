# MRS UAV Unreal Engine Simulator

## Installation

### Linux

1. Install the MRS UAV System
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
6. Download the [Unreal Engine Drone Simulator]()
3. Launch the unreal simulator by `./ueds.sh`.
4. Start the MRS UAV System UE5 endpoint
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```

### Windows

Requirements:
* Windows 11
* WSL 2.0

1. Install Ubuntu 20.04 into the WSL 2.0 using the Microsoft Store
2. Create `.wslconfig` file in `C:/Users/<User>/.wslconfig`
3. Place the following content into `.wslconfig`
```
[wsl2]
firewall=false
networkingMode=mirrored
debugConsole=false
[experimental]
hostAddressLoopback=true
```
4. Install the [MRS UAV System]() into WSL 2.0
5. Install the MRS UAV System UE5 endpoint
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
6. Download the [Unreal Engine Drone Simulator](https://nasmrs.felk.cvut.cz/index.php/s/qLVIlBxtk8VCj5q)
7. Start the Unreal Engine Drone Simulator
4. Start the MRS UAV System UE5 endpoint
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```
