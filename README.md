# MRS UAV Unreal Engine 5 (UE5) Simulator

## Installation

### Linux

1. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)
2. Install the MRS UAV System UE5 endpoint
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
3. Download the and untar [Unreal Engine Drone Simulator](https://nasmrs.felk.cvut.cz/index.php/s/kfuWK1KS9h7oJ0I)
4. Launch the UE5 Simulator by `./ueds.sh`.
5. Start the MRS UAV System UE5 endpoint
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
4. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) into WSL 2.0
5. Install the MRS UAV System UE5 endpoint
```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```
6. Download and unzip the [Unreal Engine Drone Simulator](https://nasmrs.felk.cvut.cz/index.php/s/qLVIlBxtk8VCj5q)
7. Start the Unreal Engine Drone Simulator
8. Start the MRS UAV System UE5 endpoint
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```
