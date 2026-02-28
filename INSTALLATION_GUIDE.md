# Installation Guide -- LattePanda Alpha Robot

Complete setup guide for Daniel's differential drive robot running on LattePanda Alpha (Core i5) with built-in Arduino Leonardo.

---

## 1. Install Ubuntu 24.04 on LattePanda Alpha

1. Download **Ubuntu 24.04.x LTS Desktop** ISO from https://releases.ubuntu.com/24.04/
2. Create a bootable USB drive using [Balena Etcher](https://etcher.balena.io/) or `dd`
3. Plug USB into LattePanda Alpha, boot from USB (press F7 or Del for boot menu)
4. Install Ubuntu -- select "Erase disk and install Ubuntu"
5. After install, update the system:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git build-essential python3-pip
```

---

## 2. Install ROS2 Jazzy

```bash
# Add ROS2 GPG key
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Source ROS2 in every terminal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Install Required ROS2 Packages

```bash
# Navigation
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# SLAM
sudo apt install -y ros-jazzy-slam-toolbox

# ros2_control (for reference config)
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Teleop keyboard
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# Build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Python serial library (for Arduino communication)
pip3 install pyserial
```

> **Note:** The RPLidar C1 requires `sllidar_ros2` from source (not the apt `rplidar_ros` package).
> It is cloned in Step 5 below alongside the robot package.

---

## 4. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

---

## 5. Clone the Repository

```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone the robot package
git clone https://github.com/MianIdrees/Tomas_bot.git
cd Tomas_bot
git checkout feature/lattepanda-alpha-daniel
cd ..

# Clone sllidar_ros2 (RPLidar C1 driver -- not available via apt)
git clone https://github.com/Slamtec/sllidar_ros2.git
```

---

## 6. Install udev Rules

These create stable device symlinks so the robot always finds the RPLidar and Arduino:

```bash
cd ~/robot_ws/src/Tomas_bot
sudo cp config/99-robot-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Verify the Arduino Leonardo is detected:**
```bash
# The built-in Leonardo on LattePanda appears as /dev/ttyACM*
ls -la /dev/ttyACM*

# After udev rules, it should also appear as:
ls -la /dev/arduino
```

**If the symlink doesn't appear**, find your device's actual vendor/product IDs:
```bash
# Plug in only the Arduino (or use the built-in one):
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct'

# For RPLidar:
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
```

Update the IDs in `config/99-robot-devices.rules` if they differ.

---

## 7. Set Serial Port Permissions

```bash
# Add yourself to the dialout group (required for serial access)
sudo usermod -a -G dialout $USER

# Log out and back in for the group change to take effect
# OR reboot:
sudo reboot
```

---

## 8. Build the Workspace

```bash
cd ~/robot_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 9. Install Arduino IDE (for firmware upload)

```bash
# Install Arduino IDE 2.x (or use the CLI)
sudo apt install -y arduino

# OR download Arduino IDE 2.x from:
# https://www.arduino.cc/en/software
```

**Important:** Select **Arduino Leonardo** as the board type in Arduino IDE.

---

## 10. Upload Arduino Firmware

1. Open Arduino IDE
2. Open `~/robot_ws/src/Tomas_bot/firmware/motor_controller.ino`
3. Select:
   - **Board:** Arduino Leonardo
   - **Port:** `/dev/ttyACM0` (or `/dev/arduino`)
4. Click **Upload**
5. Open Serial Monitor (115200 baud) — you should see:
   ```
   Leonardo motor controller ready (PID + polling)
   ```

---

## Hardware Wiring Reference

### L298N Motor Driver → Arduino Leonardo

| L298N Pin | Arduino Pin | Function          |
|-----------|-------------|-------------------|
| ENA       | D5          | Left motor PWM    |
| IN1       | D7          | Left motor dir    |
| IN2       | D6          | Left motor dir    |
| IN3       | D10         | Right motor dir   |
| IN4       | D9          | Right motor dir   |
| ENB       | D11         | Right motor PWM   |

### Encoder Wiring

| Motor | Wire   | Arduino Pin | Function     |
|-------|--------|-------------|--------------|
| Left  | Green  | D3          | Encoder Ch A (interrupt) |
| Left  | Yellow | D2          | Encoder Ch B (direction) |
| Right | Yellow | A4          | Encoder Ch A (polled)    |
| Right | Green  | A5          | Encoder Ch B (direction) |

### Power

- L298N 12V input → 12V battery
- L298N 5V output → can power Arduino (if not powered by USB)
- Encoder VCC → Arduino 5V
- Encoder GND → Arduino GND

### RPLidar C1

- Connect RPLidar C1 via USB to LattePanda Alpha
- It will appear as `/dev/ttyUSB0` (symlinked to `/dev/rplidar`)

---

## Troubleshooting

### Arduino not detected
```bash
# Check if device exists
ls /dev/ttyACM*

# Check dmesg for USB events
dmesg | tail -20

# Try giving permissions manually
sudo chmod 666 /dev/ttyACM0
```

### RPLidar not detected
```bash
ls /dev/ttyUSB*
# If nothing, check USB cable and power
```

### Build errors
```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Clean and rebuild
cd ~/robot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Serial permission denied
```bash
# Verify you're in dialout group
groups $USER
# Should include "dialout"

# If not, add yourself:
sudo usermod -a -G dialout $USER
# Then log out and back in
```
