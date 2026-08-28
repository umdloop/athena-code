# Athena Code

![Alt text](docs/title_picture.JPG)

This repository contains all the code for UMDLoop's 2025-26 rover, Athena.

When cloning, be sure to use `git clone --recursive [URL to Git repository]` to pull in the submodules found in `src/third-party`. Alternatively, run `git submodule update --init --recursive` if you have already cloned.

## How To Use

### Docker setup

*Prior to building:*

Plug in all necessary usb devices. If a new one is plugged in, rebuild the devcontainer.

*devcontainer:*

`Ctrl+Shift+P`

Dev Containers: Rebuild and Reopen in Container

Choose based on what computer this repository is running on. Developer is general use, the other two are for competition specific systems.

*no vscode available:*

1. Navigate to desired system directory (base_station, developer, or jetson)
2. Build the docker

```bash
./build_docker.sh
docker images
```

3. Run the docker

```bash
./run_docker.sh
docker ps
```

*Quick hardware test (joystick controller):*

```bash
sudo evtest
```

### Hardware Setup

- Ensure CAN Bus is connected and hardware is configured to the correct IDs
- Verify that each of the ros2_control xacro files (in `src/description/ros2_control/`) have the correct IDs inside of the `node_id` parameter for each joint

### CAN Setup

_Hardware:_

```bash
./src/tools/scripts/can_setup.sh
```

_Virtual:_

```bash
./src/tools/scripts/virtual_can_setup.sh
```

Use `ip link` to verify that `can0` or `vcan0` is up.

---

### Building

1. Install required dependencies:

```bash
rosdep install --from-paths src -y --ignore-src
```

2. Build the workspace:

```bash
colcon build --symlink-install
```

3. Source the workspace in every terminal you use files specific to this workspace:

```bash
source install/setup.bash
```

4. Launch your desired subsystem.

---

## Subsystems

### Arm

See the [Arm subsystem README](src/subsystems/arm/README.md).

### Drive

See the [Drive subsystem README](src/subsystems/drive/README.md).

### Science

See the [Science subsystem README](src/subsystems/science/README.md).

### Navigation

See the [Navigation subsystem README](src/subsystems/navigation/README.md).

### General

See the [General subsystem README](src/subsystems/general/README.md).

---

## Hardware Interfaces

For a list of available hardware interfaces, see the [Hardware Interfaces README](src/hardware_interfaces/README.md)

---

## Controller Switching

All subsystems use a controller switching service. Open a new terminal, source the workspace, and call:

```bash
source install/setup.bash
ros2 service call /set_controller msgs/srv/SetController "{controller_names: [CONTROLLER_NAME]}"
```

Only one motion controller should be active at a time (alongside the `joint_state_broadcaster`).

---

## How To Contribute

### Learning Git

If you're new to Git and GitHub, start with this beginner-friendly tutorial: [GitHub&#39;s Hello World Guide](https://docs.github.com/en/get-started/quickstart/hello-world)

For a more comprehensive introduction, check out: [Git and GitHub Tutorial for Beginners](https://www.freecodecamp.org/news/git-and-github-for-beginners/)

### Setting up Git

Before you can contribute, you'll need to have Git installed and configured on your machine:

1. **Install Git** - Follow GitHub's guide to [set up Git](https://docs.github.com/en/get-started/git-basics/set-up-git) on your system
2. **Configure Git** - Set your username and email that will be associated with your commits

### SSH Keys

To securely authenticate with GitHub without entering your password each time, set up SSH keys:

1. **Check for existing SSH keys** - See if you already have SSH keys: [Checking for existing SSH keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)
2. **Generate a new SSH key** - If needed, create a new SSH key pair: [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
3. **Add SSH key to GitHub** - Upload your public key to your GitHub account: [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### Making a Pull Request (PR)

Once you're set up, here's how to contribute your changes:

1. **Fork the repository** - Create your own copy of the repository: [Fork a repo](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo)
2. **Clone your fork** - Download your forked repository to your local machine using `git clone`.
3. **Create a feature branch** - Make a new branch for your specific feature or fix:

```bash
git checkout -b feature/your-feature-name
```

4. **Make your changes** - Write your code, test it thoroughly, and commit your changes
5. **Push to your fork** - Upload your feature branch to your GitHub fork

```bash
git push origin feature/your-feature-name
```

6. **Open a Pull Request** - Submit your changes for review: [Creating a pull request from a fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork)

Your PR will be reviewed by your lead, and once approved, it will be merged into the main codebase!

---

### Enabling Real Time ros2_control

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Afterwards, add the following limits to the realtime group in `/etc/security/limits.conf`:

```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

---

## Future Work

- Test docker setup and documentation branch changes on actual rover.
- Add navigation readme.
- Modify `.devcontainer/jetson/Dockerfile` for the Jetson platform and its hardware-specific dependencies.
- Modify `.devcontainer/base_station/Dockerfile` for the base-station machine and its runtime requirements.
- Validate both images on their target machines so they can replace the shared developer setup.
- Test Docker hardware access with a CANable adapter when one is available.
- Fix joystick publisher.
