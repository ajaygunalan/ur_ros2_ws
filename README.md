## Universal Robot ROS2 Workspace
This workspace provides a containerized development environment for Universal Robots with ROS2. You have two options: A or B.

### A. Setup with [VSCode Extensions](https://code.visualstudio.com/docs/remote/remote-overview)

```bash
git clone git@github.com:ajaygunalan/ur_ros2_ws.git
cd ur_ros2_ws
vc import src < workspace.repos
```

open in VS Code by `code .` and select `Reopen in Container`

### B. Normal Docker Setup

#### Initial Setup (Do Once)

1. Build the docker image
    ```bash
    cd ~/ur_ros2_ws
    docker build -t ur-admittance-workspace -f .devcontainer/Dockerfile .
    ```

2. Create and run the docker container (do this only once):

     `docker run` command combines all the settings from your `docker-compose.yml` and relevant parts of `devcontainer.json`.

    ```bash
    docker run -d --name ur-dev-container \
    --network=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LIBGL_ALWAYS_INDIRECT=0 \
    -e ROS_DOMAIN_ID=42 \
    -e QT_X11_NO_MITSHM=1 \
    -v ./src:/home/ros/ros2_ws/src:cached \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --gpus all \
    -w /home/ros/ros2_ws \
    -u ros \
    ur-admittance-workspace \
    sleep infinity
    ```

    Notes:
    - --network=host lets the container use the WSL network stack directly (ideal with mirrored networking).
    - The X11 socket is mounted for GUI apps (RViz2, etc.).


    Command Breakdown:

    * `docker run -d --name ur-dev-container`: Runs the container in detached mode (`-d`) and gives it a memorable name.
    * `--network=host`: Corresponds to `network_mode: host`.
    * `--ipc=host`: Corresponds to `ipc: host`.
    * `--privileged`: Corresponds to `privileged: true`.
    * `-e ...`: Sets each environment variable, just like the `environment` section.
    * `-v ...`: Mounts your local `src` directory and the X11 socket, mirroring the `volumes` section.
    * `--gpus all`: Corresponds to the `deploy` resources for the GPU.
    * `-w /home/ros/ros2_ws`: Sets the working directory inside the container, from `workspaceFolder`.
    * `-u ros`: Sets the user inside the container, from `remoteUser`.
    * `ur-admittance-workspace`: The image you just built.
    * `sleep infinity`: The command to keep the container running, from `command`.

#### Daily Usage

Once the container is created, use these commands for daily development:

1. **Start the container:**
    ```bash
    docker start ur-dev-container
    ```

2. **Access the container's shell:**
    ```bash
    docker exec -it ur-dev-container bash
    ```

3. **Stop the container when done:**
    ```bash
    docker stop ur-dev-container
    ```

Now, follow the steps [here](https://github.com/ajaygunalan/ur_admittance_controller/blob/master/README.md) like a normal ROS2 workspace.


### To to control the robot via WSL2:


Add the following config to C:\USERS\\<user_name>\\.wslconfig
```
[wsl2]
networkingMode=mirrored
firewall=true
```
Then restart it: `wsl --shutdown` and `wsl`


Ensure Windows Firewall allows inbound traffic (`AllowInboundRules=True`) to the WSL interface after enabling mirrored mode by:

```
 Get-NetFirewallProfile -PolicyStore ActiveStore 
```

if not set it by:

```
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

