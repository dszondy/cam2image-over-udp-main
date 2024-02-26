# Tools
This branch documents the process of transmitting webcam frames from a local host to a Multipass instance over UDP.

# General Installation steps:
These steps are written with the assumption that none of this is set up yet. Please use your own common sense.

- Install Visual Studio Code
- Install Multipass https://multipass.run/docs/how-to-guides
  - Select the right way to install
- Install Python https://www.python.org/downloads/
- Make sure to install the used packages using pip
  - This can be done using ```$ pip install -r requirements.txt``` in the top-level folder in which requirements.txt is also located
  - virtualenvs are allowed, but issues with these are your own problem to solve!


# Installation on Windows machines:
First, the necessary programs are installed

- Install VirtualBox (if your machine has Hyper-V this is not necessary) https://www.virtualbox.org/wiki/Downloads

## Set up local environment in Windows
- Open the videoserver.py file in Visual Studio Code


Fill in the ip address of the VirtualBox connection as host IP. The IP address can be found as follows:

`Open Control Panel -> Network and Internet -> Network and Sharing -> Change adapter settings -> right click on the ethernet adapter that VirtualBox is using -> properties -> click on Internet Protocol Version 4 (TCP/IP) -> Properties -> IP adress`

Run the python script

# Installation on MacOS machines:
See steps above, no extra step needs to be taken

## Set up of the local environment on MacOS
- Open the `videoserver.py` file in your editor (Visual Studio Code).
- Go through the setup of Multipass and find out what the Multipass instance IP is
  - Fill in this IP in `videoserver.py`, but change the last number set to .1
  - e.g. `192.168.64.1`

# Set up Multipass environment
Open a command terminal
run the following command to launch a Multipass instance with ros2 Humble:
```
multipass launch ros2-humble --name humble-vm
```

To create a mounted local folder that is accessible through the multipass environment, the following should be done:
1. Run the following command in a terminal to allow mounting a folder
```
multipass set local.privileged-mounts=true
```

2. Create a folder on your host machine that serves as the shared folder between the host and multipass environment

3. Mount the created folder to the multipass environment:
```
multipass mount /some/local/path humble-vm:/some/remote/path
```
Now the local folder is shared with the multipass environment

run the following command to open the vm in a shell:
```
multipass shell humble-vm
```
This opens a the Multipass instance in a windows shell

Check if the local folder is visible and accessible in the multipass environment by typing ls in the shell

In the local folder, place the videoclient.py file and fill in the correct host_ip that you found
run the python script in the shell

# Installing `image_tools_sdfr` on the VM

1. Create a workspace folder (eg. `cam2image_ws`).
2. Put the `src` folder and the `cam2image.yaml` file in the workspace folder.
3. Open a terminal that has its location in the workspace folder and run `colcon build`.
4. After compilation run `. install/setup.bash`.

To run the custom `cam2image`, run the following command:

`ros2 run image_tools_sdfr cam2image --ros-args --params-file cam2image.yaml`

## TODO
- [ ] Convert documentation for Multipass.