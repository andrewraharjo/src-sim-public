# Overview

We are proud to make our simulation available to participants in Self Racing Cars!

## Rules

The challenge is structured like a qualifying session.  Your car will be the only one on the track, as you attempt to set the fastest time from the start-finish line, around the course, back to the start-finish line.

The car will start on the start-finish straight with plenty of room to get up to top speed before crossing the line.  You can let the car do as many laps as you like, without restarting, and the best will be counted.  Any excursions off the track or collisions will result in that lap being invalidated.  In the section below on what messages we publish, we tell you how you will know if your lap will be invalidated.

## Titles

__Grand Prix:__ goes to the team that completes a lap (start-finish line to start-finish line) in the least time.

__Endurance:__ goes to the team that completes the most laps in the time they have with their instance.

__Project D:__ goes to the team that maximizes sideslip angle over their lap (computed as mean L2 norm).

__Demolition Derby:__ goes to the team that has the highest-speed collision.

## Training data

To get everyone started, we provide (human-driven) training data in rosbag form.  Here are ten laps:

[1](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap1.bag.tar.gz) [2](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap2.bag.tar.gz) [3](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap3.bag.tar.gz) [4](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap4.bag.tar.gz) [5](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap5.bag.tar.gz) [6](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap6.bag.tar.gz) [7](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap7.bag.tar.gz) [8](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap8.bag.tar.gz) [9](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap9.bag.tar.gz) [10](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-training-data/lap10.bag.tar.gz)


See the "Helpful guides and further reading for ROS" section below for documentation on using the rosbag format.

## Machine setup

Below we describe two ways of using the simulator.  The first is if you're running a computer with Ubuntu (really any Linux that works with ROS).  The second is a graphical VM using Vagrant.

The basic steps for machine set up are:
- install ROS
- install custom RightHook messages
- install OpenVPN

### Bare Ubuntu

#### Install ROS

Instructions are based on [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Set up your sources.list:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Set up your keys:

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Install packages:

    sudo apt-get update
    sudo apt-get install -y ros-kinetic-desktop
    sudo apt-get install -y ros-kinetic-joystick-drivers
    sudo apt-get install -y ros-kinetic-image-view

These packages are enough to run and sanity-check the simulation.  To run your controller, you may need additional ones.  Check out what is available at the [Kinetic packages list](http://www.ros.org/browse/list.php) or by running `sudo apt-cache search ros`.

Initialize dependency helper:

    sudo rosdep init
    rosdep update

Set ROS environment to be sourced at login:

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

The line below sets the `ROS_MASTER_URI` environment variable at login to point to the simulation server through the VPN.  If you do not want to do that (maybe you use ROS for other stuff), you don't have to, but you will have to remember to `export ROS_MASTER_URI=http://10.8.0.1:11311` in any shell where you want to interact with the simulation:

    echo "export ROS_MASTER_URI=http://10.8.0.1:11311" >> ~/.bashrc
    
#### Install the custom RightHook messages:

These messages are specific to the vehicle or competition (more below in "Put your controller in the loop using ROS").  They can be downloaded [here](https://s3-us-west-2.amazonaws.com/public.righthook.io/src-packages/rh-msgs_1.0.3_amd64.deb).  Assuming you downloaded them to your home directory, install using

    sudo dpkg -i ~/rh-msgs_1.0.3_amd64.deb

#### Install OpenVPN

You will connect to the simulation using OpenVPN.

    sudo apt-get install -y openvpn

### VM

If you are using Mac or Windows, we have supplied a Vagrant file that will get a GUI Ubuntu VM running on your machine, already provisioned with everything you will need to connect to the simulator.  We have tested using MacOS Sierra and VirtualBox 5.1.16.

First, be sure you have Git, Vagrant, and VirtualBox installed.

#### Install Git

You may already have it.  Check using `git --version`.  

If you don't, please follow the instructions at [Atlassian's Install Git Tutorial](https://www.atlassian.com/git/tutorials/install-git).

#### Install Vagrant

You can install Vagrant by downloading directly from [Hashicorp's website](https://www.vagrantup.com/downloads.html) or by following [these instructions](http://sourabhbajaj.com/mac-setup/Vagrant/README.html).

#### Install VirtualBox

You can install VirtualBox by downloading directly from [VirtualBox's website](https://www.virtualbox.org/wiki/Downloads) or by following [these instructions](http://sourabhbajaj.com/mac-setup/Vagrant/README.html).

#### Clone our VM repo

Open a shell and go to a directory where you have some space and are ok with storing stuff.  Run

    git clone https://github.com/righthook/self-racing-cars-vm-gui.git

or
    
    git clone git@github.com:righthook/self-racing-cars-vm-gui.git

if you are using SSH for Git authentication.

#### Provision and boot VM

*Note:* At this point, you can copy your `righthook.io.conf` OpenVPN configuration into the directory created when you cloned the repo.  It will then be shared to the VM's `/home/vagrant` directory.

In a shell, `cd` into the `self-racing-cars-vm-gui` directory that was created when you cloned our Git repo.  Boot the VM by running

    vagrant up --provision

After some downloading, you will see the Ubuntu GUI come up.  There is a provisioning script running in the background (whose output is probably in your terminal).  *Let it run until the VM reboots.*

You can shutdown the VM any time by going to the top right corner, clicking on the power icon, and choosing "Shutdown...".  You can boot the machine at any time by `cd`ing into the `self-racing-cars-vm-gui` directory and running `vagrant up`.

If you are ever prompted for a user name and password by the Ubuntu VM, it should be user name `vagrant` and password `vagrant` unless you've changed them yourself.

## Conventions, Coordinate Frames

Unless stated otherwise in the variable name, all variables and signals are SI and angles are radians.  All coordinate frames are right-handed.

The inertial frame (broadcasted in ROS as "/level") is oriented with the x-axis positive going east, y-axis positive going north, and z-axis positive going up.

The vehicle frame is oriented with the x-axis positive going forward, the y-axis positive going port, and the z-axis positive going up.  Therefore, positive roll is right side down, positive pitch is nose down, and positive yaw is turning left.

The steering input is positive going left, matching the yaw sense.

## The vehicle

The vehicle is based on a go-kart.  It is rear-wheel drive and the brake only acts on the rear axle.

Width: 0.932 m

Length: 1.52 m

Mass: 144.0 kg

Top speed on level ground: ~14 m/s

## The sensor set

The vehicle is outfitted with virtual position, camera, and inertial measurement sensors.  They are summarized below.

*Location:* The location sensor is located at the vehicle's center of mass.  It broadcasts the transform from the Cartesian reference point (near the start-finish line, oriented east-north-up) to the vehicle.  Additionally, we project this into WGS84 coordinates and broadcast those.

*Camera:*  The camera sensor is located on the vehicle's left-right centerline, 0.8m in front of the center of mass and 0.3m off the ground.  The camera image is 1280x720 encoded using `bayer_rggb8`.

*Inertial Measurement Unit:*  The IMU is located at the vehicle's center of mass and broadcasts linear acceleration, orientation, and angular velocity.

# Using the Simulation

## Starting your instance

To start your instance, navigate to [our portal](http://app.righthook.io) and log in using the credentials we supplied you.  You will see your remaining time credit as well as buttons to download your VPN configuration and to start the instance.  If you have not yet, download the VPN configuration to your machine.  In subsequent steps, we assume you saved the VPN configuration as `~/righthook.io.conf`.  _Note_: this is also where the VPN configuration will be if you used our VM.  Do not share the VPN configurations, they are unique to each team.

Click on the "Start" button to start your instance.  It will go from "Pending" to "Stop," signifying that the instance is ready to go.

When you are done, please click "Stop" or your credits will continue to deplete.

__IMPORTANT__: time is counted rounding up to the hour because that's how we get billed for it.  If you get the car stuck or invalidate your lap, to save time, you can reset your lap using the simulation controls described below in "Pausing, unpausing, and resetting the simulation."  If you are making minor tweaks to your controller, we suggest you leave the instance running.

## Connecting to your instance

To marshall traffic between the simulation instance and your local machine, we create a virtual private network (VPN) using OpenVPN.  If you do not already have OpenVPN installed, run

```
sudo apt-get install openvpn
```

to get it.

Once OpenVPN is installed, connect to your instance by running

```
sudo openvpn --config ~/righthook.io.conf
```

and type in the VPN password you were provided.

The OpenVPN process takes up its own terminal and is not easily backgrounded or `nohup`-ed.  If you are working from the command line and have no GUI, we suggest using `screen` to manage multiple terminals ([screen documentation](https://help.ubuntu.com/community/Screen)).

To check if you have successfully connected, run `ifconfig`.  You should have an interface called `tun0` listed:

    tun0      Link encap:UNSPEC  HWaddr 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  
              inet addr:10.8.0.6  P-t-P:10.8.0.5  Mask:255.255.255.255
              UP POINTOPOINT RUNNING NOARP MULTICAST  MTU:1500  Metric:1
              RX packets:0 errors:0 dropped:0 overruns:0 frame:0
              TX packets:0 errors:0 dropped:0 overruns:0 carrier:0
              collisions:0 txqueuelen:100 
              RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)


The IP address listed under `inet addr` is your IP address on the virtual private network.  In subsequent steps, we assume your local machine is at `10.8.0.6`, as shown above.

The simulation instance is at `10.8.0.1` on the VPN.  You can verify connectivity using `ping 10.8.0.1`.

__IMPORTANT__: your regular internet traffic will _not_ go over the VPN.

## Starting and stopping the simulation

To start the simulation, send a request to `http://10.8.0.1:8080/launch_src` by navigating there in your web browser or running

```
curl http://10.8.0.1:8080/launch_src
```

*Note:* the first time you send this request on the instance, the simulation may take up to five minutes to start.


To stop the simulation, send a request to `http://10.8.0.1:8080/kill` by navigating there in your web browser or running

```
curl http://10.8.0.1:8080/kill
```

__IMPORTANT__: you must stop the simulation for your lap times to count.

## Connect to the ROS middleware

Once the simulator is running, it will publish sensor messages to which you can subscribe.  You must tell your local machine where the simulation instance is (`ROS_MASTER_URI` environment variable) and how the simulation instance can address your local machine (`ROS_IP` and `ROS_HOSTNAME` environment variables).  Do this by running:

    export ROS_MASTER_URI=http://10.8.0.1:11311
    export ROS_IP=10.8.0.6
    export ROS_HOSTNAME=10.8.0.6

Test your connectivity to the ROS server by running `rostopic list`, and you should get a list of topics published by the simulation.

On a machine with a GUI, you should be able to view the camera signal by running 

    rosrun image_view image_view image:=/kart/camera/image_raw

Some of the topics we publish are message structures that we have defined.  They are in the `rh_msgs` package that you installed.

## Put your controller in the loop using ROS

To control the car, create a ROS node that subscribes to the sensor messages you need, and publishes an actuation message.

### Sensor messages

The sensor messages available are:

* `/kart/camera/camera_info` (type `sensor_msgs/CameraInfo`): the `camera_info` message contains the parameters of the camera, including image height, width, projection matrix, and the name of the transform frame that gives the location and orientation of the camera.

* `/kart/camera/image_raw` (type `sensor_msgs/Image`): the `image_raw` message contains the live virtual camera image.  The camera image is 1280x720 encoded using `bayer_rggb8`.

* `/kart/camera/image_raw/*` (type varies): the `image_raw` namespace contains compressed versions of the camera image, which may be necessary depending on your bandwidth.  We found that the uncompressed `image_raw` uses ~24 MiB/s while the compressed `image_raw/compressed` uses ~12 MiB/s.  If this kind of bandwidth is unavailable to you, please [talk to us](mailto:support@righthook.io) about hosting your controller in the cloud.

* `/kart/collision` (type `rh_msgs/Collision`): the `collision` message is only published when the kart collides with something, and it's always a bad thing :(  If you receive one of these messages during a lap, that lap will be invalidated.

* `/kart/pose_wgs84/nav_sat_fix` (type `sensor_msgs/NavSatFix`): the `nav_sat_fix` gives the location of the kart in geospatial coordinates (latitude, longitude and altitude).

* `/kart/state` (type `rh_msgs/VehicleState`): the `/kart/state` gives some miscellaneous information about the kart's state, including its speed in the body frame (`velocity_x`, `velocity_y`, `velocity_z`); and the material under each wheel (`contact_material_name`).  If you receive a message where one or more wheels lists `Dirt` as the contact material, that lap will be invalidated.

* `/metric/timing_and_scoring` (type `std_msgs/String`): the `/metric/timing_and_scoring` message is a broadcast for you to see how your lap times will be scored.

* `/tf` (type `tf/tfMessage`): the `/tf`, or transform, message tells you where frames are relative to each other.  The inertial frame is called `/level`, and its origin is located at the start/finish line marshall's stand.  Its x-axis points east, y-axis north, and z-axis up.  The `/tf` message captures the transform from the inertial frame to the kart, and from the kart to its sensors.

* `/kart/imu` (type `sensor_msgs/Imu`): the `/kart/imu` message gives inertial measurements at the kart's center of mass, namely linear accelerations, angular velocities, and orientation.  The axes are the body frame.

### Actuation message

The actuation message that you will publish is at topic `/kart/actuation/actuation_request` (type `rh_msgs/VehicleActuation`).  You can view the message fields by running

    rosmsg show rh_msgs/VehicleActuation

The actuation fields are:

* `steering_degrees` (type `float32`): The is the angle the front wheels will be pointing.  Positive is left.  This can be in the range [-35.0, 35.0].

* `normalized_throttle` (type `float32`): This is the amount of throttle in the range [0.0, 1.0].

* `normalized_brake` (type `float32`): This is the amount of brake in the range [0.0, 1.0].  The brake only acts on the rear axle.  In the training data, our tame racing trainer did not need to use the brakes.

* `normalized_handbrake` (type `float32`): Probably pointless unless you like to feel like Colin McRae, this is handbrake in [0.0, 1.0].  It only acts on the rear axle.

* `reverse_gear_request` (type `bool`): When this is true, throttle will move you backwards.

For an example of how to pack and send the actuation message, check out the `joystick_test.py` script in the `rh_msgs` package (`/opt/ros/kinetic/lib/rh_msgs`).

## Pausing, unpausing, and resetting the simulation

For convenience and to help save you time, the simulation is running a ROS service that is capable of pausing, unpausing, and resetting the simulation.  We have included Python scripts that execute each one of these functions in the `rh_msgs` package.  You should be able to find them at `/opt/ros/kinetic/lib/rh_msgs`.  Consider automating reset using these service calls in your controller if you know your lap has been invalidated.  There are also some neat possibilities for reinforcement learning type stuff.

## Using the example controller

We have provided an example controller in this repo.  The controller uses a set path in x-y-z (taken from one of the training laps) and follows that path using a basic pure pursuit controller.

## Helpful guides and further reading for ROS

[Tutorials covering configuring your workspace, basic publish and subscribe](http://wiki.ros.org/ROS/Tutorials)

[Using rosbag for logging](http://wiki.ros.org/rosbag)

[Using OpenCV with ROS](http://wiki.ros.org/vision_opencv)

[ROS Services (like our pause, unpause and reset service)](http://wiki.ros.org/Services)

# FAQ

## I am having trouble getting it to work.

No worries, drop us a line at [support@righthook.io](mailto:support@righthook.io) and we'll do our best to get you up and running.

## What data are you collecting from the simulation?

We are only collecting the information that we need to run the simulation and score participants.  Specifically, we only collect: instance start and stop times, simulation start and stop times, the /tf message (to time a lap), the /kart/state message (to be sure you didn't go off-track on a timed lap), the /kart/collision message (to be sure you didn't hit anything on a timed lap), and the type of node connected to the actuation message (to be sure a human is not driving the car).

## Are you collecting camera and actuation data so you can train your own algorithm?

NOPE.  In the above question we list the only messages we collect, and you'll notice that camera and actuation are not among them.  More broadly, this is in line with the core mission of RightHook.  Our mission is not to develop automated systems; we exist to help _you_ develop automated systems.

## I found a bug.

_Please_ drop us a line at [feedback@righthook.io](mailto:feedback@righthook.io); we live for feedback and very much appreciate it.  Please include your instance ID (found in the portal) and if possible/applicable, the UUID of the simulation that was running.

## I have a feature request.

Cool, [let us know](mailto:feedback@righthook.io).

## I found the Easter Egg.

Then you know what to do :)
