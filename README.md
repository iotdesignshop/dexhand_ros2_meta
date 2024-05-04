# dexhand_ros2_meta
![DEXHAND-ROS](https://github.com/iotdesignshop/dexhand_ros2_meta/assets/2821763/7ada5897-f3f1-4629-a548-4b434183bc92)

Metapackage for DexHand ROS 2 Packages. Used to manage the collection of ROS 2 packages and environment for the DexHand humanoid robot hand.

## Overview
The DexHand is an open-source humanoid robot hand. You likely already know that if you landed here, but if not, you can visit our project webpage to get a bit more context on the project and various packages and components: http://www.dexhand.org

This package, in particular, will help you to gather together a workspace and environment for building and running the ROS 2 packages to control either a real or simulated hand. 

## Pre-Requisires and Dependencies

### ROS 2 Humble Installation

Currently, we build and test our ROS 2 packages against [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on Ubuntu Linux 22.04. You may be able to get the packages running on different versions of ROS, or on different platforms but we haven't tested those and likely won't be able to provide a ton of advice or troubleshooting help.

### DexHand Hardware

DexHand hardware isn't an absolute requirement, as we've created basic simulation packages, and intend to publish more. Support for NVidia Isaac Sim is in the works. For now, we have RVIZ2 visualizations and simulations for the packages, so that can help to get you up and running. Our hardware build wiki for making your own DexHand [can be found here](https://github.com/iotdesignshop/dexhand-mechanical-build/wiki). We are continuing to make the designs easier to build with each iteration of the hand, so hopefully you will become inspired to assemble your own!

## ROS 2 Package Overview

This metapackage depends on a number of ROS 2 packages that we maintain on GitHub. This list provides links and an overview for each package, but more detailed information about the packages is available in the README files contained within each repo.

* [**dexhand_description**](https://github.com/iotdesignshop/dexhand_description): URDF files, and RVIZ2 launch files for viewing the DexHand URDF in RVIZ2. 

* [**dexhand_gesture_controller**](https://github.com/iotdesignshop/dexhand_gesture_controller): High level animation interface, gesture controller, simulation driver, and hardware driver for high level control of a DexHand. This package lets you drive either simulated or hardware DexHand's into poses with an easy high level interface and is a good study on how to pose the hand or as a foundation for other types of controls (such as our LLM controller below).

* [**dexhand_usb_serial**](https://github.com/iotdesignshop/dexhand_usb_serial): Provides serial communication via USB between ROS 2 and the Arduino-based DexHand firmware running on the hand (available in the [dexhand-ble repo on GitHub](https://github.com/iotdesignshop/dexhand-ble)). This package is used by dexhand_gesture_controller to send gestures over to the hardware device, but provides general access to the serial command structure available on the hand.

* [**dexhand_llm_control**](https://github.com/iotdesignshop/dexhand_llm_control): Provides a bridge between ChatGPT (GPT-4) and the DexHand Gesture Controller to enable experiments where you use the DexHand as an output device. Ask it math questions, to do hand gestures, like the peace sign, or other questions and see how GPT-4 interprets and uses the hand as an output device. [Check out our demo video on YouTube](https://youtu.be/GWHLRgOuJLU).

## How to Use This Meta Package

### Concept

The core purpose of the dexhand_ros2_meta package is that it should provide a set of scripts and tools to make it easy to set up a ROS 2 Workspace with all of the dependent packages so that you can easily launch higher level tasks that you want the DexHand systems to perform, such as launching RVIZ2 with a simulated hand, or connecting to actual hardware and so forth.

### Usage

First, you clone this repo to your local machine:

```
git clone https://github.com/iotdesignshop/dexhand_ros2_meta.git
```

Then, change into the directory and make the __setup.sh__ script executable:

```
cd dexhand_ros2_meta
chmod +x scripts/setup.sh
```

Finally run the script with an argument showing where you would like your DexHand workspace created:
```
scripts/setup.sh <Desired Workspace Path>
```

For example:
```
scripts/setup.sh ~/dexhand_ws
```

Once installation is complete, you can switch to your workspace, source the environment and you should be ready to go:
```
cd <Your Workspace Path>
source install/setup.bash
```

At that point, you should be ready to use the packages and the launch files contained within them, see the next section for some examples.


## Some Example Use Cases

These examples below all launch hand visualizations and simualtions using the RVIZ2 tool from ROS 2. Additional notes are provided on how to connect to and use DexHand hardware below. You should be able to try out all the demos in this section without a DexHand device, so feel free to experiment!


### Launching RVIZ2 with the Joint State Publisher GUI
<img width="600" alt="Screenshot 2023-10-01 at 8 11 12 AM" src="https://github.com/iotdesignshop/dexhand_ros2_meta/assets/2821763/14a82e9d-45fc-4dc2-bb43-b03e8b692a72">


This launches RVIZ2 with the DexHand URDF as well as the Joint State Publisher GUI allowing you to experiment with all of the DOF's in the DexHand in an interactive manner. This functionality is provided by the [dexhand_description package](https://github.com/iotdesignshop/dexhand_description). 


```
ros2 launch dexhand_description display.launch.py
```

### Launching the Gesture Controller

<img width="600" alt="Screenshot 2023-10-01 at 8 28 26 AM" src="https://github.com/iotdesignshop/dexhand_ros2_meta/assets/2821763/a0e38771-86b2-417d-bf03-6f1a9338ab0e">


To get started with the DexHand, we provide a high level gesture controller which allows you to easily specify common hand poses and control the fingers with a simplified interface. We also sometimes call this the "semantic" interface because you're providing descriptive high level poses to the hand as opposed to detailed finger positions. 

This functionality is provided by the [dexhand_gesture_controller package](https://github.com/iotdesignshop/dexhand_gesture_controller). More detailed information is available there on the commands and functionality of the package.

The DexHand gesture controller takes care of driving ROS 2 messages for both the simulated version of the hand in RVIZ2 as well as actual DexHand hardware if you have it connected to the host via USB. There is more information on using physical hands down below.

To launch RVIZ2 and the Gesture Controller, you can use the following command:

```
ros2 launch dexhand_gesture_controller simulation.launch.py
```

Once that is running, you can open a second ROS 2 Terminal, source your environment, and issue gesture commands. Lots of different poses are available including "fist, peace, horns, shaka". You can try them out in the sim to see how they work, and of course, we highly recommend reviewing the code in the package to get a better understanding of what we are doing. 


As an example, to form a fist:
```
ros2 topic pub -1 /dexhand_gesture std_msgs/msg/String "data: 'fist'"
```

And to return back to base pose:
```
ros2 topic pub -1 /dexhand_gesture std_msgs/msg/String "data: 'reset'"
```

### ChatGPT/LLM Control of the DexHand

[![DexHand LLM](https://github.com/iotdesignshop/dexhand-mechanical-build/blob/main/docs/images/web-general/llm-vid-thumb.png?raw=true)](https://youtu.be/GWHLRgOuJLU)
This is a video of the demo from YouTube.

As an experimental package, we have created an interface between ChatGPT and the DexHand as a demonstration of what is possible when using the DexHand as an output device for a LLM. Some truly interesting and unique emergent behavior occurs when you allow GPT-4 to control the DexHand. 

This functionality is provided by the [dexhand_llm_control package](https://github.com/iotdesignshop/dexhand_llm_control). Much more detail is provided there in terms of the system, hardware, and software interfaces. 

**NOTE: To run this package, you will need to have an OpenAI API key set in your environment. The dexhand_llm_control package describes how to set this up**

Once you have the API key and environment set up, you can run the demonstration as follows:
```
ros2 launch dexhand_llm_control simulation.launch.py
```

## Using DexHand Hardware

The DexHand hardware is also open source and can be made with relatively inexpensive parts and 3D printing. If you haven't already visited the site, you can get more information on this at http://www.dexhand.org.

Firmware is provided for the DexHand to run on the Arduino Nano RP2040 Connect, which is a Raspberry Pi Pico-based board available from Arduino. The firmware and a Python interface for connecting to it and testing it is provided in the [dexhand-ble package](https://github.com/iotdesignshop/dexhand-ble) on GitHub. 

Assuming you have a working DexHand, and the firware installed. The interface between ROS 2 and the DexHand firmware is provided by the [dexhand_usb_serial package](https://github.com/iotdesignshop/dexhand_usb_serial). This package creates a bridge for sending commands across the USB port to the DexHand hardware. 

As noted above, the Dexhand gesture controller package actually generates hardware messages as well as simulation events for driving the joints in RVIZ2. To start streaming these commands to the DexHand hardware, all you need to do is to start the USB Serial Node.

To do so, launch the Dexhand gesture controller as per the instructions above, then open an additional terminal window, source your workspace environment and run the following command (assuming your Arduino board is connected to /dev/ttyACM0, which is the default - you can override this on the command line if it is not):

```
ros2 run dexhand_usb_serial usb_serial
```

With that node running - events from the gesture controller should stream to the hand hardware as well as the simulated hands.


## MANUS VR Glove Support

We recently added support for the MANUS VR Glove SDK in the DexHand packages. This is an optional install as it does require authenticated access to download and install the MANUS SDK, so we didn't want to just assume that by default. However, if you do have MANUS VR Gloves and an account, you can use them to control the DexHand Simulator and Hardware. 

### Installation Process

We assume you have already completed the installation process described at the top of this README for setting up a workspace with the DexHand packages. The MANUS setup occurs in the same folder and adds a few packages. 

Change back into the directory where you cloned the dexhand_ros2_meta package and make the __setup_manus.sh__ script executable:

```
cd dexhand_ros2_meta
chmod +x scripts/setup_manus.sh
```

Run the script with the same argument showing where you created your DexHand workspace:
```
scripts/setup_manus.sh <Desired Workspace Path>
```

For example:
```
scripts/setup_manus.sh ~/dexhand_ws
```

After that, you will need to download and add the Manus SDK to the `/ext` folder of the `manus_ros2` project. This requires a Manus account and login, which you can create on the Manus site. You likely already have one if you've installed Manus Core on your PC. 

https://www.manus-meta.com/resources/downloads/overview

Once installation is complete, you can switch to your workspace, build the new packages, source the environment and you should be ready to go:
```
cd <Your Workspace Path>
colcon build --symlink-install
source install/setup.bash
```

To launch the Manus nodes along with a simulation of the DexHand for testing, you can run:
```
ros2 launch dexhand_manus simulation.launch.py
```





