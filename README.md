# dexhand_ros2_meta
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

### Launching RVIZ2 with the Joint State Publisher GUI
<img width="600" alt="Screenshot 2023-10-01 at 8 11 12 AM" src="https://github.com/iotdesignshop/dexhand_ros2_meta/assets/2821763/14a82e9d-45fc-4dc2-bb43-b03e8b692a72">

This launches RVIZ2 with the DexHand URDF as well as the Joint State Publisher GUI allowing you to experiment with all of the DOF's in the DexHand in an interactive manner.
```
ros2 launch dexhand_description display.launch.py
```

