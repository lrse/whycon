Description
==========

_WhyCon_ is a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
These characteristics enable it to be used as an alternative to more expensive systems available for research communities.

Ease of use and setup are also important features of this software. Furthermore, there are plans for this software to be eventually included in the OpenCV library and mobile robotics frameworks such as MAVLINK/MAVCONN and ROS.

The localization system works by detecting planar ring-shaped targets printed on a B/W paper. With a single camera, the 3D position plus two rotation angles (5DoF) of a single target can be obtained. However, the reference of this target pose is the camera itself. Since in most conditions the user will likely want to define its own coordinate system, there is simple initial configuration/calibration step that needs to be performed. After this step, many targets/robots can be tracked.

Status and Planned Improvements
===========================

The system is currently defined for targets/robots moving on a plane (generally, the ground plane). Full 3D pose and working area are work in progress.

Publication and citation
=====================

_WhyCon_ was first presented on ICRA2013 conference and will be published as a full article soon, where the details of the algorithm will be presented.
In the meantime, if you use this software for your publication please use the following URL for citations: [http://purl.org/robotics/whycon].

Installation
===========

The installation process is really straightforward, as with any CMake based project.
Once you downloaded the sources, enter the source directory and perform:

    mkdir build
    cd build
    cmake ..
    make

Then, inside this build directory you will find two executables: `calibrator` and `localization-system`. The first executable permits creating an XML-based calibration file, containing all camera's instrict parameters. This will be explained further below. The second executable consists of the localization system itself. Please read the appropriate sections for each program.

Usage
====

Please go to the WhyCon Wiki, [https://github.com/v01d/whycon/wiki], for a brief tutorial on how to setup the localization system, and how to use the calibrator.


