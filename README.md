# Description

_WhyCon_ is a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.

These characteristics enable it to be used as an alternative to more expensive systems available for research communities. 

The system is capable of detecting several targets very efficiently which are detected by a video camera. This can be used as a source for ground-truth data
for experiments or for on-line pose information.

WhyCon is proposed as an alternative to widely used and expensive localization systems. It is fully open-source.

# Publication and citation

If you use this software for your publication it is mandatory to cite using the references in the provided `cite.bib`
file (you can download it [here](https://raw.github.com/lrse/whycon/master/cite.bib)).

Note that this .bib includes not only the references to the scientific works that describe the underlying method, but
also a reference to the implementation for a specific (stable) version of the code on GitHub (look for the DOI containing
the word "zenodo").

# Installation

The code can be compiled either as a ROS package (shared library) or in a standalone version.

**NOTE**: while the standalone version includes a demo application, this demo is not actively maintained anymore and will probably be removed soon. The reference
application is provided as a series of ROS nodes, which utilize the whycon shared library. For an example of how to implement your own standalone application, the ROS
node should be used as a reference.

Stable [releases](https://github.com/lrse/whycon/releases) are available periodically. Latest stable release can be downloaded by clicking [here](https://github.com/lrse/whycon/releases/latest). 

For the latest development version (which should also work and may contain new experimental features) you can clone the repository directly.

## ROS

The main directory should be placed inside a catkin workspace source-space (eg: ~catkin_ws/src).
It can then be compiled simply by:

    catkin_make

## Standalone

The standalone version requires you to take care of installing the correct dependencies: OpenCV and Boost. If you are on ubuntu, simply perform the following:

    sudo apt-get install libopencv-dev libboost-all-dev

The installation process is really straightforward, as with any CMake based project.
Inside the main directory do:

    mkdir build
    cd build
    cmake ..
    make

The code can be installed to the system by doing:

    make install

Note the default CMake location is `/usr/local`, but you can redefine this by invoking cmake in this way instead:

    cmake -DCMAKE_INSTALL_PREFIX=/usr ..

# Usage Manual

Please refer to the [wiki](https://github.com/lrse/whycon/wiki).
