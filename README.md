# Description

_WhyCon_ is a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
These characteristics enable it to be used as an alternative to more expensive systems available for research communities.

# Publication and citation

_WhyCon_ was first presented on ICRA2013 conference and accepted for an extended version in the JINT journal.

If you use this software for your publication it is mandatory to cite using the references in the provided `cite.bib`
file (you can download it [here](https://raw.github.com/lrse/whycon/master/cite.bib)).

Note that this .bib includes not only the references to the scientific works that describe the underlying method, but
also a reference to the implementation for a specific (stable) version of the code on GitHub (look for the DOI containing
the word "zenodo").

# Installation

The code can be compiled either as a ROS package or in a standalone version.

## ROS (precompiled)

If you are reading this, you probably want to compile the code from source. If you actually want to simply install a precompiled
version of the code, use your standard installation tool for your distribution.

In Ubuntu you should do:

    sudo apt-get install ros-hydro-whycon

In ArchLinux you should do:

    yaourt -S --noconfirm ros-hydro-whycon

## ROS (from source)

The main directory should be placed inside a catkin workspace source-space (eg: ~catkin_ws/src).
It can then be compiled simply by:

    catkin_make

For detailed documentation of the provided ROS nodes, please see the corresponding documentation at the [ROS wiki](http://wiki.ros.org/whycon)

## Standalone

The installation process is really straightforward, as with any CMake based project.
Inside the main directory do:

    mkdir build
    cd build
    cmake ..
    make

If this fails, you may not have installed the required dependencies: OpenCV and Boost. If you are on ubuntu, simply perform the following:

    sudo apt-get install libopencv-dev libboost-all-dev

and then repeat the `cmake` and `make` steps.

The code can be installed to the system by doing:

    make install

Note the default CMake location is `/usr/local`, but you can redefine this by invoking cmake in this way instead:

    cmake -DCMAKE_INSTALL_PREFIX=/usr ..

# Usage Manual

Please refer to the [wiki](https://github.com/lrse/whycon/wiki).
