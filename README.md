# WhyCon
## A precise, efficient and low-cost localization system

_WhyCon_ is a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
These characteristics allow its use as an alternative to more expensive localization systems available. 
The system is capable of efficient real-time detection and precise position estimation of several circular markers in a video stream. 
It can be used both off-line, as a source of ground-truth for robotics experiments, or on-line as a component of robotic systems that require real-time, precise position estimation.
_WhyCon_ is meant as an alternative to widely used and expensive localization systems. It is fully open-source.


| WhyCon example application (video)  | Scenario description |
| ------ | ----------- |
|[![WhyCon applications](https://raw.githubusercontent.com/wiki/gestom/WhyCon/pics/whycon.png)](https://www.youtube.com/watch?v=KgKrN8_EmUA"AAAA")|-precise docking to a charging station (EU project STRANDS),<br/> -fitness evaluation for self-evolving robots (EU proj. SYMBRION),<br/>-relative localization of UAV-UGV formations (CZ-USA project COLOS),<br/>-energy source localization in (EU proj REPLICATOR),<br/>-robotic swarm localization (EU proj HAZCEPT).|

The _WhyCon_ system was developed as a joint project between the University of Buenos Aires, Czech Technical University and University of Lincoln, UK.
The main contributors were [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao), [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) and [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en). Each of these contributors maintains a slightly different version of WhyCon.

| WhyCon version  | Application | Main features | Maintainer|
| --------------- | ----------- | ------ | ----- |
| [WhyCon-ROS](https://github.com/lrse/whycon) | general | 2D, 3D, ROS + lightweight | [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao) |
| [WhyCon-Mini](http://labe.felk.cvut.cz/~tkrajnik/circle_detector/) | general | 2D, 3D, lightweight, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao)|
| [SwarmCon](https://github.com/gestom/CosPhi/tree/master/Localization) | μ-swarms | 2D, individual IDs, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |
| [Caspa-WhyCon](http://robotics.fel.cvut.cz/faigl/caspa/) | UAVs | embedded, open HW-SW solution | [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) |
| [Social-card](https://github.com/strands-project/strands_social/tree/hydro-devel/social_card_reader) | HRI | orientation translated to commands  | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |

#### How does it work ?

A six-page overview of the <i>WhyCon</i> system was first presented at the ICAR conference [[1](#references)].
A detailed description was published in the Journal of Intelligent and Robotics Systems [[2](#references)].
An overview of _WhyCon's_ applications was presented at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015 [[3](#references)].
Nowadays, the system is being used in several research projects the across globe, see the following video for examples.

#### Can I use it ?
 
If you decide to use this _WhyCon_ for your research, please cite it using the one of the references provided in this [bibtex](https://github.com/lrse/whycon/blob/master/cite.bib) file.
Note that this .bib includes not only the references to the scientific works that describe the underlying method, but also a reference to the implementation for a specific (stable) version of the code on GitHub (look for the DOI containing the word "zenodo").

------
### References
1. T. Krajník, M. Nitsche et al.: <b>[External localization system for mobile robotics.](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2013_icar_whycon.pdf)</b> International Conference on Advanced Robotics (ICAR), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2013_icar_whycon.bib)].
2. T. Krajník, M. Nitsche et al.: <b>[A Practical Multirobot Localization System.](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2015_JINT_whycon.pdf)</b> Journal of Intelligent and Robotic Systems (JINT), 2014. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2015_JINT_whycon.bib)].
3. M. Nitsche, T. Krajník et al.: <b>[WhyCon: An Efficient, Marker-based Localization System.](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2015_irososar_whycon.pdf)</b> IROS Workshop on Open Source Aerial Robotics, 2015. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/WhyCon/papers/2015_irososar_whycon.bib)].
4. J. Faigl, T. Krajník et al.: <b>[Low-cost embedded system for relative localization in robotic swarms.](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6630694)</b> International Conference on Robotics and Automation (ICRA), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icra_whycon.bib)].

----

## Installing WhyCon

The code can be compiled as a ROS 2 package

Stable [releases](https://github.com/arunser/whycon2/releases) are available periodically. Latest stable release can be downloaded by clicking [here](https://github.com/arunser/whycon2/releases/latest). 

For the latest development version (which should also work and may contain new experimental features) you can clone the repository directly.

### ROS 2

Only LTS versions are targeted. At the moment, only Humble is targeted.

#### Dependencies

It is recommended to install required dependencies using

    sudo apt install ros-humble-usb-cam
    sudo apt install ros-humble-image-view

#### Compilation

The main directory should be placed inside a colcon workspace source-space (e.g.: ~whycon_ws/src).
It can then be build using:
    
    cd ~whycon_ws/
    colcon build


## Using WhyCon

Please refer to the [wiki](https://github.com/arunser/whycon2/wiki).

----

### Acknowledgements



