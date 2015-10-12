^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package whycon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2015-10-12)
------------------
* fixed email
* fixed email
* LCAS maintainers
* Merge branch 'master' of github.com:lrse/whycon
* cleanup launch files
* insist on settings axis until success
* Merge pull request `#2 <https://github.com/LCAS/whycon/issues/2>`_ from gestom/master
  Fancy Readme
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* Readme
* Readme
* Readme
* README.md - cite link
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme improved.
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* launchers
* fixes
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* cambios en conf
* angles, although probably not useful
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* keep mono.launch a general example
* quick fix for monochrome cameras; will actually make this better by taking advantage from it
* fix del robot state publisher
* changes
* config not correctly used
* switch some calculations to double, just in case
* disable ros flag missing in readme
* README again
* use /camera/image_rect_color by default
* readme update
* big change: detectors need not cleanup between detections and they ignore each other's circles for free; global cleanup is used only once at the end
  since it is quite fast; all console printing is now turned off and can be turned on during compilation
* improvements (better resetting); robot_pose_publisher
* e-mail update
* Merge branch 'master' into unstable
* ignore
* readme
* ignore
* launch files
* use cv::eigen
* set release flags
* indigo fixes
* add debug info
* README, again
* bug in local window search fixed
* improved readme
* fixed some warnings
* GSL is no longer a dependency: switched to OpenCV's eigen()
* fix circle refinement
* fix package.xml
* package is now both ROS and non-ROS. everything in a single repo
* make into ROS compatible package
* fix compile error
* delete leftover MAVLink stuff
* after tracking loss, allow for inspecting only a local window around last detected area (default behaviuor); report partial results when not all circles are localized
* calibrator: remove leftover stereo crap; allow calibration from pre-recorded images
* raise exception when axis file is not found
* removed old files
* remove mavlink/mavconn/pcl support
* oops
* change citation key
* citation updates
* enable defines in doxyfile
* whycon installation and cmake files
* Doxyfile and little update in readme
* parameters for setting camera resolution
* inner/outer-diameter clashes
* Merge branch 'master' into unstable
* bug fix for live camera input
* fixed MAVCONN. separated viewer on own executable using MAVCONN
* little argument parsing fix
* disable usage of two cameras by camera calibrator (does not work)
* simplify readme
* output help into sections
* report time for whole localization also
* number frames from start of capture
* initialize transform to identity
* many changes
  * more robust working for circles that disappear, explicit initialize step is gone.
  * randomized threshold and full undistort available as compile time options
* undistort map precomputed. not yet used nor tested
* simplified code
* fix drawing of ellipses (looks uglier but it is correct). remove old code
* fixed wrong (inverse) application of distortion model. interface is not final
* allow refinments to be made when not using camera. removed commented out tbb code
* debugging facilities
* drawing fix!
* allow specifying diameters on command line. do not require axis for tracking
* missing files. ignore updated
* more work for mavconn
* moved
* initial support for MAVCONN (not finished, but compiles). rearranged files
* removed old code
* re-enabled output writing
* separated executable in two modes: axis setting and tracking. added comments in circle_detector.cpp
* missing localization viewer files
* initial (not finished) support for 3D visualization using PCL (optional)
* removed some warnings
* typo
* support for more robust command line options handling
* circle was regenerated on inkscape and is now provided in SVG/PDF
* fixed readme
* citation
* change link order for some strange compilers
* fix for numerical problem when circle is aligned with optical axis
* lot of stuff commented out (couts). fixed problem with TBB headers
* make a shared library of the main sources
* mirrored XY circle pos (to follow pixel coords). auto detection of correct axis order (assuming first circle as 0,0). establish_error.rb script to measure error
* start circle search where previous valid circle was found. speeds up a bit
* Merge branch 'many'
  Conflicts:
  many_circle_detector.cpp
* faster drawing (and only during init)
* timings
* working version
* do not tag white pixels on main loop, solves obscure bug. also, paint white, to speedup ignoring other circles
* fast buffer cleanup
* add comments and remove segmentArray, great memory reduction
* nothing important
* better font sizes, reduce memory requirements a bit
* executable now takes calibration file as parameter
* pleace =b
* README
* add circle pattern to repo
* rename
* big rename, makes for sense
* cleanups, disabled ellipse improving since that needs testing
* localization system working, simple tests performed. needs accuracy report yet
* homography based computation implemented, needs further testing
* missing file
* readme
* fixes and ellipse improvement
* cleanup gui
* support for similarit transform
* more friendly output and fixed problem when not detecting circles
* make N attempts on every frame (currently 50) and fix little bug
* disable tbb for now
* fix, old code was in the way
* latest changes by tom integrated. to be tested
* save axis transform
* save axis pose, fix ellipse display
* calibration by opencv
* save frames when clicking, allow setting real world scale (NOTE: ratio was set to 6:5 for X,Y)
* Tested and working!
* localization system 99% complete
* localizer code (for many circles) using TBB/serial
* first working version with images
* Contributors: Marc Hanheide, Matias N., Thomas Fischer, Tom Krajnik, v01d

* fixed email
* fixed email
* LCAS maintainers
* Merge branch 'master' of github.com:lrse/whycon
* cleanup launch files
* insist on settings axis until success
* Merge pull request `#2 <https://github.com/LCAS/whycon/issues/2>`_ from gestom/master
  Fancy Readme
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* WhyCon versions
* Readme
* Readme
* Readme
* README.md - cite link
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme
* Readme improved.
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* launchers
* fixes
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* cambios en conf
* angles, although probably not useful
* Merge branch 'master' of robotica.dc.uba.ar:lrse/whycon
* keep mono.launch a general example
* quick fix for monochrome cameras; will actually make this better by taking advantage from it
* fix del robot state publisher
* changes
* config not correctly used
* switch some calculations to double, just in case
* disable ros flag missing in readme
* README again
* use /camera/image_rect_color by default
* readme update
* big change: detectors need not cleanup between detections and they ignore each other's circles for free; global cleanup is used only once at the end
  since it is quite fast; all console printing is now turned off and can be turned on during compilation
* improvements (better resetting); robot_pose_publisher
* e-mail update
* Merge branch 'master' into unstable
* ignore
* readme
* ignore
* launch files
* use cv::eigen
* set release flags
* indigo fixes
* add debug info
* README, again
* bug in local window search fixed
* improved readme
* fixed some warnings
* GSL is no longer a dependency: switched to OpenCV's eigen()
* fix circle refinement
* fix package.xml
* package is now both ROS and non-ROS. everything in a single repo
* make into ROS compatible package
* fix compile error
* delete leftover MAVLink stuff
* after tracking loss, allow for inspecting only a local window around last detected area (default behaviuor); report partial results when not all circles are localized
* calibrator: remove leftover stereo crap; allow calibration from pre-recorded images
* raise exception when axis file is not found
* removed old files
* remove mavlink/mavconn/pcl support
* oops
* change citation key
* citation updates
* enable defines in doxyfile
* whycon installation and cmake files
* Doxyfile and little update in readme
* parameters for setting camera resolution
* inner/outer-diameter clashes
* Merge branch 'master' into unstable
* bug fix for live camera input
* fixed MAVCONN. separated viewer on own executable using MAVCONN
* little argument parsing fix
* disable usage of two cameras by camera calibrator (does not work)
* simplify readme
* output help into sections
* report time for whole localization also
* number frames from start of capture
* initialize transform to identity
* many changes
  * more robust working for circles that disappear, explicit initialize step is gone.
  * randomized threshold and full undistort available as compile time options
* undistort map precomputed. not yet used nor tested
* simplified code
* fix drawing of ellipses (looks uglier but it is correct). remove old code
* fixed wrong (inverse) application of distortion model. interface is not final
* allow refinments to be made when not using camera. removed commented out tbb code
* debugging facilities
* drawing fix!
* allow specifying diameters on command line. do not require axis for tracking
* missing files. ignore updated
* more work for mavconn
* moved
* initial support for MAVCONN (not finished, but compiles). rearranged files
* removed old code
* re-enabled output writing
* separated executable in two modes: axis setting and tracking. added comments in circle_detector.cpp
* missing localization viewer files
* initial (not finished) support for 3D visualization using PCL (optional)
* removed some warnings
* typo
* support for more robust command line options handling
* circle was regenerated on inkscape and is now provided in SVG/PDF
* fixed readme
* citation
* change link order for some strange compilers
* fix for numerical problem when circle is aligned with optical axis
* lot of stuff commented out (couts). fixed problem with TBB headers
* make a shared library of the main sources
* mirrored XY circle pos (to follow pixel coords). auto detection of correct axis order (assuming first circle as 0,0). establish_error.rb script to measure error
* start circle search where previous valid circle was found. speeds up a bit
* Merge branch 'many'
  Conflicts:
  many_circle_detector.cpp
* faster drawing (and only during init)
* timings
* working version
* do not tag white pixels on main loop, solves obscure bug. also, paint white, to speedup ignoring other circles
* fast buffer cleanup
* add comments and remove segmentArray, great memory reduction
* nothing important
* better font sizes, reduce memory requirements a bit
* executable now takes calibration file as parameter
* pleace =b
* README
* add circle pattern to repo
* rename
* big rename, makes for sense
* cleanups, disabled ellipse improving since that needs testing
* localization system working, simple tests performed. needs accuracy report yet
* homography based computation implemented, needs further testing
* missing file
* readme
* fixes and ellipse improvement
* cleanup gui
* support for similarit transform
* more friendly output and fixed problem when not detecting circles
* make N attempts on every frame (currently 50) and fix little bug
* disable tbb for now
* fix, old code was in the way
* latest changes by tom integrated. to be tested
* save axis transform
* save axis pose, fix ellipse display
* calibration by opencv
* save frames when clicking, allow setting real world scale (NOTE: ratio was set to 6:5 for X,Y)
* Tested and working!
* localization system 99% complete
* localizer code (for many circles) using TBB/serial
* first working version with images
* Contributors: Marc Hanheide, Matias N., Thomas Fischer, Tom Krajnik, v01d
