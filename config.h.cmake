#ifndef __CONFIG_H__
#define __CONFIG_H__

#cmakedefine PCL_FOUND
#cmakedefine ENABLE_3D

#if defined(PCL_FOUND) && defined(ENABLE_3D)
#define ENABLE_VIEWER
#endif

#cmakedefine ENABLE_MAVCONN

#cmakedefine ENABLE_FULL_UNDISTORT
#cmakedefine ENABLE_RANDOMIZED_THRESHOLD

#endif
