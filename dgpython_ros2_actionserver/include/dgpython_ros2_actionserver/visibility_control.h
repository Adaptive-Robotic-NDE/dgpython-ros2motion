#ifndef DGPYTHON_ROS2_ACTIONSERVER__VISIBILITY_CONTROL_H_
#define DGPYTHON_ROS2_ACTIONSERVER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DGPYTHON_ROS2_ACTIONSERVER_EXPORT __attribute__ ((dllexport))
    #define DGPYTHON_ROS2_ACTIONSERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define DGPYTHON_ROS2_ACTIONSERVER_EXPORT __declspec(dllexport)
    #define DGPYTHON_ROS2_ACTIONSERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DGPYTHON_ROS2_ACTIONSERVER_BUILDING_DLL
    #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC DGPYTHON_ROS2_ACTIONSERVER_EXPORT
  #else
    #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC DGPYTHON_ROS2_ACTIONSERVER_IMPORT
  #endif
  #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC_TYPE DGPYTHON_ROS2_ACTIONSERVER_PUBLIC
  #define DGPYTHON_ROS2_ACTIONSERVER_LOCAL
#else
  #define DGPYTHON_ROS2_ACTIONSERVER_EXPORT __attribute__ ((visibility("default")))
  #define DGPYTHON_ROS2_ACTIONSERVER_IMPORT
  #if __GNUC__ >= 4
    #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC __attribute__ ((visibility("default")))
    #define DGPYTHON_ROS2_ACTIONSERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC
    #define DGPYTHON_ROS2_ACTIONSERVER_LOCAL
  #endif
  #define DGPYTHON_ROS2_ACTIONSERVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DGPYTHON_ROS2_ACTIONSERVER__VISIBILITY_CONTROL_H_