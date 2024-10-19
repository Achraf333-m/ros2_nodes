#ifndef MOVE_ROVER_CPP__VISIBILITY_CONTROL_H_
#define MOVE_ROVER_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVE_ROVER_CPP_EXPORT __attribute__ ((dllexport))
    #define MOVE_ROVER_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVE_ROVER_CPP_EXPORT __declspec(dllexport)
    #define MOVE_ROVER_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVE_ROVER_CPP_BUILDING_DLL
    #define MOVE_ROVER_CPP_PUBLIC MOVE_ROVER_CPP_EXPORT
  #else
    #define MOVE_ROVER_CPP_PUBLIC MOVE_ROVER_CPP_IMPORT
  #endif
  #define MOVE_ROVER_CPP_PUBLIC_TYPE MOVE_ROVER_CPP_PUBLIC
  #define MOVE_ROVER_CPP_LOCAL
#else
  #define MOVE_ROVER_CPP_EXPORT __attribute__ ((visibility("default")))
  #define MOVE_ROVER_CPP_IMPORT
  #if __GNUC__ >= 4
    #define MOVE_ROVER_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define MOVE_ROVER_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVE_ROVER_CPP_PUBLIC
    #define MOVE_ROVER_CPP_LOCAL
  #endif
  #define MOVE_ROVER_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MOVE_ROVER_CPP__VISIBILITY_CONTROL_H_