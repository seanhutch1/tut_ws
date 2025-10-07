#ifndef IAR_DIJKSTRA_PLANNER__VISIBILITY_CONTROL_H_
#define IAR_DIJKSTRA_PLANNER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IAR_DIJKSTRA_PLANNER_EXPORT __attribute__ ((dllexport))
    #define IAR_DIJKSTRA_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define IAR_DIJKSTRA_PLANNER_EXPORT __declspec(dllexport)
    #define IAR_DIJKSTRA_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef IAR_DIJKSTRA_PLANNER_BUILDING_LIBRARY
    #define IAR_DIJKSTRA_PLANNER_PUBLIC IAR_DIJKSTRA_PLANNER_EXPORT
  #else
    #define IAR_DIJKSTRA_PLANNER_PUBLIC IAR_DIJKSTRA_PLANNER_IMPORT
  #endif
  #define IAR_DIJKSTRA_PLANNER_PUBLIC_TYPE IAR_DIJKSTRA_PLANNER_PUBLIC
  #define IAR_DIJKSTRA_PLANNER_LOCAL
#else
  #define IAR_DIJKSTRA_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define IAR_DIJKSTRA_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define IAR_DIJKSTRA_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define IAR_DIJKSTRA_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IAR_DIJKSTRA_PLANNER_PUBLIC
    #define IAR_DIJKSTRA_PLANNER_LOCAL
  #endif
  #define IAR_DIJKSTRA_PLANNER_PUBLIC_TYPE
#endif

#endif  // IAR_DIJKSTRA_PLANNER__VISIBILITY_CONTROL_H_
