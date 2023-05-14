#ifndef POLYGONS__VISIBILITY_CONTROL_H_
#define POLYGONS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
//! See that reference to understand why these macros are necessary to optimize
//! both code and generated binaries

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POLYGONS_EXPORT __attribute__ ((dllexport))
    #define POLYGONS_IMPORT __attribute__ ((dllimport))
  #else
    #define POLYGONS_EXPORT __declspec(dllexport)
    #define POLYGONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef POLYGONS_BUILDING_LIBRARY
    #define POLYGONS_PUBLIC POLYGONS_EXPORT
  #else
    #define POLYGONS_PUBLIC POLYGONS_IMPORT
  #endif
  #define POLYGONS_PUBLIC_TYPE POLYGONS_PUBLIC
  #define POLYGONS_LOCAL
#else
  //! In declarations, you'd use these two when building the library, and
  //! when using it from outside (including its header which includes this)
  #define POLYGONS_EXPORT __attribute__ ((visibility("default")))
  #define POLYGONS_IMPORT
  #if __GNUC__ >= 4
    //! In definitions, while writing the library code, you'd use these
    //! to mark stuff that must be visible from outside the DSO and stuff that
    //! must not be so
    #define POLYGONS_PUBLIC __attribute__ ((visibility("default")))
    #define POLYGONS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POLYGONS_PUBLIC
    #define POLYGONS_LOCAL
  #endif
  #define POLYGONS_PUBLIC_TYPE
#endif

#endif  // POLYGONS__VISIBILITY_CONTROL_H_
