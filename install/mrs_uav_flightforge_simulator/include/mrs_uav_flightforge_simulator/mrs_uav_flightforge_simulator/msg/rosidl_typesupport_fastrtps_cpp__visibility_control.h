// generated from
// rosidl_typesupport_fastrtps_cpp/resource/rosidl_typesupport_fastrtps_cpp__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef MRS_UAV_FLIGHTFORGE_SIMULATOR__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
#define MRS_UAV_FLIGHTFORGE_SIMULATOR__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mrs_uav_flightforge_simulator __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_mrs_uav_flightforge_simulator __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mrs_uav_flightforge_simulator __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_mrs_uav_flightforge_simulator __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_CPP_BUILDING_DLL_mrs_uav_flightforge_simulator
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mrs_uav_flightforge_simulator ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mrs_uav_flightforge_simulator
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mrs_uav_flightforge_simulator ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_mrs_uav_flightforge_simulator
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mrs_uav_flightforge_simulator __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_mrs_uav_flightforge_simulator
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mrs_uav_flightforge_simulator __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mrs_uav_flightforge_simulator
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // MRS_UAV_FLIGHTFORGE_SIMULATOR__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
