#ifndef CNOID_OPENVR_PLUGIN_EXPORTDECL_H
# define CNOID_OPENVR_PLUGIN_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_OPENVR_PLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_OPENVR_PLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_OPENVR_PLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_OPENVR_PLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_OPENVR_PLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_OPENVR_PLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_OPENVR_PLUGIN_DLLIMPORT
#   define CNOID_OPENVR_PLUGIN_DLLEXPORT
#   define CNOID_OPENVR_PLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_OPENVR_PLUGIN_STATIC
#  define CNOID_OPENVR_PLUGIN_DLLAPI
#  define CNOID_OPENVR_PLUGIN_LOCAL
# else
#  ifdef CnoidOpenVRPlugin_EXPORTS
#   define CNOID_OPENVR_PLUGIN_DLLAPI CNOID_OPENVR_PLUGIN_DLLEXPORT
#  else
#   define CNOID_OPENVR_PLUGIN_DLLAPI CNOID_OPENVR_PLUGIN_DLLIMPORT
#  endif
#  define CNOID_OPENVR_PLUGIN_LOCAL CNOID_OPENVR_PLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_OPENVR_PLUGIN_DLLAPI
