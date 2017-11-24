#ifndef PEDI2_EXPORTDECL_H_INCLUDED
# define PEDI2_EXPORTDECL_H_INCLUDED
# if defined(_WIN32) || defined (__CYGWIN__)
#  define PEDI2_DLLIMPORT __declspec(dllimport)
#  define PEDI2_DLLEXPORT __declspec(dllexport)
# else
#  if __GNUC__ >= 4
#   define PEDI2_DLLIMPORT __attribute__ ((visibility("default")))
#   define PEDI2_DLLEXPORT __attribute__ ((visibility("default")))
#  else
#   define PEDI2_DLLIMPORT
#   define PEDI2_DLLEXPORT
#  endif
# endif

# ifdef PEDI2_EXPORTS
#  define PEDI2_DLLAPI PEDI2_DLLEXPORT
# else
#  define PEDI2_DLLAPI PEDI2_DLLIMPORT
# endif

#endif

#ifdef __EXPORT
#undef __EXPORT
#endif
#define __EXPORT PEDI2_DLLAPI
