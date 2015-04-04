
#if defined(WIN32)
#	ifdef QH_EXPORT
#   	define QH_EXPORTIMPORT __declspec(dllexport)
#	else
#   	define QH_EXPORTIMPORT __declspec(dllimport)
#	endif
#else
#   define QH_EXPORTIMPORT
#endif

#if defined(WIN32)
// on windows, this replaces the "unsafe" version of functions
// like strcpy by "safe" version (e.g., strcpy_s)
#define _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES 1
#endif
