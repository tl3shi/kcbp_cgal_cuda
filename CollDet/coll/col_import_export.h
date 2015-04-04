
#if defined(_WIN32)
#	ifdef COL_EXPORT
#   	define COL_EXPORTIMPORT __declspec(dllexport)
#	else
#   	define COL_EXPORTIMPORT __declspec(dllimport)
#	endif
#else
#   define COL_EXPORTIMPORT
#endif

#if defined(_WIN32)
// on windows, this replaces the "unsafe" version of functions
// like strcpy by "safe" version (e.g., strcpy_s)
#define _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES 1
#endif
