#include <stdarg.h>

#ifdef 	DEBUG
#ifdef __cplusplus
extern "C" {
#endif
void thread_safe_printf(const char *format, ...);
void thread_safe_printf_newline(const char *format, ...);
void initialize_debug_printf();
#ifdef __cplusplus
}
#endif

#define PRINT(...) thread_safe_printf(__VA_ARGS__)
#define PRINTLN(...) thread_safe_printf_newline(__VA_ARGS__)
#else
#define PRINT(...)
#define PRINTLN(...)
#endif
