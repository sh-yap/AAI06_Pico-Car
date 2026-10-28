/* The motion task must not print during the control loop. */
#include <tk/tkernel.h>
INT tm_printf(const UB *format, ...);
