/**
 ******************************************************************************
 * @file      syscalls.c
 * @brief     Minimal syscalls for arm-none-eabi (newlib-nano) + printf via USART0
 *            GD32F303CCT6 Pressure Relief Module
 ******************************************************************************
 */

#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <errno.h>
#include <stdint.h>

#define BSP_USE_UART_LOG
#include "bsp.h"

#undef errno
extern int errno;

char *__env[1] = { 0 };
char **environ = __env;

int _close(int file) { (void)file; return -1; }
int _execve(char *name, char **argv, char **env) { (void)name; (void)argv; (void)env; errno = ENOMEM; return -1; }
int _fork(void) { errno = EAGAIN; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _getpid(void) { return 1; }
int _isatty(int file) { (void)file; return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _link(char *old, char *new) { (void)old; (void)new; errno = EMLINK; return -1; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _open(const char *name, int flags, int mode) { (void)name; (void)flags; (void)mode; return -1; }
int _stat(const char *filepath, struct stat *st) { (void)filepath; st->st_mode = S_IFCHR; return 0; }
clock_t _times(struct tms *buf) { (void)buf; return -1; }
int _unlink(char *name) { (void)name; errno = ENOENT; return -1; }
int _wait(int *status) { (void)status; errno = ECHILD; return -1; }

int _read(int file, char *ptr, int len) {
    (void)file; (void)ptr; (void)len;
    return 0;
}

/**
 * @brief  Write to stdout/stderr via BSP UART Log (printf backend)
 */
int _write(int file, char *ptr, int len) {
    (void)file;
    return Bsp_UartLog_Write(ptr, (uint32_t)len);
}
