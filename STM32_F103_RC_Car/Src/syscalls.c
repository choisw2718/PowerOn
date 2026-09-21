#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

char *__env[1] = { 0 };
char **environ = __env;

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1)
    {
    }
}

void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end;
    extern uint8_t _estack;
    extern uint32_t _Min_Stack_Size;
    static uint8_t *heap_end;
    uint8_t *prev_heap_end;
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;

    if (heap_end == NULL)
    {
        heap_end = &_end;
    }

    if ((uint32_t)(heap_end + incr) > stack_limit)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev_heap_end = heap_end;
    heap_end += incr;
    return prev_heap_end;
}
