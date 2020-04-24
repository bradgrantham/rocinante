/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <reent.h>
#include <unistd.h>
#include <sys/wait.h>

#include "ff.h"

#undef errno
extern int errno;

#undef FreeRTOS
#define MAX_STACK_SIZE 0x200

extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));


caddr_t _sbrk(int incr)
{
	static char *heap_end;
	extern char heap_low;
	extern char heap_top;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &heap_low;

	prev_heap_end = heap_end;

	if (heap_end + incr > &heap_top)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

/*
 * _gettimeofday primitive (Stub function)
 * */
int _gettimeofday (struct timeval * tp, struct timezone * tzp)
{
  /* Return fixed data for the timezone.  */
  if (tzp)
    {
      tzp->tz_minuteswest = 0;
      tzp->tz_dsttime = 0;
    }

  return 0;
}

void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}
}

#define MAX_FILES 8
enum { FD_OFFSET = 3};
static FIL files[MAX_FILES];    /* starting with fd=3, so fd 3 through 10 */
static int filesOpened[MAX_FILES];

int _write(int file, char *ptr, int len)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	int DataIdx;

		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
		   __io_putchar( *ptr++ );
		}
	return len;
    } else {
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX write: file not opened\n");
            errno = EBADF;
            return -1;
        }
        unsigned int wrote;
        FRESULT result = f_write(&files[myFile], ptr, len, &wrote);
        if(result != FR_OK) {
            printf("XXX write: file result %d\n", result);
            errno = EIO;
            return -1;
        }
        return wrote;
    }
}

int _close(int file)
{
    int myFile = file - FD_OFFSET;
    if(!filesOpened[myFile]) {
        errno = EBADF;
        return -1;
    }
    f_close(&files[myFile]);
    filesOpened[myFile] = 0;
    return 0;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	return 0;
    } else {
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX lseek: file not opened %d\n", myFile);
            errno = EBADF;
            return -1;
        }
        FRESULT result;
        if(dir == SEEK_SET) {
            result = f_lseek(&files[myFile], ptr);
        } else if(dir == SEEK_CUR) {
            result = f_lseek(&files[myFile], ptr + f_tell(&files[myFile]));
        } else /* SEEK_END */ {
            result = f_lseek(&files[myFile], f_size(&files[myFile]) - 1 - ptr);
        }
        if(result != FR_OK) {
            printf("XXX lseek: result not OK %d\n", result);
            errno = EIO;
            return -1;
        }
        return f_tell(&files[myFile]);
    }
}

int _read(int file, char *ptr, int len)
{
    if(file < 0) { errno =  EINVAL; return -1; }

    if((file == 0) || (file == 1) || (file == 2)) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
	  *ptr++ = __io_getchar();
	}
        return len;
    } else {
        int myFile = file - FD_OFFSET;
        if(!filesOpened[myFile]) {
            printf("XXX read: file not opened %d\n", myFile);
            errno = EBADF;
            return -1;
        }
        unsigned int wasRead;
        FRESULT result = f_read(&files[myFile], ptr, len, &wasRead);
        if(result != FR_OK) {
            printf("XXX read: result not OK %d\n", result);
            errno = EIO;
            return -1;
        }
        return wasRead;
    }
}

int _open(char *path, int flags, ...)
{
    if(path == NULL) {
        errno = EFAULT;
        return -1;
    }

    int which = 0;
    while(which < MAX_FILES && filesOpened[which]) {
        which++;
    }
    if(which >= MAX_FILES) {
        return ENFILE;
    }

    int FatFSFlags = 0;

    if(flags == O_RDONLY) {
        FatFSFlags |= FA_READ | FA_OPEN_EXISTING;
    } else if(flags & O_WRONLY) {
        FatFSFlags |= FA_WRITE;
    } else if(flags & O_RDWR) {
        FatFSFlags |= FA_WRITE | FA_READ;
    }

    if(flags & O_APPEND) {
        FatFSFlags |= FA_OPEN_APPEND;
    }
    if(flags & O_CREAT) {
        FatFSFlags |= FA_CREATE_NEW;
    }
    if(flags & O_TRUNC) {
        FatFSFlags |= FA_CREATE_ALWAYS;
    }
    errno = 0;
    printf("XXX opened from flags 0x%X with FatFSFlags 0x%X\n", flags, FatFSFlags);
    FRESULT result = f_open (&files[which], path, FatFSFlags);
    if(result) {
        printf("XXX open couldn't open \"%s\" for reading, FatFS result %d\n", path, result);
        errno = EIO;
        return -1;
    }
    filesOpened[which] = 1;

    return which + FD_OFFSET;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}
