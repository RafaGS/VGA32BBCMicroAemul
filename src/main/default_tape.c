#include "default_tape.h"

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

default_tape_result_t default_tape_path(const char *root, char *path, size_t size)
{
    int n = snprintf(path, size, "%s/tape.uef", root);
    if (n < 0 || (size_t)n >= size)
        return DEFAULT_TAPE_INVALID_PATH;

    struct stat info;
    if (stat(path, &info) != 0)
        return errno == ENOENT ? DEFAULT_TAPE_NO_FILE : DEFAULT_TAPE_IO_ERROR;
    if (!S_ISREG(info.st_mode))
        return DEFAULT_TAPE_IO_ERROR;
    return DEFAULT_TAPE_OK;
}
