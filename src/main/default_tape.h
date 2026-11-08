#ifndef DEFAULT_TAPE_H
#define DEFAULT_TAPE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEFAULT_TAPE_OK,
    DEFAULT_TAPE_NO_FILE,
    DEFAULT_TAPE_IO_ERROR,
    DEFAULT_TAPE_INVALID_PATH
} default_tape_result_t;

default_tape_result_t default_tape_path(const char *root, char *path, size_t size);

#ifdef __cplusplus
}
#endif

#endif
