idf_component_register(
    SRCS "bbc_machine.c"
    INCLUDE_DIRS "include"
    REQUIRES bbc_memory bbc_cpu m6522 sn76489 bbc_video bbc_tape
)
