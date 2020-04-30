#ifndef __DPP_H
#define __DPP_H

#include <string.h>
#include "libc.h"


typedef struct {
    int p_w[2];
    int w_p[2];
} double_channel_t;

#define n_phil 5

typedef enum{
    FREE,
    TAKEN,
    RESERVED
} chopstick_t;

#endif //KERNEL_DPP_H
