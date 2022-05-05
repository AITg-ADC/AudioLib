#ifndef AUD_MEM_H
#define AUD_MEM_H
#include "comdef_nvt.h"
#include <stdint.h>
#include <stdlib.h>

extern void *(*AUD_malloc)(s32 s32size);
extern void *(*AUD_calloc)(s32 s32num, s32 s32size);
extern void *(*AUD_realloc)(void *ptr, s32 s32size);
extern void (*AUD_free)(void *ptr);

void AUD_Malloc_Init(void *ptr, u32 u32TotalLen);
int AUD_Malloc_Uninit(void);

#endif
