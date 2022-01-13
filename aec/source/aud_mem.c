#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "aud_mem.h"

void *(*AUD_malloc)(s32 s32size);
void *(*AUD_calloc)(s32 s32num, s32 s32size);
void *(*AUD_realloc)(void *ptr, s32 s32size);
void (*AUD_free)(void *ptr);


typedef enum _EN_AUD_MEMORY_ALLOC_STATUS
{
    EN_AUD_MEMORY_ALLOC_STATUS_NORMAL,
    EN_AUD_MEMORY_ALLOC_STATUS_OVER_TOTAL_LEN,
    EN_AUD_MEMORY_ALLOC_STATUS_TOTAL
} EN_AUD_MEMORY_ALLOC_STATUS;


typedef struct _ST_AUD_MEMORY_ALLOC
{
    u8  *pu8StartAddress;
    u32 u32TotalLen;
    u32 u32UsedLen;
    EN_AUD_MEMORY_ALLOC_STATUS enMEMAllocStatus;
    u32 u32Counter;
} ST_AUD_MEMORY_ALLOC, *PST_AUD_MEMORY_ALLOC;

static ST_AUD_MEMORY_ALLOC _astMemAlloc[1];
static void *_pMemInitAddr;

//==========================================================================================
// memory allocate
//==========================================================================================
void * _AUD_Malloc(s32 s32SizeOrg)
{
    PST_AUD_MEMORY_ALLOC pstMemAlloc = &_astMemAlloc[0];
    u8 *pu8StartAddr = pstMemAlloc->pu8StartAddress + pstMemAlloc->u32UsedLen;
    s32 s32Size;

    if(pstMemAlloc->enMEMAllocStatus == EN_AUD_MEMORY_ALLOC_STATUS_OVER_TOTAL_LEN)
    {
        return(NULL);    
    }
    
    s32Size = (s32SizeOrg + 3) & 0xfffffffc;
    s32Size += pstMemAlloc->u32UsedLen;

    if((u32)s32Size > pstMemAlloc->u32TotalLen)
    {
        pstMemAlloc->enMEMAllocStatus = EN_AUD_MEMORY_ALLOC_STATUS_OVER_TOTAL_LEN;
    }

    if(pstMemAlloc->enMEMAllocStatus == EN_AUD_MEMORY_ALLOC_STATUS_OVER_TOTAL_LEN)
    {
        printf("\n[AUD] Error !!! (_AUD_Malloc) over max. length\n");
        return(NULL);
    }
    
    pstMemAlloc->u32UsedLen = s32Size;
    pstMemAlloc->u32Counter++;

    // printf("[AUD](%d) MemAlloc size: %d, addr: 0x%x, total: %d\n",pstMemAlloc->u32Counter, s32SizeOrg, (intptr_t)pu8StartAddr, s32Size);
    
    return(pu8StartAddr);    
}

//-------------------------------------------------------------------------------------
void * _AUD_Calloc(s32 s32num, s32 s32size)
{
    void *pstart = _AUD_Malloc(s32num * s32size);
    if(pstart != NULL)
    {
        memset(pstart, 0, s32num * s32size);
    }

    return(pstart);
}

//-------------------------------------------------------------------------------------
void * _AUD_Realloc(void *ptr, s32 s32Size)
{
    void *pviodDes = _AUD_Malloc(s32Size);   
    if(ptr != NULL && pviodDes != NULL)
    {
        memcpy (pviodDes, ptr, s32Size);
    }
    
    return(pviodDes);
}

//-------------------------------------------------------------------------------------
void _AUD_Free(void *ptr)
{
    return;
}
//--------------------------------------------------------------------------------------
void AUD_Malloc_Init(void *ptr, u32 u32TotalLen)
{
    PST_AUD_MEMORY_ALLOC pstMemAlloc = &_astMemAlloc[0];

    _pMemInitAddr = ptr;

    pstMemAlloc->pu8StartAddress = (u8 *)ptr;

    pstMemAlloc->u32TotalLen = u32TotalLen;
    pstMemAlloc->u32UsedLen = 0;    
    pstMemAlloc->enMEMAllocStatus = EN_AUD_MEMORY_ALLOC_STATUS_NORMAL;
    pstMemAlloc->u32Counter = 0;

    AUD_malloc = _AUD_Malloc;
    AUD_realloc = _AUD_Realloc;
    AUD_calloc = _AUD_Calloc;
    AUD_free = _AUD_Free;
}

