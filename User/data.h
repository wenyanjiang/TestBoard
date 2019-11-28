#ifndef __DATA_H_
#define __DATA_H_
#include <stdint.h>

#include "type.h"

extern u8* g_pu8DataBuf;

u16 CRC16(u8* Buf, u32 BufLen, u16 u16CRC);
u16 Checksum16(u8* Buf, u32 Length);
u16 MergeU16(u8* SrcBuf);
u32 MergeU32(u8* SrcBuf);
void SplitU16(u8* DstBuf, u16 Value);
void SplitU32(u8* DstBuf, u32 Value);
u32 getAlignTimes(u32 dividend, u32 Divisor);
extern bool mallocBuf(u32 size);

#endif
