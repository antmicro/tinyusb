/**************************************************************************/
/*!
    @file     tusb_common.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

/** \ingroup Group_Common
 *  \defgroup Group_CommonH common.h
 *  @{ */

#ifndef _TUSB_COMMON_H_
#define _TUSB_COMMON_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// INCLUDES
//--------------------------------------------------------------------+

//------------- Standard Header -------------//
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

//------------- TUSB Option Header -------------//
#include "tusb_option.h"

//------------- Common Header -------------//
#include "tusb_compiler.h"
#include "tusb_verify.h"
#include "binary.h"
#include "tusb_error.h"
#include "tusb_timeout.h"
#include "tusb_types.h"

//--------------------------------------------------------------------+
// MACROS
//--------------------------------------------------------------------+
#define TU_ARRAY_SZIE(_arr)      ( sizeof(_arr) / sizeof(_arr[0]) )

#define U16_HIGH_U8(u16) ((uint8_t) (((u16) >> 8) & 0x00ff))
#define U16_LOW_U8(u16)  ((uint8_t) ((u16)       & 0x00ff))
#define U16_TO_U8S_BE(u16)  U16_HIGH_U8(u16), U16_LOW_U8(u16)
#define U16_TO_U8S_LE(u16)  U16_LOW_U8(u16), U16_HIGH_U8(u16)

#define U32_B1_U8(u32) ((uint8_t) (((u32) >> 24) & 0x000000ff)) // MSB
#define U32_B2_U8(u32) ((uint8_t) (((u32) >> 16) & 0x000000ff))
#define U32_B3_U8(u32) ((uint8_t) (((u32) >>  8) & 0x000000ff))
#define U32_B4_U8(u32) ((uint8_t) ((u32)        & 0x000000ff)) // LSB

#define U32_TO_U8S_BE(u32) U32_B1_U8(u32), U32_B2_U8(u32), U32_B3_U8(u32), U32_B4_U8(u32)
#define U32_TO_U8S_LE(u32) U32_B4_U8(u32), U32_B3_U8(u32), U32_B2_U8(u32), U32_B1_U8(u32)

//------------- Endian Conversion -------------//
#define ENDIAN_BE(u32) \
    (uint32_t) ( (((u32) & 0xFF) << 24) | (((u32) & 0xFF00) << 8) | (((u32) >> 8) & 0xFF00) | (((u32) >> 24) & 0xFF) )

#define ENDIAN_BE16(le16) ((uint16_t) ((U16_LOW_U8(le16) << 8) | U16_HIGH_U8(le16)) )

// for declaration of reserved field, make use of _TU_COUNTER_
#define TU_RESERVED   XSTRING_CONCAT_(reserved, _TU_COUNTER_)

/*------------------------------------------------------------------*/
/* Count number of arguments of __VA_ARGS__
 * - reference https://groups.google.com/forum/#!topic/comp.std.c/d-6Mj5Lko_s
 * - _GET_NTH_ARG() takes args >= N (64) but only expand to Nth one (64th)
 * - _RSEQ_N() is reverse sequential to N to add padding to have
 * Nth position is the same as the number of arguments
 * - ##__VA_ARGS__ is used to deal with 0 parameter (swallows comma)
 *------------------------------------------------------------------*/
#ifndef VA_ARGS_NUM_

#define VA_ARGS_NUM_(...) 	 NARG_(_0, ##__VA_ARGS__,_RSEQ_N())
#define NARG_(...)        _GET_NTH_ARG(__VA_ARGS__)
#define _GET_NTH_ARG( \
          _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
         _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
         _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
         _31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
         _41,_42,_43,_44,_45,_46,_47,_48,_49,_50, \
         _51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
         _61,_62,_63,N,...) N
#define _RSEQ_N() \
         62,61,60,                      \
         59,58,57,56,55,54,53,52,51,50, \
         49,48,47,46,45,44,43,42,41,40, \
         39,38,37,36,35,34,33,32,31,30, \
         29,28,27,26,25,24,23,22,21,20, \
         19,18,17,16,15,14,13,12,11,10, \
         9,8,7,6,5,4,3,2,1,0
#endif

#ifndef FOR_EACH

// Make a FOREACH macro
#define _FE_00(WHAT, ...)
#define _FE_01(WHAT, X, ...) WHAT(X)
#define _FE_02(WHAT, X, ...) WHAT(X) _FE_01(WHAT, __VA_ARGS__)
#define _FE_03(WHAT, X, ...) WHAT(X) _FE_02(WHAT, __VA_ARGS__)
#define _FE_04(WHAT, X, ...) WHAT(X) _FE_03(WHAT, __VA_ARGS__)
#define _FE_05(WHAT, X, ...) WHAT(X) _FE_04(WHAT, __VA_ARGS__)
#define _FE_06(WHAT, X, ...) WHAT(X) _FE_05(WHAT, __VA_ARGS__)
#define _FE_07(WHAT, X, ...) WHAT(X) _FE_06(WHAT, __VA_ARGS__)
#define _FE_08(WHAT, X, ...) WHAT(X) _FE_07(WHAT, __VA_ARGS__)
#define _FE_09(WHAT, X, ...) WHAT(X) _FE_08(WHAT, __VA_ARGS__)
#define _FE_11(WHAT, X, ...) WHAT(X) _FE_09(WHAT, __VA_ARGS__)
#define _FE_12(WHAT, X, ...) WHAT(X) _FE_11(WHAT, __VA_ARGS__)
#define _FE_13(WHAT, X, ...) WHAT(X) _FE_12(WHAT, __VA_ARGS__)
#define _FE_14(WHAT, X, ...) WHAT(X) _FE_13(WHAT, __VA_ARGS__)
#define _FE_15(WHAT, X, ...) WHAT(X) _FE_14(WHAT, __VA_ARGS__)
#define _FE_16(WHAT, X, ...) WHAT(X) _FE_15(WHAT, __VA_ARGS__)
#define _FE_17(WHAT, X, ...) WHAT(X) _FE_16(WHAT, __VA_ARGS__)
#define _FE_18(WHAT, X, ...) WHAT(X) _FE_17(WHAT, __VA_ARGS__)
#define _FE_19(WHAT, X, ...) WHAT(X) _FE_18(WHAT, __VA_ARGS__)
#define _FE_21(WHAT, X, ...) WHAT(X) _FE_19(WHAT, __VA_ARGS__)
#define _FE_22(WHAT, X, ...) WHAT(X) _FE_21(WHAT, __VA_ARGS__)
#define _FE_23(WHAT, X, ...) WHAT(X) _FE_22(WHAT, __VA_ARGS__)
#define _FE_24(WHAT, X, ...) WHAT(X) _FE_23(WHAT, __VA_ARGS__)
#define _FE_25(WHAT, X, ...) WHAT(X) _FE_24(WHAT, __VA_ARGS__)
#define _FE_26(WHAT, X, ...) WHAT(X) _FE_25(WHAT, __VA_ARGS__)
#define _FE_27(WHAT, X, ...) WHAT(X) _FE_26(WHAT, __VA_ARGS__)
#define _FE_28(WHAT, X, ...) WHAT(X) _FE_27(WHAT, __VA_ARGS__)
#define _FE_29(WHAT, X, ...) WHAT(X) _FE_28(WHAT, __VA_ARGS__)
#define _FE_31(WHAT, X, ...) WHAT(X) _FE_29(WHAT, __VA_ARGS__)
#define _FE_32(WHAT, X, ...) WHAT(X) _FE_31(WHAT, __VA_ARGS__)
#define _FE_33(WHAT, X, ...) WHAT(X) _FE_32(WHAT, __VA_ARGS__)
#define _FE_34(WHAT, X, ...) WHAT(X) _FE_33(WHAT, __VA_ARGS__)
#define _FE_35(WHAT, X, ...) WHAT(X) _FE_34(WHAT, __VA_ARGS__)
#define _FE_36(WHAT, X, ...) WHAT(X) _FE_35(WHAT, __VA_ARGS__)
#define _FE_37(WHAT, X, ...) WHAT(X) _FE_36(WHAT, __VA_ARGS__)
#define _FE_38(WHAT, X, ...) WHAT(X) _FE_37(WHAT, __VA_ARGS__)
#define _FE_39(WHAT, X, ...) WHAT(X) _FE_38(WHAT, __VA_ARGS__)
#define _FE_41(WHAT, X, ...) WHAT(X) _FE_39(WHAT, __VA_ARGS__)
#define _FE_42(WHAT, X, ...) WHAT(X) _FE_41(WHAT, __VA_ARGS__)
#define _FE_43(WHAT, X, ...) WHAT(X) _FE_42(WHAT, __VA_ARGS__)
#define _FE_44(WHAT, X, ...) WHAT(X) _FE_43(WHAT, __VA_ARGS__)
#define _FE_45(WHAT, X, ...) WHAT(X) _FE_44(WHAT, __VA_ARGS__)
#define _FE_46(WHAT, X, ...) WHAT(X) _FE_45(WHAT, __VA_ARGS__)
#define _FE_47(WHAT, X, ...) WHAT(X) _FE_46(WHAT, __VA_ARGS__)
#define _FE_48(WHAT, X, ...) WHAT(X) _FE_47(WHAT, __VA_ARGS__)
#define _FE_49(WHAT, X, ...) WHAT(X) _FE_48(WHAT, __VA_ARGS__)
#define _FE_51(WHAT, X, ...) WHAT(X) _FE_49(WHAT, __VA_ARGS__)
#define _FE_52(WHAT, X, ...) WHAT(X) _FE_51(WHAT, __VA_ARGS__)
#define _FE_53(WHAT, X, ...) WHAT(X) _FE_52(WHAT, __VA_ARGS__)
#define _FE_54(WHAT, X, ...) WHAT(X) _FE_53(WHAT, __VA_ARGS__)
#define _FE_55(WHAT, X, ...) WHAT(X) _FE_54(WHAT, __VA_ARGS__)
#define _FE_56(WHAT, X, ...) WHAT(X) _FE_55(WHAT, __VA_ARGS__)
#define _FE_57(WHAT, X, ...) WHAT(X) _FE_56(WHAT, __VA_ARGS__)
#define _FE_58(WHAT, X, ...) WHAT(X) _FE_57(WHAT, __VA_ARGS__)
#define _FE_59(WHAT, X, ...) WHAT(X) _FE_58(WHAT, __VA_ARGS__)
#define _FE_61(WHAT, X, ...) WHAT(X) _FE_59(WHAT, __VA_ARGS__)
#define _FE_62(WHAT, X, ...) WHAT(X) _FE_61(WHAT, __VA_ARGS__)

#define _FE_RSEQ_N() \
         _FE_62,_FE_61,_FE_60,_FE_                      \
         _FE_59,_FE_58,_FE_57,_FE_56,_FE_55,_FE_54,_FE_53,_FE_52,_FE_51,_FE_50,_FE_ \
         _FE_49,_FE_48,_FE_47,_FE_46,_FE_45,_FE_44,_FE_43,_FE_42,_FE_41,_FE_40,_FE_ \
         _FE_39,_FE_38,_FE_37,_FE_36,_FE_35,_FE_34,_FE_33,_FE_32,_FE_31,_FE_30,_FE_ \
         _FE_29,_FE_28,_FE_27,_FE_26,_FE_25,_FE_24,_FE_23,_FE_22,_FE_21,_FE_20,_FE_ \
         _FE_19,_FE_18,_FE_17,_FE_16,_FE_15,_FE_14,_FE_13,_FE_12,_FE_11,_FE_10,_FE_ \
         _FE_09,_FE_08,_FE_07,_FE_06,_FE_05,_FE_04,_FE_03,_FE_02,_FE_01,_FE_00

#define FOR_EACH(x, ...) \
    NARG_(_0, ##__VA_ARGS__,_FE_RSEQ_N())(x, ##__VA_ARGS__)

#endif

//--------------------------------------------------------------------+
// INLINE FUNCTION
//--------------------------------------------------------------------+
#define tu_memclr(buffer, size)  memset((buffer), 0, (size))
#define tu_varclr(_var)          tu_memclr(_var, sizeof(*(_var)))

static inline bool tu_mem_test_zero (void const* buffer, uint32_t size)
{
  uint8_t const* p_mem = (uint8_t const*) buffer;
  for(uint32_t i=0; i<size; i++) if (p_mem[i] != 0)  return false;
  return true;
}


//------------- Conversion -------------//
static inline uint32_t tu_u32_from_u8(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
  return ( ((uint32_t) b1) << 24) + ( ((uint32_t) b2) << 16) + ( ((uint32_t) b3) << 8) + b4;
}

static inline uint8_t tu_u16le_high(uint16_t u16)
{
  return (uint8_t) ( ((uint16_t) (u16 >> 8)) & 0x00ff);
}

static inline uint8_t tu_u16le_low(uint16_t u16)
{
  return (uint8_t) (u16 & 0x00ff);
}

//#ifdef LITTLE_ENDIAN
#define tu_u16_high tu_u16le_high
#define tu_u16_low tu_u16le_low
//#else
//#define tu_u16_high tu_u16le_low
//#define tu_u16_low tu_u16le_high
//#endif

static inline uint16_t tu_u16_le2be(uint16_t u16)
{
  return ((uint16_t)(tu_u16_low(u16) << 8)) | tu_u16_high(u16);
}

//------------- Min -------------//
static inline uint8_t tu_min8(uint8_t x, uint8_t y)
{
  return (x < y) ? x : y;
}

static inline uint16_t tu_min16(uint16_t x, uint16_t y)
{
  return (x < y) ? x : y;
}

static inline uint32_t tu_min32(uint32_t x, uint32_t y)
{
  return (x < y) ? x : y;
}

//------------- Max -------------//
static inline uint32_t tu_max32(uint32_t x, uint32_t y)
{
  return (x > y) ? x : y;
}

//------------- Align -------------//
static inline uint32_t tu_align32 (uint32_t value)
{
	return (value & 0xFFFFFFE0UL);
}

static inline uint32_t tu_align16 (uint32_t value)
{
	return (value & 0xFFFFFFF0UL);
}

static inline uint32_t tu_align_n (uint32_t alignment, uint32_t value)
{
	return value & ((uint32_t) ~(alignment-1));
}

static inline uint32_t tu_align4k (uint32_t value)
{
	return (value & 0xFFFFF000UL);
}

static inline uint32_t tu_offset4k(uint32_t value)
{
	return (value & 0xFFFUL);
}

//------------- Mathematics -------------//
static inline uint32_t tu_abs(int32_t value)
{
  return (value < 0) ? (-value) : value;
}


/// inclusive range checking
static inline bool tu_within(uint32_t lower, uint32_t value, uint32_t upper)
{
  return (lower <= value) && (value <= upper);
}

// TODO use clz
static inline uint8_t tu_log2(uint32_t value)
{
  uint8_t result = 0; // log2 of a value is its MSB's position

  while (value >>= 1)
  {
    result++;
  }
  return result;
}

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_COMMON_H_ */

/**  @} */
