/*
 * Copyright (c) 2024-, Masscore Electronic, Inc. All Rights Reserved.
 * Author: lixiaogang lixiaogang@masscore.cn
 *         created 2024-May-06
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Regents nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
 * HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
 * MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 */

#ifndef __RISCV_LOG_PRINT_H__
#define __RISCV_LOG_PRINT_H__

#include <stdio.h>

#define RISCV_LOG_PRINT_ERROR   3
#define RISCV_LOG_PRINT_NOTICE  2
#define RISCV_LOG_PRINT_DEBUG   1
#define RISCV_LOG_PRINT_INFO    0

#define RISCV_LOG_PRINT_LEVEL   RISCV_LOG_PRINT_INFO
#define LOG_MEMORY_ALIGNMENT    32

#if (RISCV_LOG_PRINT_LEVEL == RISCV_LOG_PRINT_ERROR)
#define LOG_PRINT_ERROR(X...)  printf(X)
// #define LOG_PRINT_ERROR(X...)   do {} while(0)
#define LOG_PRINT_NOTICE(X...)  do {} while(0)
#define LOG_PRINT_DEBUG(X...)   do {} while(0)
#define LOG_PRINT_INFO(X...)    do {} while(0)
#elif (RISCV_LOG_PRINT_LEVEL == RISCV_LOG_PRINT_NOTICE)
#define LOG_PRINT_ERROR(X...)   printf(X)
#define LOG_PRINT_NOTICE(X...)  printf(X)
#define LOG_PRINT_DEBUG(X...)   do {} while(0)
#define LOG_PRINT_INFO(X...)    do {} while(0)
#elif (RISCV_LOG_PRINT_LEVEL == RISCV_LOG_PRINT_DEBUG)
#define LOG_PRINT_ERROR(X...)   printf(X)
#define LOG_PRINT_NOTICE(X...)  printf(X)
#define LOG_PRINT_DEBUG(X...)   printf(X)
#define LOG_PRINT_INFO(X...)    do {} while(0)
#elif (RISCV_LOG_PRINT_LEVEL == RISCV_LOG_PRINT_INFO)
#define LOG_PRINT_ERROR(X...)   printf(X)
#define LOG_PRINT_NOTICE(X...)  printf(X)
#define LOG_PRINT_DEBUG(X...)   printf(X)
#define LOG_PRINT_INFO(X...)    printf(X)
#else
#error "unknown log out level"
#endif

static inline void LOG_PRINT_MEMORY(const uint8_t* data, size_t len, int level = 1)
{


  if (level < RISCV_LOG_PRINT_LEVEL)
    return;

  size_t i;
  for (i = 0; i < len; i++) {
    if (i % LOG_MEMORY_ALIGNMENT == (LOG_MEMORY_ALIGNMENT-1))
      printf("%02x\n", data[i]);
    else
      printf("%02x ", data[i]);
  }

  if (i % LOG_MEMORY_ALIGNMENT != 0)
    printf("\n");
}

#endif /* __RISCV_LOG_PRINT_H__ */
