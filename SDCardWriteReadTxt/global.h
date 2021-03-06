/* 
 * MIT License
 *
 * Copyright (c) 2022 CoretronicMEMS
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#define GET_BITS(reg, msk, pos)             (((reg) >> pos) & msk)
#define SET_BITS(reg, msk, val)             (reg) = (((reg) & ~(msk)) | (val))
#define MCUREG_GET_FIELD(reg, bitname)      (((reg) & bitname##_Msk) >> bitname##_Pos)
#define MCUREG_SET_FIELD(reg, bitname, val) (reg) = (((reg) & ~bitname##_Msk) | (((val)<<bitname##_Pos)&bitname##_Msk))

#define PINMUX_SET(reg, port, val)  SET_BITS(SYS->reg, SYS_##reg##_##port##MFP_Msk, SYS_##reg##_##port##MFP_##val)

#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof(arr[0]))

#endif
