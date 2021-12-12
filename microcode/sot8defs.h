/*
Copyright (c) 2021 Jason R. Thorpe.
All righs reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* Register numbers */
#define	REG_R0			0
#define	REG_R1			1
#define	REG_R2			2
#define	REG_R3			3
#define	REG_SP			4
#define	REG_PC			5
#define	REG_SPa			6	/* pseudo */
#define	REG_IMM			7	/* pseudo */

/* Opcode classes */
#define	OPC_MOV			0
#define	OPC_LD			1
#define	OPC_ST			2
#define	OPC_ALU			3

/* Conditional jump codes */
#define	CJMP_C			0
#define	CJMP_Z			1
#define	CJMP_O			2
#define	CJMP_N			4

/*
 * These ALU function selectors map directly to the function
 * selectors on the 74LS382 ALU chip, except for INC_A and NOT_A.
 */
#define	ALU_FN_INC_A		0	/* pseudo */
#define	ALU_FN_B_MINUS_A	1
#define	ALU_FN_A_MINUS_B	2
#define	ALU_FN_A_PLUS_B		3
#define	ALU_FN_A_XOR_B		4
#define	ALU_FN_A_OR_B		5
#define	ALU_FN_A_AND_B		6
#define	ALU_FN_NOT_A		7	/* pseudo */

/*
 * Standard instruction encoding:
 *
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | class |    dst    |    src    |
 * +-------------------------------+
 */
#define	OPCODE(class, dreg, sreg)					\
	(((class) << 6) | ((dreg) << 3) | (sreg))

/*
 * ALU instruction encoding:
 *
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | C |   func    |  dst  |
 * +-------------------------------+
 *
 * C == Instruction is a "with carry" variant
 *
 * dst register is also one of the input operands; other register is
 * always Rb.
 */
#define	ALU_OPCODE(carry, fn, dreg)					\
	((OPC_ALU << 6) | ((carry) ? 1 << 5 : 0) | ((fn) << 2) | (dreg))

