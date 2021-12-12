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

%{
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "../microcode/sot8defs.h"
#include "y.tab.h"

#define	RETURN_INSN(x)	{ yylval.insn = (x); return (x); }

extern int yyline;
%}

%%

;.*		{ /* discard comments */; }
[ \t]+		return WS;

,		return ',';
\n		{ yyline++; return '\n'; }

\[		return '[';
\]		return ']';

:		return ':';
=		return '=';

%r[0-3]	{
		yylval.regnum = yytext[2] - '0';
		return GP_REG;
	}
%sp	{
		yylval.regnum = REG_SP;
		return SP_REG;
	}
%pc	{
		yylval.regnum = REG_PC;
		return PC_REG;
	}
#0[0-7]* {
		yylval.immediate = (long)strtoul(yytext + 1, NULL, 8);
		return IMMEDIATE;
	}
#0[xX][0-9a-fA-F]+ {
		yylval.immediate = (long)strtoul(yytext + 3, NULL, 16);
		return IMMEDIATE;
	}
#[\-]?[1-9][0-9]* {
		yylval.immediate = strtol(yytext + 1, NULL, 10);
		return IMMEDIATE;
	}
\$[a-zA-Z][0-9a-zA-Z_]* {
		yylval.label = strdup(yytext);
		return LABEL;
	}

mov		RETURN_INSN(INSN_MOV)
hlt		RETURN_INSN(INSN_HLT)
nop		RETURN_INSN(INSN_NOP)
jmp		RETURN_INSN(INSN_JMP)
jc		RETURN_INSN(INSN_JC)
jnc		RETURN_INSN(INSN_JNC)
jz		RETURN_INSN(INSN_JZ)
jnz		RETURN_INSN(INSN_JNZ)
jo		RETURN_INSN(INSN_JO)
jno		RETURN_INSN(INSN_JNO)
jn		RETURN_INSN(INSN_JN)
jnn		RETURN_INSN(INSN_JNN)
call		RETURN_INSN(INSN_CALL)
spa		RETURN_INSN(INSN_SPA)

jeq		RETURN_INSN(INSN_JZ)
jne		RETURN_INSN(INSN_JNZ)
jgt		RETURN_INSN(INSN_JNN)
jlt		RETURN_INSN(INSN_JN)

ld		RETURN_INSN(INSN_LD)
pop		RETURN_INSN(INSN_POP)
ret		RETURN_INSN(INSN_RET)
inb		RETURN_INSN(INSN_INB)
ldi		RETURN_INSN(INSN_LDI)

st		RETURN_INSN(INSN_ST)
push		RETURN_INSN(INSN_PUSH)
outb		RETURN_INSN(INSN_OUTB)
sti		RETURN_INSN(INSN_STI)

inc		RETURN_INSN(INSN_INC)
dec		RETURN_INSN(INSN_DEC)
add		RETURN_INSN(INSN_ADD)
adc		RETURN_INSN(INSN_ADC)
sub		RETURN_INSN(INSN_SUB)
sbc		RETURN_INSN(INSN_SBC)
and		RETURN_INSN(INSN_AND)
or		RETURN_INSN(INSN_OR)
xor		RETURN_INSN(INSN_XOR)
not		RETURN_INSN(INSN_NOT)
cmp		RETURN_INSN(INSN_CMP)
tst		RETURN_INSN(INSN_TST)

%%

int
yywrap(void)
{
	return 1;
}
