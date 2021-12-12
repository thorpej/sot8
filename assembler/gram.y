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
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../microcode/sot8defs.h"
#include "asdefs.h"

extern int yylex(void);

int yyline;

static void yyerror(const char *);

static struct operand *gen_reg_operand(int);
static struct operand *gen_label_operand(char *);
static struct operand *gen_imm_operand(long);

static void gen_label_decl(char *);
static void gen_label_assignment(char *, long);
static struct label *gen_labelref(char *);

static void gen_cond_jmp(int, struct operand *);

static void gen_insn2(unsigned int, struct operand *, struct operand *);
static void gen_insn3(unsigned int, struct operand *, struct operand *,
		      struct operand *);
static void gen_alu_insn(unsigned int, struct operand *, struct operand *);

static long current_address;
static unsigned int insn_count;

static void aserror(const char *fmt, ...)
   __attribute__((__format__(__printf__, 1, 2)));
static void asdebug(bool, const char *fmt, ...)
    __attribute__((__format__(__printf__, 2, 3)));
static void asmsg(const char *fmt, ...)
    __attribute__((__format__(__printf__, 1, 2)));

static const char *insn_name(int);
static const char *register_name(unsigned int);

static struct label_head all_labels =
    LIST_HEAD_INITIALIZER(all_labels);
static struct program_node_head all_nodes =
    TAILQ_HEAD_INITIALIZER(all_nodes);

%}

%union {
	int regnum;
	int insn;
	long immediate;
	char *label;
}

%token	<regnum> GP_REG SP_REG PC_REG
%token	<immediate> IMMEDIATE
%token	<label> LABEL
%token	WS

%token	<insn> INSN_MOV INSN_HLT INSN_NOP INSN_JMP
%token	<insn> INSN_JC INSN_JNC INSN_JZ INSN_JNZ
%token	<insn> INSN_JO INSN_JNO INSN_JN INSN_JNN
%token	<insn> INSN_CALL INSN_SPA

%token	<insn> INSN_LD INSN_POP INSN_INB INSN_LDI INSN_RET

%token	<insn> INSN_ST INSN_PUSH INSN_OUTB INSN_STI

%token	<insn> INSN_INC INSN_DEC INSN_ADD INSN_ADC INSN_SUB INSN_SBC
%token	<insn> INSN_AND INSN_OR INSN_XOR INSN_NOT INSN_CMP INSN_TST

%type	<regnum>	any_reg
%type	<insn>		mov_cond_jmp_which alu_1opr_which alu_2opr_which

%%

/* A program. */
program:
	  statements
;

statements:
	  /* empty */
	| statements statement
;

statement:
	  empty_statement
	| label_statement
	| insn_statement
;

rest_of_line:
	  opt_ws '\n'
;

opt_ws:
	  /* empty */
	| WS
;

opr_sep:
	  opt_ws ',' opt_ws
;

opt_label_decl:
	  opt_ws
	| opt_ws LABEL ':' opt_ws
	{
		gen_label_decl($2);
	}
;

any_reg:
	  GP_REG		{ $$ = $1; }
	| SP_REG		{ $$ = $1; }
	| PC_REG		{ $$ = $1; }
;

empty_statement:
	  rest_of_line
;

/*
 * $some_label:
 *	[code goes here]
 *
 * $some_constant = #0xf0	; Useful for defining, uh, constants
 */
label_statement:
	  opt_ws LABEL ':' rest_of_line
	{
		gen_label_decl($2);
	}
	| opt_ws LABEL opt_ws '=' opt_ws IMMEDIATE rest_of_line
	{
		gen_label_assignment($2, $6);
	}
;

insn_statement:
	  opt_label_decl insn rest_of_line
;

insn:
	  mov_insn
	| ld_insn
	| st_insn
	| alu_insn
;

/*
 * MOV class instructions
 */
mov_insn:
	  mov_reg_reg_insn
	| mov_hlt_insn
	| mov_nop_insn
	| mov_jmp_insn
	| mov_reg_imm_insn
	| mov_cond_jmp_insn
	| mov_call_insn
	| mov_spa_insn
;

/*
 *	mov	%rd, %rs
 *
 * %sp and %pc can also be used in either operand, but "mov %pc, %pc"
 * is disallowed.
 */
mov_reg_reg_insn:
	  INSN_MOV WS any_reg opr_sep any_reg
	{
		/* We have to filter out "mov %pc, %pc" */
		if ($3 == REG_PC && $5 == REG_PC) {
			aserror("\"%s %s, %s\" -- "
			    "illegal register combination",
			    insn_name($1),
			    register_name($3), register_name($5));
		} else {
			gen_insn2(OPC_MOV,
			    gen_reg_operand($3),
			    gen_reg_operand($5));
		}
	}
;

/*
 *	hlt
 */
mov_hlt_insn:
	  INSN_HLT
	{
		/* This is "mov %pc, %pc */
		gen_insn2(OPC_MOV,
		    gen_reg_operand(REG_PC),
		    gen_reg_operand(REG_PC));
	}
;

/*
 *	nop
 */
mov_nop_insn:
	  INSN_NOP
	{
		/* This is "mov %r0, %r0" */
		gen_insn2(OPC_MOV,
		    gen_reg_operand(REG_R0),
		    gen_reg_operand(REG_R0));
	}
;

/*
 *	jmp	#0x50
 *	jmp	$label
 */
mov_jmp_insn:
	  INSN_JMP WS IMMEDIATE
	{
		/* This is "mov %pc, #IMM" */
		gen_insn3(OPC_MOV,
		    gen_reg_operand(REG_PC),
		    gen_reg_operand(REG_IMM),
		    gen_imm_operand($3));
	}
	| INSN_JMP WS LABEL
	{
		/* This is "mov %pc, #IMM" */
		gen_insn3(OPC_MOV,
		    gen_reg_operand(REG_PC),
		    gen_reg_operand(REG_IMM),
		    gen_label_operand($3));
	}
;

/*
 *	mov	%rd, #1
 *	mov	%sp, #1
 *
 *	mov	%rd, $label
 *	mov	%sp, $label
 */
mov_reg_imm_insn:
	  INSN_MOV WS any_reg opr_sep IMMEDIATE
	{
		/* We need to filter out "mov %pc, #IMM" */
		if ($3 == REG_PC) {
			aserror("\"%s %s, #IMM\" -- "
			    "illegal register",
			    insn_name($1),
			    register_name($3));
		} else {
			gen_insn3(OPC_MOV,
			    gen_reg_operand($3),
			    gen_reg_operand(REG_IMM),
			    gen_imm_operand($5));
		}
	}
	| INSN_MOV WS any_reg opr_sep LABEL
	{
		/* We need to filter out "mov %pc, #IMM" */
		if ($3 == REG_PC) {
			aserror("\"%s %s, $label\" -- "
			    "illegal register",
			    insn_name($1),
			    register_name($3));
		} else {
			gen_insn3(OPC_MOV,
			    gen_reg_operand($3),
			    gen_reg_operand(REG_IMM),
			    gen_label_operand($5));
		}
	}
;

/*
 *	jz	#0x50
 *	jz	$label
 *
 * Valid insns: jc, jnc, jz, jnz, jo, jno, jn, jnn
 *
 * The lexical scanner also provides the following useful
 * aliases:
 *
 *	jeq	-> jz
 *	jne	-> jnz
 *	jgt	-> jnn
 *	jlt	-> jn
 *
 * Sorry, kids, no jle or jge -- all of the condition codes except
 * C are multiplexed in the hardware, so only one decision at a time.
 */
mov_cond_jmp_insn:
	  mov_cond_jmp_which WS IMMEDIATE
	{
		gen_cond_jmp($1, gen_imm_operand($3));
	}
	| mov_cond_jmp_which WS LABEL
	{
		gen_cond_jmp($1, gen_label_operand($3));
	}
;

mov_cond_jmp_which:
	  INSN_JC		{ $$ = $1; }
	| INSN_JNC		{ $$ = $1; }
	| INSN_JZ		{ $$ = $1; }
	| INSN_JNZ		{ $$ = $1; }
	| INSN_JO		{ $$ = $1; }
	| INSN_JNO		{ $$ = $1; }
	| INSN_JN		{ $$ = $1; }
	| INSN_JNN		{ $$ = $1; }
;

/*
 *	call	#0x50
 *	call	$label
 */
mov_call_insn:
	  INSN_CALL WS IMMEDIATE
	{
		gen_insn3(OPC_MOV,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand(REG_PC),
		    gen_imm_operand($3));
	}
	| INSN_CALL WS LABEL
	{
		gen_insn3(OPC_MOV,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand(REG_PC),
		    gen_label_operand($3));
	}
;

/*
 *	spa	#2
 */
mov_spa_insn:
	  INSN_SPA WS IMMEDIATE
	{
		gen_insn3(OPC_MOV,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand(REG_IMM),
		    gen_imm_operand($3));
	}
;

/*
 * LD class instructions
 */
ld_insn:
	  ld_indirect_insn
	| ld_imm_indirect_insn
	| ld_sp_rel_insn
	| ld_pop_insn
	| ld_ret_insn
	| ld_inb_insn
	| ld_ldi_insn
;

/*
 *	ld	%rd, [%rs]
 *	ld	%sp, [%rs]
 *	ld	%pc, [%rs]
 */
ld_indirect_insn:
	  INSN_LD WS any_reg opr_sep '[' GP_REG ']'
	{
		gen_insn2(OPC_LD,
		    gen_reg_operand($3),
		    gen_reg_operand($6));
	}
;

/*
 *	ld	%rd, [#0x50]
 *	ld	%sp, [#0x50]
 *	ld	%pc, [#0x50]
 *
 *	ld	%rd, [$label]
 *	ld	%sp, [$label]
 *	ld	%pc, [$label]
 */
ld_imm_indirect_insn:
	  INSN_LD WS any_reg opr_sep '[' IMMEDIATE ']'
	{
		gen_insn3(OPC_LD,
		    gen_reg_operand($3),
		    gen_reg_operand(REG_IMM),
		    gen_imm_operand($6));
	}
	| INSN_LD WS any_reg opr_sep '[' LABEL ']'
	{
		gen_insn3(OPC_LD,
		    gen_reg_operand($3),
		    gen_reg_operand(REG_IMM),
		    gen_label_operand($6));
	}
;

/*
 *	ld	%rd, [%sp]
 *	ld	%sp, [%sp]
 *	ld	%pc, [%sp]
 *
 *	ld	%rd, #2[%sp]
 *	ld	%sp, #2[%sp]
 *	ld	%pc, #2[%sp]
 */
ld_sp_rel_insn:
	  INSN_LD WS any_reg opr_sep '[' SP_REG ']'
	{
		gen_insn2(OPC_LD,
		    gen_reg_operand($3),
		    gen_reg_operand($6));
	}
	| INSN_LD WS any_reg opr_sep IMMEDIATE '[' SP_REG ']'
	{
		/* We can use the smaller encoding for offset #0. */
		if ($5 == 0) {
			gen_insn2(OPC_LD,
			    gen_reg_operand($3),
			    gen_reg_operand($7));
		} else {
			/* This encoding is weird. */
			gen_insn3(OPC_LD,
			    gen_reg_operand(REG_SPa),
			    gen_reg_operand($3),
			    gen_imm_operand($5));
		}
	}
;

/*
 *	pop	%rd
 *	pop	%sp
 */
ld_pop_insn:
	  INSN_POP WS any_reg
	{
		/* Disallow %pc. */
		if ($3 == REG_PC) {
			aserror("\"%s %s\" -- "
			    "illegal register",
			    insn_name($1),
			    register_name($3));
		} else {
			gen_insn2(OPC_LD,
			    gen_reg_operand($3),
			    gen_reg_operand(REG_SPa));
		}
	}
;

/*
 *	ret
 */
ld_ret_insn:
	  INSN_RET
	{
		gen_insn2(OPC_LD,
		    gen_reg_operand(REG_PC),
		    gen_reg_operand(REG_SPa));
	}
;

/*
 *	inb	%rd, [#0x10]
 *	inb	%rd, [$label]
 */
ld_inb_insn:
	  INSN_INB WS GP_REG opr_sep '[' IMMEDIATE ']'
	{
		/* This encoding is weird. */
		gen_insn3(OPC_LD,
		    gen_reg_operand(REG_IMM),
		    gen_reg_operand($3),
		    gen_imm_operand($6));
	}
	| INSN_INB WS GP_REG opr_sep '[' LABEL ']'
	{
		/* This encoding is weird. */
		gen_insn3(OPC_LD,
		    gen_reg_operand(REG_IMM),
		    gen_reg_operand($3),
		    gen_label_operand($6));
	}
;

/*
 *	ldi	%rd, [%r0]
 */
ld_ldi_insn:
	  INSN_LDI WS any_reg opr_sep '[' GP_REG ']'
	{
		/* src register must be %r0. */
		if ($6 != REG_R0) {
			aserror("\"%s %s, [%s]\" -- "
			    "illegal source register, must be %s",
			    insn_name($1),
			    register_name($3),
			    register_name($6),
			    register_name(REG_R0));
		} else {
			/* This encoding is weird. */
			gen_insn2(OPC_LD,
			    gen_reg_operand($3),
			    gen_reg_operand(REG_PC));
		}
	}
;

/*
 * ST class instructions
 */
st_insn:
	  st_indirect_insn
	| st_imm_indirect_insn
	| st_sp_rel_insn
	| st_push_insn
	| st_push_imm_insn
	| st_outb_insn
	| st_sti_insn
;

/*
 *	st	[%rd], %rs
 *	st	[%rd], %sp
 *	st	[%rd], %pc
 */
st_indirect_insn:
	  INSN_ST WS '[' GP_REG ']' opr_sep any_reg
	{
		gen_insn2(OPC_ST,
		    gen_reg_operand($4),
		    gen_reg_operand($7));
	}
;

/*
 *	st	[#0x50], %rs
 *	st	[#0x50], %sp
 *	st	[#0x50], %pc
 *
 *	st	[$label], %rs
 *	st	[$label], %sp
 *	st	[$label], %pc
 */
st_imm_indirect_insn:
	  INSN_ST WS '[' IMMEDIATE ']' opr_sep any_reg
	{
		gen_insn3(OPC_ST,
		    gen_reg_operand(REG_IMM),
		    gen_reg_operand($7),
		    gen_imm_operand($4));
	}
	| INSN_ST WS '[' LABEL ']' opr_sep any_reg
	{
		gen_insn3(OPC_ST,
		    gen_reg_operand(REG_IMM),
		    gen_reg_operand($7),
		    gen_label_operand($4));
	}
;

/*
 *	st	[%sp], %rs
 *	st	[%sp], %sp
 *	st	[%sp], %pc
 *
 *	st	#3[%sp], %rs
 *	st	#3[%sp], %sp
 *	st	#3[%sp], %pc
 */
st_sp_rel_insn:
	  INSN_ST WS '[' SP_REG ']' opr_sep any_reg
	{
		gen_insn2(OPC_ST,
		    gen_reg_operand($4),
		    gen_reg_operand($7));
	}
	| INSN_ST WS IMMEDIATE '[' SP_REG ']' opr_sep any_reg
	{
		/* We can use the smaller encoding for offset #0. */
		if ($3 == 0) {
			gen_insn2(OPC_ST,
			    gen_reg_operand($5),
			    gen_reg_operand($8));
		} else {
			/* This encoding is weird. */
			gen_insn3(OPC_ST,
			    gen_reg_operand($8),
			    gen_reg_operand(REG_SPa),
			    gen_imm_operand($3));
		}
	}
;

/*
 *	push	%rs
 *	push	%sp
 *	push	%pc
 */
st_push_insn:
	  INSN_PUSH WS any_reg
	{
		gen_insn2(OPC_ST,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand($3));
	}
;

/*
 *	push	#10
 *	push	$label
 */
st_push_imm_insn:
	  INSN_PUSH WS IMMEDIATE
	{
		gen_insn3(OPC_ST,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand(REG_IMM),
		    gen_imm_operand($3));
	}
	| INSN_PUSH WS LABEL
	{
		gen_insn3(OPC_ST,
		    gen_reg_operand(REG_SPa),
		    gen_reg_operand(REG_IMM),
		    gen_label_operand($3));
	}
;

/*
 *	outb	[#0x10], %rs
 *	outb	[$label], %rs
 */
st_outb_insn:
	  INSN_OUTB WS '[' IMMEDIATE ']' opr_sep GP_REG
	{
		/* This encoding is weird. */
		gen_insn3(OPC_ST,
		    gen_reg_operand($7),
		    gen_reg_operand(REG_IMM),
		    gen_imm_operand($4));
	}
	| INSN_OUTB WS '[' LABEL ']' opr_sep GP_REG
	{
		/* This encoding is weird. */
		gen_insn3(OPC_ST,
		    gen_reg_operand($7),
		    gen_reg_operand(REG_IMM),
		    gen_label_operand($4));
	}
;

/*
 *	sti	[%r0], %rs
 *	sti	[%r0], %sp
 *	sti	[%r0], %pc
 */
st_sti_insn:
	  INSN_STI WS '[' GP_REG ']' opr_sep any_reg
	{
		/* dst register must be %r0. */
		if ($4 != REG_R0) {
			aserror("\"%s [%s], %s\" -- "
			    "illegal destination register, must be %s",
			    insn_name($1),
			    register_name($4),
			    register_name($7),
			    register_name(REG_R0));
		} else {
			/* This encoding is weird. */
			gen_insn2(OPC_ST,
			    gen_reg_operand(REG_PC),
			    gen_reg_operand($7));
		}
	}
;

/*
 * ALU class instructions
 */
alu_insn:
	  alu_1opr_insn
	| alu_2opr_insn
;

/*
 *	inc	%rd		%rd = %rd + 1
 *	dec	%rd		%rd = %rd - 1
 *	not	%rd		%rd = ~%rd
 *	tst	%rd		flags <- (%d + 0)
 */
alu_1opr_insn:
	  alu_1opr_which WS GP_REG
	{
		/* Pass the implied %r1 to make gen_alu_insn() logic simpler. */
		gen_alu_insn($1,
		    gen_reg_operand($3),
		    gen_reg_operand(REG_R1));
	}
;

/*
 * ALU Operand:	A    B
 *	add	%rd, %r1	%rd = (A + B)
 *	adc	%rd, %r1	%rd = (A + B + carry)
 *	sub	%rd, %r1	%rd = (A - B)
 *	sbc	%rd, %r1	%rd = (A - B - !carry)
 *	and	%rd, %r1	%rd = (A & B)
 *	or	%rd, %r1	%rd = (A | B)
 *	xor	%rd, %r1	%rd = (A ^ B)
 *	cmp	%rd, %r1	flags <- (A - B)
 *
 * There is a lightly irregular syntax for the subtraction-based
 * operations.  This is because we are stuck with the implied %r1
 * operand, but swapping the arguments is useful for the negative
 * result.  gen_alu_insn() handles this, and also enforces the %r1
 * restrictions.
 *
 * ALU Operand:	B    A
 *	sub	%r1, %rd	%rd = (B - A)
 *	sbc	%r1, %rd	%rd = (B - A - !carry)
 *	cmp	%r1, %rd	flags <- (B - A)
 *
 * This is all a very long-winded way to say that %r1 is always the
 * B operand to the ALU.
 */
alu_2opr_insn:
	  alu_2opr_which WS GP_REG opr_sep GP_REG
	{
		gen_alu_insn($1,
		    gen_reg_operand($3),
		    gen_reg_operand($5));
	}
;

alu_1opr_which:
	  INSN_INC		{ $$ = $1; }
	| INSN_DEC		{ $$ = $1; }
	| INSN_NOT		{ $$ = $1; }
	| INSN_TST		{ $$ = $1; }
;

alu_2opr_which:
	  INSN_ADD		{ $$ = $1; }
	| INSN_ADC		{ $$ = $1; }
	| INSN_SUB		{ $$ = $1; }
	| INSN_SBC		{ $$ = $1; }
	| INSN_AND		{ $$ = $1; }
	| INSN_OR		{ $$ = $1; }
	| INSN_XOR		{ $$ = $1; }
	| INSN_CMP		{ $$ = $1; }
;

%%

/*****************************************************************************
 * Error / debug message support
 *****************************************************************************/

static int error_count;
static bool debug_insn = false;
static bool debug_pass1 = false;
static bool debug_pass2 = false;
static bool debug_pass3 = false;

static void
errmsg(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

static void
aserror(const char *fmt, ...)
{
	va_list ap;

	if (yyline == -1) {
		fprintf(stderr, "error: ");
	} else {
		fprintf(stderr, "error: line %d: ", yyline);
	}
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);

	error_count++;
}

static void
asdebug(bool which_debug, const char *fmt, ...)
{
	va_list ap;

	if (! which_debug) {
		return;
	}

	if (yyline == -1) {
		printf("debug: ");
	} else {
		printf("debug: line %d: ", yyline);
	}
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fputc('\n', stdout);
}

static void
asmsg(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fputc('\n', stdout);
}

static const char *
plural(long val)
{
	return val == 1 ? "" : "s";
}

#define	OPERAND_DESC_BUFSIZE	128

static const char * const opc_names[] = {
	[OPC_MOV]	=	"MOV",
	[OPC_LD]	=	"LD",
	[OPC_ST]	=	"ST",
	[OPC_ALU]	=	"ALU",
};

static const char *
opc_name(unsigned int opc)
{
	assert(opc <= OPC_ALU);
	return opc_names[opc];
}

static const char * const regnames[] = {
	[REG_R0]	=	"%r0",
	[REG_R1]	=	"%r1",
	[REG_R2]	=	"%r2",
	[REG_R3]	=	"%r3",
	[REG_SP]	=	"%sp",
	[REG_PC]	=	"%pc",
	[REG_SPa]	=	"SPa",
	[REG_IMM]	=	"IMM",
};

static const char *
register_name(unsigned int regnum)
{
	assert(regnum <= REG_IMM);
	return regnames[regnum];
}

static const char * const alu_fn_names[] = {
	[ALU_FN_INC_A]		=	"INC_A",
	[ALU_FN_B_MINUS_A]	=	"B_MINUS_A",
	[ALU_FN_A_MINUS_B]	=	"A_MINUS_B",
	[ALU_FN_A_PLUS_B]	=	"A_PLUS_B",
	[ALU_FN_A_XOR_B]	=	"A_XOR_B",
	[ALU_FN_A_OR_B]		=	"A_OR_B",
	[ALU_FN_A_AND_B]	=	"A_AND_B",
	[ALU_FN_NOT_A]		=	"NOT_A",
};

static const char *
alu_fn_name(unsigned int fn)
{
	assert(fn <= ALU_FN_NOT_A);
	return alu_fn_names[fn];
}

static const char * const ccode_names[] = {
	[CJMP_C]	=	"C",
	[CJMP_Z]	=	"Z",
	[CJMP_O]	=	"O",
	[CJMP_N]	=	"N",
};

static const char *
ccode_name(unsigned int ccode)
{
	assert(ccode <= CJMP_N && ccode_names[ccode] != NULL);
	return ccode_names[ccode];
}

static const char *
operand_desc(struct operand *o, char *buf)
{
	switch (o->type) {
	case OPR_REG:
		snprintf(buf, OPERAND_DESC_BUFSIZE,
		    "REGISTER %s", register_name(o->regnum));
		break;
	
	case OPR_IMM:
		snprintf(buf, OPERAND_DESC_BUFSIZE,
		    "IMMEDIATE %ld (%d)", o->immval, (int)(int8_t)o->immval);
		break;
	
	case OPR_LABEL:
		snprintf(buf, OPERAND_DESC_BUFSIZE,
		    "LABEL %s", o->label->ident);
		break;
	
	case OPR_ALU:
		snprintf(buf, OPERAND_DESC_BUFSIZE,
		    "ALU %s%s", o->alu.carry ? "C " : "",
		    alu_fn_name(o->alu.fn));
		break;

	case OPR_CC:
		snprintf(buf, OPERAND_DESC_BUFSIZE,
		    "CC %s", ccode_name(o->ccode));
		break;
	
	default:
		assert(0);
	}

	return buf;
}

static const char *
insn_name(int which)
{
	/* Must be a switch because lex tokens. */
	switch (which) {
	case INSN_MOV:		return "mov";
	case INSN_HLT:		return "hlt";
	case INSN_NOP:		return "nop";
	case INSN_JMP:		return "jmp";
	case INSN_JC:		return "jc";
	case INSN_JNC:		return "jnc";
	case INSN_JZ:		return "jz";
	case INSN_JNZ:		return "jnz";
	case INSN_JO:		return "jo";
	case INSN_JNO:		return "jno";
	case INSN_JN:		return "jn";
	case INSN_JNN:		return "jnn";
	case INSN_CALL:		return "call";
	case INSN_SPA:		return "spa";
	case INSN_LD:		return "ld";
	case INSN_POP:		return "pop";
	case INSN_RET:		return "ret";
	case INSN_INB:		return "inb";
	case INSN_LDI:		return "ldi";
	case INSN_ST:		return "st";
	case INSN_PUSH:		return "push";
	case INSN_OUTB:		return "outb";
	case INSN_STI:		return "sti";
	case INSN_INC:		return "inc";
	case INSN_DEC:		return "dec";
	case INSN_ADD:		return "add";
	case INSN_ADC:		return "adc";
	case INSN_SUB:		return "sub";
	case INSN_SBC:		return "sbc";
	case INSN_AND:		return "and";
	case INSN_OR:		return "or";
	case INSN_XOR:		return "xor";
	case INSN_NOT:		return "not";
	case INSN_CMP:		return "cmp";
	case INSN_TST:		return "tst";
	default:		assert(0);
	}
}

static void
yyerror(const char *msg)
{
	aserror("%s", msg);
}

/*****************************************************************************
 * Abstract code generation
 *****************************************************************************/

static struct operand *
gen_operand(operand_type type)
{
	struct operand *op = calloc(1, sizeof(*op));
	assert(op != NULL);
	op->type = type;
	return op;
}

static struct operand *
gen_reg_operand(int regnum)
{
	struct operand *op = gen_operand(OPR_REG);
	op->regnum = regnum;
	return op;
}

static struct operand *
gen_label_operand(char *label)
{
	struct operand *op = gen_operand(OPR_LABEL);
	op->label = gen_labelref(label);
	return op;
}

static struct operand *
gen_imm_operand(long immval)
{
	struct operand *op = gen_operand(OPR_IMM);

	if (immval > UINT8_MAX || immval < INT8_MIN) {
		aserror("immediate value '%ld' out of range.", immval);
	}

	op->immval = immval;
	return op;
}

static struct operand *
gen_alu_operand(unsigned int fn, bool carry)
{
	struct operand *op = gen_operand(OPR_ALU);
	op->alu.fn = fn;
	op->alu.carry = carry;
	return op;
}

static struct operand *
gen_cc_operand(int ccode)
{
	struct operand *op = gen_operand(OPR_CC);
	op->ccode = ccode;
	return op;
}

static void
gen_program_node(node_type type, void *obj)
{
	struct program_node *p = calloc(1, sizeof(*p));
	assert(p != NULL);
	p->type = type;
	p->object = obj;

	TAILQ_INSERT_TAIL(&all_nodes, p, link);
}

static struct label *
label_lookup(const char *name)
{
	struct label *l;

	LIST_FOREACH(l, &all_labels, link) {
		if (strcmp(l->ident, name) == 0) {
			return l;
		}
	}
	return NULL;
}

static struct label *
gen_label(char *name)
{
	struct label *l = label_lookup(name);

	if (l != NULL) {
		free(name);	/* strdup()'d in scan.l */
		return l;
	}

	l = calloc(1, sizeof(*l));
	assert(l != NULL);

	l->ident = name;	/* take ownership of this buffer */
	LIST_INSERT_HEAD(&all_labels, l, link);
	return l;
}

static void
gen_label_decl(char *name)
{
	struct label *l = gen_label(name);

	/*
	 * If this label is resolved or already has a line number,
	 * then this is a duplicate, which is an error.
	 */
	if (l->resolved || l->lineno != 0) {
		aserror("duplicate label: '%s' (first declared at line %d)",
		    l->ident, l->lineno);
		return;
	}

	/*
	 * Label declarations remain unresolved until we assign
	 * addresses at codegen time, but they get a line number
	 * assignment.
	 */
	l->lineno = yyline;

	asdebug(debug_insn, "LABEL DECL: %s (@ line %d)", l->ident, yyline);

	gen_program_node(PGN_LABEL, l);
}

static void
gen_label_assignment(char *name, long value)
{
	struct label *l = gen_label(name);

	/* As above. */
	if (l->resolved || l->lineno != 0) {
		aserror("duplicate label: '%s' (first declared at line %d)",
		    l->ident, l->lineno);
		return;
	}

	l->lineno = yyline;
	l->value = value;
	l->resolved = true;

	asdebug(debug_insn, "LABEL ASSIGNMENT: %s %ld (@ line %d)",
	    l->ident, l->value, yyline);

	/*
	 * Label assignments don't need program nodes; they just
	 * go into the symbol table.
	 */
}

static struct label *
gen_labelref(char *name)
{
	struct label *l = gen_label(name);
	assert(l != NULL);
	return l;
}

static struct insn *
gen_insn(unsigned int opc)
{
	struct insn *i = calloc(1, sizeof(*i));
	assert(i != NULL);
	i->opc = opc;
	i->lineno = yyline;
	return i;
}

static void
gen_insn2(unsigned int opc, struct operand *opr1, struct operand *opr2)
{
	struct insn *i = gen_insn(opc);
	char opr1_desc[OPERAND_DESC_BUFSIZE];
	char opr2_desc[OPERAND_DESC_BUFSIZE];

	i->operands[0] = opr1;
	i->operands[1] = opr2;

	asdebug(debug_insn, "INSN:   %s (@ line %d)", opc_name(opc), yyline);
	asdebug(debug_insn, "  OPR1: %s", operand_desc(opr1, opr1_desc));
	asdebug(debug_insn, "  OPR2: %s", operand_desc(opr2, opr2_desc));

	gen_program_node(PGN_INSN, i);
}

static void
gen_insn3(unsigned int opc, struct operand * opr1, struct operand *opr2,
	  struct operand *opr3)
{
	struct insn *i = gen_insn(opc);
	char opr1_desc[OPERAND_DESC_BUFSIZE];
	char opr2_desc[OPERAND_DESC_BUFSIZE];
	char opr3_desc[OPERAND_DESC_BUFSIZE];

	i->operands[0] = opr1;
	i->operands[1] = opr2;
	i->operands[2] = opr3;

	/* opr3 must always be an IMMEDIATE or LABEL. */
	assert(opr3->type == OPR_LABEL || opr3->type == OPR_IMM);

	asdebug(debug_insn, "INSN:   %s (@ line %d)", opc_name(opc), yyline);
	asdebug(debug_insn, "  OPR1: %s", operand_desc(opr1, opr1_desc));
	asdebug(debug_insn, "  OPR2: %s", operand_desc(opr2, opr2_desc));
	asdebug(debug_insn, "  OPR3: %s", operand_desc(opr3, opr3_desc));

	gen_program_node(PGN_INSN, i);
}

static void
gen_cond_jmp(int which, struct operand *opr3)
{
	unsigned int flag;
	unsigned int opr1;

	switch (which) {
	case INSN_JC:
	case INSN_JNC:
		flag = CJMP_C;
		break;
	
	case INSN_JZ:
	case INSN_JNZ:
		flag = CJMP_Z;
		break;
	
	case INSN_JO:
	case INSN_JNO:
		flag = CJMP_O;
		break;
	
	case INSN_JN:
	case INSN_JNN:
		flag = CJMP_N;
		break;
	
	default:
		assert(0);
	}

	switch (which) {
	case INSN_JC:
	case INSN_JZ:
	case INSN_JO:
	case INSN_JN:
		opr1 = REG_IMM;
		break;
	
	case INSN_JNC:
	case INSN_JNZ:
	case INSN_JNO:
	case INSN_JNN:
		opr1 = REG_SPa;
		break;
	
	default:
		assert(0);
	}

	gen_insn3(OPC_MOV, gen_reg_operand(opr1), gen_cc_operand(flag), opr3);
}

static void
gen_alu_insn(unsigned int which, struct operand *opr1, struct operand *opr2)
{
	bool carry = false;
	struct operand *a_opr = opr1;
	unsigned int fn;

	/* All ALU operands must be registers. */
	assert(opr1->type == OPR_REG);
	assert(opr2->type == OPR_REG);

	/*
	 * SUB / SBC / CMP -- at least one operand must be %r1, and the
	 * non-%r1 operand gets the result (unless both operands are %r1,
	 * in which case obviously %r1 will get the result).
	 *
	 * All others -- the second operand must be %r1, and the first
	 * operand gets the result.
	 */
	switch (which) {
	case INSN_SUB:
	case INSN_SBC:
	case INSN_CMP:
		if (opr1->regnum != REG_R1 && opr2->regnum != REG_R1) {
			aserror("\"%s %s, %s\" -- "
			    "illegal register combination, one must be %s",
			    insn_name(which),
			    register_name(opr1->regnum),
			    register_name(opr2->regnum),
			    register_name(REG_R1));
			return;
		}

		/*
		 * If the callers wants B - A, use opr2 in the opcode.
		 * We'll detect this below and use the B_MINUS_A funcion.
		 */
		if (opr1->regnum == REG_R1) {
			a_opr = opr2;
		}
		break;
	
	default:
		if (opr2->regnum != REG_R1) {
			aserror("\"%s %s, %s\" -- "
			    "second operand must be %s",
			    insn_name(which),
			    register_name(opr1->regnum),
			    register_name(opr2->regnum),
			    register_name(REG_R1));
			return;
		}
		break;
	}

	/* Some of these encodings are weird - trust the microcode. */
	switch (which) {
	case INSN_INC:
		fn = ALU_FN_INC_A;
		break;

	case INSN_DEC:
		fn = ALU_FN_INC_A;
		carry = true;
		break;

	case INSN_ADD:
		fn = ALU_FN_A_PLUS_B;
		break;

	case INSN_ADC:
		fn = ALU_FN_A_PLUS_B;
		carry = true;
		break;

	case INSN_SUB:
		fn = a_opr == opr2 ? ALU_FN_B_MINUS_A : ALU_FN_A_MINUS_B;
		break;

	case INSN_SBC:
		fn = a_opr == opr2 ? ALU_FN_B_MINUS_A : ALU_FN_A_MINUS_B;
		carry = true;
		break;

	case INSN_AND:
		fn = ALU_FN_A_AND_B;
		break;

	case INSN_OR:
		fn = ALU_FN_A_OR_B;
		break;

	case INSN_XOR:
		fn = ALU_FN_A_XOR_B;
		break;

	case INSN_NOT:
		fn = ALU_FN_NOT_A;
		break;

	case INSN_CMP:
		fn = a_opr == opr2 ? ALU_FN_A_OR_B : ALU_FN_A_AND_B;
		carry = true;
		break;

	case INSN_TST:
		fn = ALU_FN_NOT_A;
		carry = true;
		break;

	default:
		assert(0);
	}

	gen_insn2(OPC_ALU, gen_alu_operand(fn, carry), a_opr);
}

/*****************************************************************************
 * Address assignment and symbol resolution.
 *****************************************************************************/

static void
codegen_pass1(void)
{
	struct program_node *p;

	current_address = 0;

	TAILQ_FOREACH(p, &all_nodes, link) {
		switch (p->type) {
		case PGN_LABEL:
			/*
			 * Unresolved labels get assigned the current address.
			 */
			assert(! p->label->resolved);
			p->label->value = current_address;
			p->label->resolved = true;
			asdebug(debug_pass1,
			    "PASS1: label '%s' at line %d -> %ld",
			    p->label->ident, p->label->lineno,
			    p->label->value);
			break;

		case PGN_INSN:
			/*
			 * Instructions increment the current address.  2
			 * operand instructions are 1 byte, 3 operand
			 * instructions are 2 bytes (the 3rd operand is
			 * always an immediate value / label referece).
			 */
			if (current_address >= 0x100) {
				errmsg("error: I-space overflow at line %d",
				    p->insn->lineno);
				error_count++;
				return;
			}
			int cost = 1;
			if (p->insn->operands[2] != NULL) {
				cost++;
			}
			asdebug(debug_pass1,
			    "PASS1: insn at line %d (addr %ld) "
			    "occupies %d byte%s",
			    p->insn->lineno, current_address, cost,
			    plural(cost));
			current_address += cost;
			break;

		default:
			assert(0);
		}
	}
}

static void
codegen_pass2(void)
{
	struct program_node *p;

	TAILQ_FOREACH(p, &all_nodes, link) {
		/* Only checking for unresolved labels at insn nodes. */
		if (p->type != PGN_INSN) {
			continue;
		}

		/*
		 * All insns should have 2 operands (even if the syntax
		 * only uses 1).
		 */
		assert(p->insn->operands[0] != NULL);
		assert(p->insn->operands[1] != NULL);

		/*
		 * Operand 0 must always be a reg or an ALU function.
		 */
		assert(p->insn->operands[0]->type == OPR_REG ||
		       p->insn->operands[0]->type == OPR_ALU);

		/*
		 * Operand 1 must always be a reg or a condition code.
		 */
		assert(p->insn->operands[1]->type == OPR_REG ||
		       p->insn->operands[1]->type == OPR_CC);

		/*
		 * Operand 3 is optional, and must be an immediate
		 * value or a label reference.
		 */
		if (p->insn->operands[2] == NULL) {
			continue;
		}
		assert(p->insn->operands[2]->type == OPR_IMM ||
		       p->insn->operands[2]->type == OPR_LABEL);

		if (p->insn->operands[2]->type == OPR_LABEL) {
			struct label *l = p->insn->operands[2]->label;

			asdebug(debug_pass2,
			    "PASS2: checking label ref ('%s') at line %d",
			    l->ident, p->insn->lineno);

			if (! l->resolved) {
				errmsg("error: unresolved label '%s' "
				    "at line %d", l->ident, l->lineno);
				error_count++;
			}
		}
	}
}

/*****************************************************************************
 * Concrete code generation.
 *****************************************************************************/

static size_t
fill_alu_insn(const struct insn *i, uint8_t insn_buf[2])
{

	/* No ALU insns have immediates. */
	assert(i->operands[2] == NULL);

	assert(i->operands[0]->type == OPR_ALU);
	assert(i->operands[1]->type == OPR_REG);

	insn_buf[0] = (uint8_t)ALU_OPCODE(i->operands[0]->alu.carry,
	    i->operands[0]->alu.fn, i->operands[1]->regnum);

	asdebug(debug_pass3,
	    "PASS3: program stream bytes for line %d: 0x%02x",
	    i->lineno, insn_buf[0]);

	return 1;
}

static size_t
fill_insn(const struct insn *i, uint8_t insn_buf[2])
{
	unsigned int dreg, sreg;
	long immval;

	/* ALU insns have a different encoding. */
	if (i->opc == OPC_ALU) {
		return fill_alu_insn(i, insn_buf);
	}

	assert(i->operands[0]->type == OPR_REG);
	dreg = i->operands[0]->regnum;
	assert(dreg <= REG_IMM);

	if (i->operands[1]->type == OPR_REG) {
		sreg = i->operands[1]->regnum;
	} else {
		assert(i->operands[1]->type == OPR_CC);
		sreg = i->operands[1]->ccode;
	}
	assert(sreg <= REG_IMM);	/* ccodes fit this, too */

	insn_buf[0] = (uint8_t)OPCODE(i->opc, dreg, sreg);

	if (i->operands[2] != NULL) {
		if (i->operands[2]->type == OPR_IMM) {
			immval = i->operands[2]->immval;
		} else {
			assert(i->operands[2]->type == OPR_LABEL);
			assert(i->operands[2]->label->resolved);
			immval = i->operands[2]->label->value;
		}
		insn_buf[1] = (uint8_t)immval;

		asdebug(debug_pass3,
		    "PASS3: program stream bytes for line %d: 0x%02x 0x%02x",
		    i->lineno, insn_buf[0], insn_buf[1]);

		return 2;
	}

	asdebug(debug_pass3,
	    "PASS3: program stream bytes for line %d: 0x%02x",
	    i->lineno, insn_buf[0]);

	return 1;
}

static void
codegen_pass3(int outfd)
{
	struct program_node *p;
	uint8_t insn_buf[2];
	size_t insn_size;
	ssize_t rv;

	current_address = 0;

	TAILQ_FOREACH(p, &all_nodes, link) {
		switch (p->type) {
		case PGN_LABEL:
			/* Ignore these now. */
			break;

		case PGN_INSN:
			insn_size = fill_insn(p->insn, insn_buf);
			rv = write(outfd, insn_buf, insn_size);
			if (rv != (ssize_t)insn_size) {
				errmsg("error writing output file "
				    "at %d", current_address);
				error_count++;
				return;
			}
			current_address += insn_size;
			insn_count++;
			break;

		/*
		 * If we ever support .org directives, that would
		 * mean a case right here to seek the output file.
		 */

		default:
			assert(0);
		}
	}
}

/*****************************************************************************
 * Main entry point and general "get the process started" stuff.
 *****************************************************************************/

static void
usage(void)
{
	fprintf(stderr,
	    "usage: sot8as [-d dbgtype] [-o outfile] infile\n");
	exit(1);
}

int
main(int argc, char *argv[])
{
	extern FILE *yyin;
	const char *infname;
	const char *outfname = "a.out";
	FILE *infile;
	int outfd;
	int ch;

	while ((ch = getopt(argc, argv, "d:")) != -1) {
		switch (ch) {
		case 'd':
			if (strcmp(optarg, "parser") == 0) {
				yydebug = 1;
			} else if (strcmp(optarg, "insn") == 0) {
				debug_insn = true;
			} else if (strcmp(optarg, "pass1") == 0) {
				debug_pass1 = true;
			} else if (strcmp(optarg, "pass2") == 0) {
				debug_pass2 = true;
			} else if (strcmp(optarg, "pass3") == 0) {
				debug_pass3 = true;
			} else {
				errmsg("unknown debugging option: '%s'",
				    optarg);
			}
			break;

		case 'o':
			outfname = optarg;
			break;

		default:
			usage();
		}
	}

	argc -= optind;
	argv += optind;

	if (argc != 1) {
		usage();
	}
	infname = argv[0];

	outfd = open(outfname, O_WRONLY | O_TRUNC | O_CREAT, 0666);
	if (outfd == -1) {
		errmsg("unable to open outpout file '%s'.", outfname);
		exit(1);
	}

	infile = fopen(infname, "r");
	if (infile == NULL) {
		errmsg("unable to open input file '%s'.", infname);
		exit(1);
	}

	/* Humans think line #s start at 1. */
	yyline = 1;

	yyin = infile;
	yyparse();

	/* We're done with dynamic input lines. */
	yyline = -1;

	if (error_count) {
		asmsg("%d error%s while parsing '%s'.", error_count,
		    plural(error_count), infname);
		exit(1);
	}

	/* First pass code gen -- assign addresses. */
	codegen_pass1();

	if (error_count) {
		asmsg("%d error%s doing code gen pass 1 on '%s'.",
		    error_count, plural(error_count), infname);
		exit(1);
	}

	/* Second pass -- check for unresolved labels. */
	codegen_pass2();

	if (error_count) {
		asmsg("%d error%s doing code gen pass 2 on '%s'.",
		    error_count, plural(error_count), infname);
		exit(1);
	}

	/* Output the binary code. */
	codegen_pass3(outfd);

	if (error_count) {
		asmsg("%d error%s doing code gen pass 3 on '%s'.",
		    error_count, plural(error_count), infname);
	}

	printf("Output %u insn%s for %ld bytes of I-space to '%s'.\n",
	    insn_count, plural(insn_count), current_address, outfname);

	exit(0);
}
