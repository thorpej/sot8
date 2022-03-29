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

/*
 * genucode --
 *
 * Generates microcode for my variation of the James Bates 8-bit CPU,
 * which is in-turn inspired by the Ben Eater 8-bit breadboard CPU.
 *
 * Summary of differences:
 *
 * ==> Decode ROM organization
 *
 * The Bates CPU places the opcode in bits 0-7 of the decode ROM address,
 * and the microcode step number in bits 8-10.  My variation places the
 * microcode step in bits 0-2 and opcode in bits in bits 3-10.  This places
 * all of the microcode steps for a given instruction sequentially in the
 * decode ROM, makes populating the ROM a little more straightforward, and
 * in general seems like a more natural way or organize the decode ROM.
 *
 * ==> Address space identifier
 *
 * The Bates CPU has a PGM signal to select program memory, and if that
 * signal is not asserted then we get data memory.  I want to support
 * additional address spaces, e.g. an I/O space, a stack region, and
 * eventually additional RAM.  So, the PGM signal is replaced with a
 * (currently 2-bit) address space ID that is then input into a binary
 * decoder to select one of 4 address spaces.
 *
 * This removes the PGM signal and adds AS0 and AS1 signals.  These signals
 * are decoded by the memory module.
 *
 * ==> IMMEDIATE VALUE register on control unit
 *
 * In order to support an IMMEDIATE addressing mode for more instructions
 * (e.g. CALL), I've added an Immediate Value (IV) register to the control
 * unit that can hold an additional 8-bits of immediate data during the
 * instruction fetch cycles.  Immediate data will always be loaded into IV,
 * and will also be reflected into any other destination register as needed.
 * The IV register is cleared at the end of every instruction (by snooping
 * the assertion of the _ucSR signal).
 *
 * This adds _IVE and _IVW signals.
 *
 * ==> Changes to the CALL instruction
 *
 * The Bates CPU implicitly uses Rc to hold the target of the CALL
 * instruction.  The reason for this is because his CALL instruction is
 * encoded as "PUSH PC", which is actually "ST [SPa], PC", which consumes
 * both register slots in the opcode.  This is inefficient because the
 * typical use of CALL is to call a function at a label.  A label, of course,
 * is simply an immediate value, which means that the typical usage requires
 * 2 instructions:
 *
 *	MOV	Rc, some_label
 *	CALL	Rc
 *
 * So, what I've done here is removed the magic from "PUSH PC".  This allows
 * the atypical case to still be used in an open-coded manner:
 *
 *	PUSH	PC
 *	MOV	PC, Rc
 *
 * ...but allows the typical usage to be encoded in 2 bytes instead of 3,
 * which is important when you only have 256 bytes of program space!
 *
 * Our new syntax is:
 *
 *	CALL	some_label
 *
 * which uses the encoding "MOV SPa, PC" plus an immediate value.  This
 * does what you might expect: pushes the PC onto the stack and then moves
 * the immediate value into the program counter.  This instruction uses the
 * IV register to hold the CALL destination while the PUSH portion of the
 * instruction is being executed.
 *
 * ==> Changes to the ALU
 *
 * Bates' CPU hard-codes the B-operand of the ALU to either 0 or Rb.
 * This is largely because the opcode doesn't have room for both the
 * ALU function *and* 2 registers.  However, this is baked into his
 * hardware; Rb has a back-door input into the ALU.  I don't like this
 * for a couple of reasons:
 *
 *	1. It makes Rb hardware different from all of the other GPRs,
 *	   and I'd rather that not be the case.
 *
 *	2. It limits our ability to use the ALU for some other things
 *	   (see below).
 *
 * So, instead of the selector bank that selects Rb or 0, we add an
 * ALU_B register that is cleared at the end of every instruction
 * (with the assertion of _ucSR).  This provides the hard-coded 0 for
 * instructions that require it.  Then, instead of an _ALB signal
 * to use the back-door from Rb, we burn a microcode cycle copying Rb
 * into ALU_B for instruction that operate on Rb.  This costs us a
 * microcode cycle for these common operations, but raw performance is
 * not necessarily the goal, and this design unlocks some additional
 * capabilities that we'll see below.
 *
 * Side note: because this means that the ALU now has 2 registers (B-operand
 * and the result register), I am renaming the result register to ALU_R.
 *
 * ==> Changes to Condition Codes
 *
 * The condition codes work somewhat differently on this CPU, compared to
 * the Bates design.  The CC register is not located with the ALU in this
 * design.  Instead, it resides in the control unit and can be updated from
 * two different places: the ALU (by asserting the ACC signal) or the bus
 * (by asserting BCC signal).  This allows us to do a couple of useful things:
 *
 * a) selectively update the flags from the ALU, in case we're not doing a
 * strictly arithmentic operation (like SP-relative loads/stores; see below).
 *
 * b) update the Z and N flags during a LD or MOV into one of the general
 * purpose registers, which can reduce trhe following pattern:
 *
 *	LD	Ra, #1[SP]
 *	TST	Ra
 *	JZ	$somewhere
 *
 * to this:
 *
 *	LD	RA, #1[SP]
 *	JZ	$somewhere
 *
 * This is done by placing a selector in front of the CC register and
 * using some combinational logic to enable the register input when
 * either ACC or BCC is asserted.
 *
 * ==> SP-relative loads/stores
 *
 * Because this variation has so much stack space and a more useful
 * CALL instruction, it would be kind of nice to be able to pass
 * function arguments on the stack.  But accessing them in the callee
 * is really tough without the ability to load from an offset relative
 * to the stack pointer.  Happily, the ALU_B register makes it really
 * easy for us to do this; all we need to do is find a pair of opcodes
 * that we can hijack for it.  Limiting these to the GPRs seems pretty
 * reasonable, so we use "LD SPa, Ra" for an SP-relative load, and
 * "ST Ra, SPa" for an SP-relative store.  Each opcode is followed by
 * an immediate value to use as the offset.  Yes, in the encoding the
 * operands are opposite of the normal usage, but hey, you're constrained
 * in 8 bits!
 *
 * ==> PUSH #IMM instruction
 *
 * Since we can use the stack to pass arguments around, it might be useful
 * to be able to push an immediate value onto the stack directly, like so:
 *
 *	PUSH	#IMM
 *
 * ...rather than having to do:
 *
 *	MOV	Rc, #IMM
 *	PUSH	Rc
 *
 * ...so we do just that by hijacking "ST SPa, IMM".
 *
 * ==> SPA (Stack Pointer Adjust) instruction
 *
 * ...and now that we've pushed all of those arguments onto the stack,
 * it would be really nice to be able to adjust the stack pointer once
 * our function returns, without having to clobber a register or burn
 * multiple instructions, so we provide a:
 *
 *	SPA	#IMM
 *
 * instruction to do this, hijacking the "MOV SPa, IMM" opcode for it.
 *
 * ==> I-space loads (and stores, because YOLO)
 *
 * Loads from I-space are useful for e.g. loading tables in from ROM.
 * The Bates CPU did this by treating Rc as a magic register for loads
 * and stores.  I wasn't willing to do that without a different opcode.
 * We hijack the "LD Rx, PC" and "ST PC, Rx" opcodes to do this, and
 * ao ahead and make Ra the magic address register; we don't want to use
 * Rb for this purpose, because if we want load from a table with a stride
 * larger then 1, we want to be able to pre-load Rb with the stride for
 * an ALU instruction.  The pattern would be like so:
 *
 *	MOV	Ra, some_table	; Ra = some_table
 *	LDI	Rc, [Ra]	; Rc = *Ra
 *	INC	Ra		; Ra++
 *
 * ...or:
 *
 * Lcopytab:
 *	MOV	Ra, some_sparse_table
 *	MOV	Rb, #2
 *	MOV	Rd, #10
 *	LDI	Rc, [Ra]
 *	ADD	Ra, Rb
 *	DEC	Rd
 *	JNZ	Lcopytab
 *	.
 *	.
 *	.
 *
 * Also provided is STI just in case you're into self-modifying
 * code (obvs won't work if you hook up I-space to a ROM).
 *
 * ==> Additional INB / OUTB instructions
 *
 * To support I/O space, INB and OUTB instructions are added to
 * the LD and ST classes.  These instructions all use an 8-bit
 * immediate address.  These instructions only allow I/O using
 * the general purpose registers, thus saving us a few opcodes
 * for future use.
 *
 * ==> Example of how to use some of these exciting features:
 *
 *	PUSH	Ra		; save Ra
 *	PUSH	#3		; second argument
 *	PUSH	Rc		; first argument
 *	CALL	multiply	; Ra = multiply(Rc, 3)
 *	MOV	Rc, Ra		; Rc = Ra
 *	SPA	#2		; adjust SP over args
 *	POP	Ra		; restore Ra
 *	.
 *	.
 *	.
 * multiply:
 *	LD	Ra, #1[SP]	; Ra = first argument
 *	JZ	Lmul02		; Get out if Ra == 0.
 *	LD	Rc, #2[SP]	; Rc = second argument
 *	JZ	Lmul03		; Get out if Rc == 0.
 *	MOV	Rb, Ra		; Rb = Ra
 * Lmul01:
 * 	DEC	Rc		; Rc--
 *	JZ	Lmul02		; get out if done
 *	ADD	Ra, Rb		; Ra = Ra + Rb
 *	JMP	Lmul01		; and again
 * Lmul02:
 *	RET			; result already in Ra
 * Lmul03:
 *	MOV	Ra, #0		; return 0
 *	RET
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sot8defs.h"

typedef	uint32_t	signal_word;
#define	NSTEPS(x)	(sizeof(x) / sizeof((x)[0]))

#define	BIT(x)		((signal_word)1 << (x))

/*
 * The Bates CPU microcode is organized around a 32-bit control signal word
 * that is then split across 4 8KB EEPROMs.  Signal names that start with
 * _ are active-low.  They are encoded in the table as active high when
 * the table is built, but a final pass to flip them before programming
 * is performed before writing out the ROM images.
 */
	/* Bank A - ALU signals */
#define	ALC	BIT(0)		/* ALU carry input */
#define	ALS0	BIT(1)		/* ALU selector; see below */
#define	ALS1	BIT(2)		/* ALU selector; see below */
#define	ALS2	BIT(3)		/* ALU selector; see below */
#define	_ALB	BIT(4)		/* ALU_B write */
#define	_ALW	BIT(5)		/* ALU_R write */
#define	_ALE	BIT(6)		/* ALU_R enable */
#define	ACC	BIT(7)		/* ALU sets condition codes */
#define	BANK_A_ACTIVE_LOW	(_ALB | _ALW | _ALE)

	/* Bank B - General Purpose register signals */
#define	_RaW	BIT(8)		/* Ra write */
#define	_RaE	BIT(9)		/* Ra enable */
#define	_RbW	BIT(10)		/* Rb write */
#define	_RbE	BIT(11)		/* Rb enable */
#define	_RcW	BIT(12)		/* Rc write */
#define	_RcE	BIT(13)		/* Rc enable */
#define	_RdW	BIT(14)		/* Rd write */
#define	_RdE	BIT(15)		/* Rd enable */
#define	BANK_B_ACTIVE_LOW	((signal_word)0xff << 8)

	/* Bank C - Special register signals */
#define	_SPW	BIT(16)		/* SP write */
#define	_SPE	BIT(17)		/* SP enable */
#define	_PCW	BIT(18)		/* PC write */
#define	_PCE	BIT(19)		/* PC enable */
#define	PCA	BIT(20)		/* PC advance */
#define	_IRW	BIT(21)		/* Insn register write */
#define	_IVW	BIT(21)		/* IV write */
#define	_IVE	BIT(22)		/* IV enable */
#define	BANK_C_ACTIVE_LOW	(_SPW | _SPE | _PCW | _PCE | _IRW | _IVE | _IVW)

	/* Bank D - Memory and misc control signals */
#define	_MAW	BIT(24)		/* Memory address write */
#define	_MW	BIT(25)		/* Memory write */
#define	_ME	BIT(26)		/* Memory enable */
#define	AS0	BIT(27)		/* Address Space selector; see below */
#define	AS1	BIT(28)		/* Address Space selector; see below */
#define	BCC	BIT(29)		/* Bus sets condition codes (only Z and N) */
#define	_HLT	BIT(30)		/* HALT */
#define	_ucSR	BIT(31)		/* microcode step reset (_TR in Bases' code) */
#define	BANK_D_ACTIVE_LOW	(_MAW | _MW | _ME | _HLT | _ucSR)

#define	ALS(x)	((signal_word)(x) << 1)

#define	AS_D	((signal_word)0 << 27)	/* data space (default) */
#define	AS_I	((signal_word)1 << 27)	/* instruction space */
#define	AS_S	((signal_word)2 << 27)	/* stack space */
#define	AS_IO	((signal_word)3 << 27)	/* I/O space */

/* Short-hand for some common microcode statements. */
#define	MEM(as)		((as) | _ME)
#define	MEMW(as)	((as) | _MW)
#define	IMMVAL(w)	((w)  | _IVW | MEM(AS_I))

#define	ACTIVE_LOW_SIGNALS						\
	(BANK_A_ACTIVE_LOW | BANK_B_ACTIVE_LOW | BANK_C_ACTIVE_LOW |	\
	 BANK_D_ACTIVE_LOW)

/*
 * Decode ROM address is built from:
 *
 * bit 12 -- condition flag
 * bit 11 -- carry flag
 * bits 3-10 -- opcode
 * bits 0-2 -- microcode step (8 total microcode steps)
 *
 * NOTE: THIS IS DIFFERENT THAN THE WAY BATES WIRED UP HIS CONTROL UNIT.
 * The Bates CU has the opcode in bits 0-7 and the step in bits 8-11.
 *
 * Because there are 13 inputs into the decode ROM, this means we have
 * 8192 control signal states.
 *
 * N.B. the first 2 microcode steps are always consumed by the
 * instruction fetch cycles, so 6 are available for each instruction.
 */
#define	ROM_ENTRY_COUNT	(1U << 13)
#define	ROM_ADDR_COND	BIT(12)
#define	ROM_ADDR_CARRY	BIT(11)
#define	INSN_STEP_INDEX(opc, ca, co, t)					\
	(((opc) << 3) |							\
	 ((ca) ? ROM_ADDR_CARRY : 0) |					\
	 ((co) ? ROM_ADDR_COND : 0) |					\
	 (t))

/*
 * Since each instruction can have up to 8 steps, there are this many
 * possible instructions.
 */
#define	MAX_INSNS	(ROM_ENTRY_COUNT / 8)

/* Opcode class short-hand. */
#define	MOV	OPC_MOV
#define	LD	OPC_LD
#define	ST	OPC_ST
#define	ALU	OPC_ALU

static const char *opcode_classes[4] = {
	[MOV]	=	"MOV",
	[LD]	=	"LD",
	[ST]	=	"ST",
	[ALU]	=	"ALU",
};

/* Register name short-hand. */
#define	Ra	REG_R0
#define	Rb	REG_R1
#define	Rc	REG_R2
#define	Rd	REG_R3
#define	SP	REG_SP
#define	PC	REG_PC
#define	SPa	REG_SPa	/* pseudo; SP with auto-inc/dec */
#define	IMM	REG_IMM	/* pseudo; immediate value follows */

static const char *register_names[8] = {
	[Ra]	=	"Ra",
	[Rb]	=	"Rb",
	[Rc]	=	"Rc",
	[Rd]	=	"Rd",
	[SP]	=	"SP",
	[PC]	=	"PC",
	[SPa]	=	"SPa",
	[IMM]	=	"IMM",
};

/* Short-hand for the ALU functions. */
#define	INC_A		ALU_FN_INC_A		/* pseudo */
#define	B_MINUS_A	ALU_FN_B_MINUS_A
#define	A_MINUS_B	ALU_FN_A_MINUS_B
#define	A_PLUS_B	ALU_FN_A_PLUS_B
#define	A_XOR_B		ALU_FN_A_XOR_B
#define	A_OR_B		ALU_FN_A_OR_B
#define	A_AND_B		ALU_FN_A_AND_B
#define	NOT_A		ALU_FN_NOT_A		/* pseudo */

static const char *alu_function_names[8] = {
	[INC_A]		=	"INC_A",
	[B_MINUS_A]	=	"B_MINUS_A",
	[A_MINUS_B]	=	"A_MINUS_B",
	[A_PLUS_B]	=	"A_PLUS_B",
	[A_XOR_B]	=	"A_XOR_B",
	[A_OR_B]	=	"A_OR_B",
	[A_AND_B]	=	"A_AND_B",
	[NOT_A]		=	"NOT_A",
};

signal_word decode_rom[ROM_ENTRY_COUNT];
unsigned int total_insns;

uint8_t insn_bitmap[MAX_INSNS / 8];
uint8_t opcode_bitmap[256 / 8];

static void
insn_mark(unsigned int insn_index)
{
	assert(insn_index < MAX_INSNS);
	unsigned int byte = insn_index / 8;
	unsigned int bit = 1 << (insn_index & 7);

	assert((insn_bitmap[byte] & bit) == 0);
	insn_bitmap[byte] |= bit;
}

static void
opcode_mark(unsigned int opcode)
{
	assert(opcode < 256);
	unsigned int byte = opcode / 8;
	unsigned int bit = 1 << (opcode & 7);

	/*
	 * Don't care about "duplicate" opcodes; we catch them in
	 * the insn_bitmap above.  We just want to be able to
	 * report unused ones later.
	 */
	opcode_bitmap[byte] |= bit;
}

static bool
opcode_used(unsigned int opcode)
{
	assert(opcode < 256);
	unsigned int byte = opcode / 8;
	unsigned int bit = 1 << (opcode & 7);

	return !!(opcode_bitmap[byte] & bit);
}

static signal_word
_E(unsigned int reg)
{
	static const signal_word esigs[] = {
		[Ra] = _RaE,
		[Rb] = _RbE,
		[Rc] = _RcE,
		[Rd] = _RdE,
		[SP] = _SPE,
		[PC] = _PCE,
	};
	assert(reg <= PC);
	return esigs[reg];
}

static signal_word
_W(unsigned int reg)
{
	static const signal_word wsigs[] = {
		[Ra] = _RaW,
		[Rb] = _RbW,
		[Rc] = _RcW,
		[Rd] = _RdW,
		[SP] = _SPW,
		[PC] = _PCW,
	};
	assert(reg <= PC);
	return wsigs[reg];
}

static signal_word
AS(unsigned int reg)
{
	static const signal_word assigs[] = {
		[Ra] = AS_D,
		[Rb] = AS_D,
		[Rc] = AS_D,
		[Rd] = AS_D,
		[SP] = AS_S,
		[PC] = AS_I,
	};
	assert(reg <= PC);
	return assigs[reg];
}

static signal_word
reg_BCC(unsigned int reg)
{
	return reg == PC ? 0 : BCC;
}

/*
 * decode_rom_init --
 *
 * Initialize the decode ROM to default values.
 */
static void
decode_rom_init(void)
{
	static const signal_word default_insn[8] = {
	/*
	 * The first two steps of every insn are the insn fetch steps:
	 */
		/* MA <- PC */
		[0] = _MAW | _PCE,

		/* IR <- MEM[I], advance PC */
		[1] = _IRW | MEM(AS_I) | PCA,

	/*
	 * Fill the remaining slots with _HLT | _ucSR since we don't have
	 * illegal instruction traps.
	 */
		[2] = _HLT | _ucSR,
		[3] = _HLT | _ucSR,
		[4] = _HLT | _ucSR,
		[5] = _HLT | _ucSR,
		[6] = _HLT | _ucSR,
		[7] = _HLT | _ucSR,
	};
	unsigned int i;

	printf("Initializing decode ROM...");

	for (i = 0; i < ROM_ENTRY_COUNT; i += 8) {
		memcpy(&decode_rom[i], default_insn, sizeof(default_insn));
	}

	printf("done.\n");
}

/*
 * decode_rom_fini --
 *
 * Perform a final pass over the decode ROM and invert the active-low
 * signal bits.
 */
static void
decode_rom_fini(void)
{
	unsigned int i;

	printf("Fixing up active-low signals...");

	for (i = 0; i < ROM_ENTRY_COUNT; i++) {
		decode_rom[i] ^= ACTIVE_LOW_SIGNALS;
	}

	printf("done.\n");
}

/*
 * fill_gen_insn --
 *
 * Copy the microcode steps for the specified generic instruction into
 * the decode ROM.  Only the (up to 6) specific steps for the instruction
 * are specified and copied into the ROM (after the insn fetch steps).
 */
static void
fill_gen_insn(unsigned int opcode, bool carry, bool cond,
    signal_word *steps, unsigned int nsteps)
{
	assert(nsteps <= 6);

	unsigned int insn_index =
	    INSN_STEP_INDEX(opcode, carry, cond, 0) / 8;

	insn_mark(insn_index);
	opcode_mark(opcode);

	/* Start after the instruction fetch steps. */
	unsigned int slot = INSN_STEP_INDEX(opcode, carry, cond, 2);

	assert(slot <= ROM_ENTRY_COUNT - 6);

	/* Copy the steps. */
	memcpy(&decode_rom[slot], steps, nsteps * sizeof(*steps));

	/* Terminate the insn with _ucSR. */
	decode_rom[slot + nsteps - 1] |= _ucSR;

	/* Count it for reporting. */
	total_insns++;
}

/*
 * fill_cond_insn --
 *
 * Copy microcode steps for a flags-conditional instruction.
 */
static void
fill_cond_insn(unsigned int opcode, bool cond,
    signal_word *steps, unsigned int nsteps)
{
	/* Write 2 copies; one for each "don't care" carry state. */
	fill_gen_insn(opcode, false, cond, steps, nsteps);
	fill_gen_insn(opcode, true, cond, steps, nsteps);
}

/*
 * fill_carry_insns --
 *
 * Copy microcode steps for a carry-conditional instruction.
 */
static void
fill_carry_insn(unsigned int opcode, bool carry,
    signal_word *steps, unsigned int nsteps)
{
	/* Write 2 copies; one for each "don't care" flag state. */
	fill_gen_insn(opcode, carry, false, steps, nsteps);
	fill_gen_insn(opcode, carry, true, steps, nsteps);
}

/*
 * fill_insn --
 *
 * Copy microcode steps for a non-conditional instruction.
 */
static void
fill_insn(unsigned int opcode, signal_word *steps, unsigned int nsteps)
{
	/* Write 4 copies; two for each "don't care" flag state. */
	fill_cond_insn(opcode, false, steps, nsteps);
	fill_cond_insn(opcode, true, steps, nsteps);
}

/*
 * gen_imm --
 *
 * Generate the microcode steps to fetch an immediate value into the
 * register corresponding to the specified control signal.  Pass in 0
 * if you want only the IV register.  bcc should be BCC if loading the
 * immediate upates the condition codes, and 0 otherwise.
 *
 * N.B. Consumes 2 microcode steps.
 */
void
gen_imm(signal_word *steps, signal_word regw, signal_word bcc)
{
	/* MA <- PC, advance PC */
	steps[0] = _MAW | _PCE | PCA;

	/* IV,reg <- MEM[I] */
	steps[1] = IMMVAL(regw) | bcc;
}

/*
 * gen_MOV_reg --
 *
 * Generate the register-to-register MOV instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 0   0 |    dst    |    src    |
 * +-------------------------------+
 */
static void
gen_MOV_reg(void)
{
	unsigned int dreg, sreg;
	signal_word steps[1];
	unsigned int prev = total_insns;

	printf("Generating MOV Rd <- Rs...");

	for (dreg = Ra; dreg <= PC; dreg++) {
		for (sreg = Ra; sreg <= PC; sreg++) {
			if (dreg == sreg) {
				/*
				 * MOV to self is a NOP.  Our official NOP
				 * is "MOV Ra, Ra", but we allow all of them.
				 * Note also that "MOV PC, PC" is how we
				 * encode HLT.
				 *
				 * N.B. WE DO NOT UPDATE THE CCs ON A NOP.
				 */
				steps[0] = dreg == PC ? _HLT : 0;
			} else {
				/* Rx <- Ry, CC <- Bus */
				steps[0] = _W(dreg) | _E(sreg) | reg_BCC(dreg);
			}
			fill_insn(OPCODE(MOV, dreg, sreg), steps,
			    NSTEPS(steps));
		}
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_MOV_imm --
 *
 * Generate the immediate-to-register MOV instructions.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   0 |    dst    | 1   1   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_MOV_imm(void)
{
	unsigned int dreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating MOV Rd <- IMM...");

	for (dreg = Ra; dreg <= PC; dreg++) {
		/* Rx <- IMM, CC <- Bus */
		gen_imm(steps, _W(dreg), reg_BCC(dreg));

		fill_insn(OPCODE(MOV, dreg, IMM), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_MOV_cond_jmp --
 *
 * Generate the MOV instructions that implement conditional jumps.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   0 | 1   1   1 |  [flag]   |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 *
 * Flag values:
 * JC -> 0 0 0 [CC_C]
 * JZ -> 0 0 1 [CC_Z]
 * JO -> 0 1 0 [CC_O]
 * JN -> 1 0 0 [CC_N]
 *
 * JNC, JNZ, JNO, JNN -- inverse of the above, encoded as:
 *
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   0 | 1   1   0 |  [flag]   |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 *
 * The bottom 3 bits of the opcode are AND'd with the corresponding ALU
 * flags to form the condition code.
 */
static void
gen_MOV_cond_jmp(void)
{
	signal_word false_steps[1];
	signal_word true_steps[2];
	unsigned int prev = total_insns;

	printf("Generating conditional jumps...");

	/*
	 * These jumps are to an IMM value, so not-taken is just
	 * advancing the PC over the IMM value.
	 */
	false_steps[0] = PCA;

	/*
	 * The taken case is the same for all of them:
	 */

	/* PC <- IMM */
	gen_imm(true_steps, _PCW, 0);

	/* JC */
	fill_carry_insn(OPCODE(MOV, IMM, CC_C), false, false_steps,
	    NSTEPS(false_steps));
	fill_carry_insn(OPCODE(MOV, IMM, CC_C), true, true_steps,
	    NSTEPS(true_steps));

	/* JNC */
	fill_carry_insn(OPCODE(MOV, SPa, CC_C), false, true_steps,
	    NSTEPS(true_steps));
	fill_carry_insn(OPCODE(MOV, SPa, CC_C), true, false_steps,
	    NSTEPS(false_steps));

	unsigned int flag;
	for (flag = CC_Z; flag <= CC_N; flag <<= 1) {
		/* JZ, JO, JN */
		fill_cond_insn(OPCODE(MOV, IMM, flag), false, false_steps,
		    NSTEPS(false_steps));
		fill_cond_insn(OPCODE(MOV, IMM, flag), true, true_steps,
		    NSTEPS(true_steps));

		/* JNZ, JNO, JNN */
		fill_cond_insn(OPCODE(MOV, SPa, flag), false, true_steps,
		    NSTEPS(true_steps));
		fill_cond_insn(OPCODE(MOV, SPa, flag), true, false_steps,
		    NSTEPS(false_steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_CALL --
 *
 * Generate the CALL instruction.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   0 | 1   1   0 | 1   0   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_CALL(void)
{
	signal_word steps[6];
	unsigned int prev = total_insns;

	/*
	 * The CALL instruction performs the following steps:
	 *
	 * - Fetches destination address into IV register.
	 * - Decrements SP.
	 * - Stores PC at new SP.
	 * - Moves IV value into PC.
	 */

	printf("Generating CALL...");

	/* IV <- IMM */
	gen_imm(steps, 0, 0);

	/* ALU_R < SP - 1 */
	steps[2] = _SPE | _ALW | ALS(A_MINUS_B);

	/* SP,MA <- ALU_R */
	steps[3] = _SPW | _MAW | _ALE;

	/* MEM[S] <- PC */
	steps[4] = MEMW(AS_S) | _PCE;

	/* PC <- IV */
	steps[5] = _PCW | _IVE;

	fill_insn(OPCODE(MOV, SPa, PC), steps, NSTEPS(steps));

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_SPA --
 *
 * Generate the SPA (Stack Pointer Adjust) instruction.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   0 | 1   1   0 | 1   1   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_SPA(void)
{
	signal_word steps[4];
	unsigned int prev = total_insns;

	printf("Generating SPA #IMM...");

	/* ALU_B <- IMM */
	gen_imm(steps, _ALB, 0);

	/* ALU_R <- SP + ALU_B */
	steps[2] = _SPE | _ALW | ALS(A_PLUS_B);

	/* SP <- ALU_R */
	steps[3] = _SPW | _ALE;

	fill_insn(OPCODE(MOV, SPa, IMM), steps, NSTEPS(steps));

	printf("done (%u insns).\n", total_insns - prev);
}


/*
 * gen_MOV --
 *
 * Generate the MOV-class instructions.
 */
static void
gen_MOV(void)
{
	gen_MOV_reg();
	gen_MOV_imm();
	gen_MOV_cond_jmp();
	gen_CALL();
	gen_SPA();
}

/*
 * gen_LD_reg --
 *
 * Generate the LD register-indirect instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 0   1 |    dst    |    src    |
 * +-------------------------------+
 *
 */
static void
gen_LD_reg(void)
{
	unsigned int dreg, sreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating LD Rd <- [Rs]...");

	for (dreg = Ra; dreg <= PC; dreg++) {
		/*
		 * N.B. Bates' microcode generator allows PC as a source
		 * register, but PC-relative data addressing doesn't make
		 * sense with a split I and D space.
		 *
		 * Bates' microcode generator ALSO does something a little
		 * wacky... if the sreg is Rc, then the load comes from
		 * I space.  This is NOT intuitive at all, so I'm going to
		 * skip that.  I do want to allow loads from I space (good
		 * for reading data tables from ROM), but I think I want
		 * to use separate opcodes for that.
		 */
		for (sreg = Ra; sreg <= SP; sreg++) {
			/* MA <- Rx */
			steps[0] = _MAW | _E(sreg);

			/* Ry <- MEM[AS], CC <- Bus */
			steps[1] = _W(dreg) | MEM(AS(sreg)) | reg_BCC(dreg);

			fill_insn(OPCODE(LD, dreg, sreg), steps, NSTEPS(steps));
		}
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_LD_imm --
 *
 * Generate the LD immediate-indirect instructions.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   1 |    dst    | 1   1   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_LD_imm(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating LD Rd <- [IMM]...");

	/* MA <- IMM */
	gen_imm(steps, _MAW, 0);

	for (dreg = Ra; dreg <= PC; dreg++) {
		/* Rx <- MEM[D], CC <- Bus */
		steps[2] = _W(dreg) | MEM(AS_D) | reg_BCC(dreg);

		fill_insn(OPCODE(LD, dreg, IMM), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_LD_SP_rel --
 *
 * Generate SP-relative loads.  These are done here because we're
 * hijacking MOV-class opcodes.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   1 | 1   1   0 |    dst    |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_LD_SP_rel(void)
{
	unsigned int dreg;
	signal_word steps[5];
	unsigned int prev = total_insns;

	printf("Generating LD Rx, IMM[SP]...");

	/* ALU_B <- IMM */
	gen_imm(steps, _ALB, 0);

	/* ALU_R <- SP + ALU_B */
	steps[2] = _SPE | _ALW | ALS(A_PLUS_B);

	/* MA <- ALU_R */
	steps[3] = _MAW | _ALE;

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* Rx <- MEM[S], CC <- Bus */
		steps[4] = _W(dreg) | MEM(AS_S) | reg_BCC(dreg);

		fill_insn(OPCODE(LD, SPa, dreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_POP --
 *
 * Generate the POP instruction.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 0   1 |    dst    | 1   1   0 |
 * +-------------------------------+
 */
static void
gen_POP(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating POP...");

	/* MA <- SP, ALU_R <- SP + 1 */
	steps[0] = _MAW | _SPE | _ALW | ALS(A_PLUS_B) | ALC;

	/* SP <- ALU_R */
	steps[1] = _SPW | _ALE;

	for (dreg = Ra; dreg <= PC; dreg++) {
		/* Rx <- MEM(S) (old SP in MA), CC <- Bus */
		steps[2] = _W(dreg) | MEM(AS_S) | reg_BCC(dreg);

		fill_insn(OPCODE(LD, dreg, SPa), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_INB --
 *
 * Generate the INB (load from I/O space) instructions.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 0   1 | 1   1   1 |    dst    |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 *
 * Because we're hijacking the IMM value in the usual dst slot, the
 * dst register is in the src register slot.
 */
static void
gen_INB(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating INB Rd <- [IMM]...");

	/* MA <- IMM */
	gen_imm(steps, _MAW, 0);

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* Rx <- MEM(IO), CC <- Bus */
		steps[2] = _W(dreg) | MEM(AS_IO) | reg_BCC(dreg);

		fill_insn(OPCODE(LD, IMM, dreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_LDI --
 *
 * Generate LDI (load form I-space) instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * | 0   1 |    dst    | 1   0   1 |
 * +-------------------------------+
 */
static void
gen_LDI(void)
{
	unsigned int dreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating LDI Rx <- [Ra]...");

	/* MA <- Ra */
	steps[0] = _MAW | _E(Ra);

	for (dreg = Ra; dreg <= PC; dreg++) {
		/* Rx <- MEM[I], CC <- Bus */
		steps[1] = _W(dreg) | MEM(AS_I), reg_BCC(dreg);

		fill_insn(OPCODE(LD, dreg, PC), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_LD --
 *
 * Generate the LD-class instructions.
 */
static void
gen_LD(void)
{
	gen_LD_reg();
	gen_LD_imm();
	gen_LD_SP_rel();
	gen_POP();
	gen_INB();
	gen_LDI();
}

/*
 * gen_ST_reg --
 *
 * Generate the ST register-indirect instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   0 |    dst    |    src    |
 * +-------------------------------+
 */
static void
gen_ST_reg(void)
{
	unsigned int dreg, sreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating ST [Rd] <- Rs...");

	/*
	 * As per above, PC-relative addressing doesn't make a lot of
	 * sense, even though the original Bates generator supports it.
	 */
	for (dreg = Ra; dreg <= SP; dreg++) {
		for (sreg = Ra; sreg <= PC; sreg++) {
			/* MA <- Rx */
			steps[0] = _MAW | _E(dreg);

			/* MEM[AS] <- Ry */
			steps[1] = MEMW(AS(dreg)) | _W(sreg);

			fill_insn(OPCODE(ST, dreg, sreg), steps, NSTEPS(steps));
		}
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_ST_imm --
 *
 * Generate the ST immediate-indirect instructions.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 1   0 | 1   1   1 |    src    |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_ST_imm(void)
{
	unsigned int sreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating ST [IMM] <- Rs...");

	/* MA <- IMM */
	gen_imm(steps, _MAW, 0);

	for (sreg = Ra; sreg <= PC; sreg++) {
		/* MEM[D] <- Rx */
		steps[2] = MEMW(AS_D) | _E(sreg);

		fill_insn(OPCODE(ST, IMM, sreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_ST_SP_rel --
 *
 * Generate SP-relative stores.  These are done here because we're
 * hijacking MOV-class opcodes.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 1   0 |    src    | 1   1   0 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_ST_SP_rel(void)
{
	unsigned int sreg;
	signal_word steps[5];
	unsigned int prev = total_insns;

	printf("Generating ST IMM[SP], Rx...");

	/* ALU_B <- IMM */
	gen_imm(steps, _ALB, 0);

	/* ALU_R <- SP + ALU_B (suppress flags) */
	steps[2] = _SPE | _ALW | ALS(A_PLUS_B);

	/* MA <- ALU_R */
	steps[3] = _MAW | _ALE;

	for (sreg = Ra; sreg <= Rd; sreg++) {
		/* MEM[S] <- Rx */
		steps[4] = MEMW(AS_S) | _E(sreg);

		fill_insn(OPCODE(ST, sreg, SPa), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_PUSH --
 *
 * Generate the PUSH instruction.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   0 | 1   1   0 |    src    |
 * +-------------------------------+
 */
static void
gen_PUSH(void)
{
	unsigned int sreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating PUSH...");

	/* ALU_R <- SP - 1 */
	steps[0] = _ALW | _SPE | ALS(A_MINUS_B);

	/* MA,SP <- ALU_R */
	steps[1] = _MAW | _SPW | _ALE;

	for (sreg = Ra; sreg <= PC; sreg++) {
		/* MEM[S] <- Rx */
		steps[2] = MEMW(AS_S) | _E(sreg);

		fill_insn(OPCODE(ST, SPa, sreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_PUSH_imm --
 *
 * Generate the PUSH #IMM instruction
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 1   0 | 1   1   0 | 1   1   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 */
static void
gen_PUSH_imm(void)
{
	signal_word steps[5];
	unsigned int prev = total_insns;

	printf("Generating PUSH #IMM...");

	/* IV <- IMM */
	gen_imm(steps, 0, 0);

	/* ALU_R <- SP - 1 */
	steps[2] = _ALW | _SPE | ALS(A_MINUS_B);

	/* MA,SP <- ALU_R */
	steps[3] = _MAW | _SPW | _ALE;

	/* MEM[S] <- IV */
	steps[4] = MEMW(AS_S) | _IVE;

	fill_insn(OPCODE(ST, SPa, IMM), steps, NSTEPS(steps));

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_OUTB --
 *
 * Generate the ST immediate-indirect instructions.
 *
 * Encoding:
 * +-------------------------------+-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+-------------------------------+
 * | 1   0 |    src    | 1   1   1 |        Immediate Value        |
 * +-------------------------------+-------------------------------+
 *
 * Because we're hijacking the IMM value in the usual src slot, the
 * src register is in the dst register slot.
 */
static void
gen_OUTB(void)
{
	unsigned int sreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating OUTB [IMM] <- Rs...");

	/* MA <- IMM */
	gen_imm(steps, _MAW, 0);

	for (sreg = Ra; sreg <= Rd; sreg++) {
		/* MEM[IO] <- Rx */
		steps[2] = MEMW(AS_IO) | _E(sreg);

		fill_insn(OPCODE(ST, sreg, IMM), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_STI --
 *
 * Generate STI (store to I-space, HA!) instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * | 0   1 | 1   0   1 |    src    |
 * +-------------------------------+
 */
static void
gen_STI(void)
{
	unsigned int sreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating STI [Ra] <- Rx...");

	/* MA <- Ra */
	steps[0] = _MAW | _E(Ra);

	for (sreg = Ra; sreg <= PC; sreg++) {
		/* MEM[I] <- Rx */
		steps[1] = MEMW(AS_I) | _E(sreg);

		fill_insn(OPCODE(ST, PC, sreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_ST --
 *
 * Generate the ST-class instructions.
 */
static void
gen_ST(void)
{
	gen_ST_reg();
	gen_ST_imm();
	gen_ST_SP_rel();
	gen_PUSH();
	gen_PUSH_imm();
	gen_OUTB();
	gen_STI();
}

/*
 * gen_INC --
 *
 * Generate the INC instruction.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 0   0   0 |  dst  |
 * +-------------------------------+
 */
static void
gen_INC(void)
{
	unsigned int dreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating INC...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_R <- Rx + 1, CC <- ALU */
		steps[0] = _ALW | _E(dreg) | ALS(A_PLUS_B) | ALC | ACC;

		/* Rx <- ALU_R */
		steps[1] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, INC_A, dreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_DEC --
 *
 * Generate the DEC instruction.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 0   0   0 |  dst  |
 * +-------------------------------+
 */
static void
gen_DEC(void)
{
	unsigned int dreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating DEC...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_R <- Rx - 1, CC <- ALU */
		steps[0] = _ALW | _E(dreg) | ALS(A_MINUS_B) | ACC;

		/* Rx <- ALU_R */
		steps[1] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(true, INC_A, dreg), steps, NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_ADD_ADC --
 *
 * Generate the ADD and ADC instructions.
 *
 * Encoding:
 * ADD
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 0   1   1 |  dst  |
 * +-------------------------------+
 *
 * ADC
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 0   1   1 |  dst  |
 * +-------------------------------+
 */
static void
gen_ADD_ADC(void)
{
	unsigned int dreg;
	signal_word true_steps[3];
	signal_word false_steps[3];
	unsigned int prev = total_insns;

	printf("Generating ADD/ADC...");

	/*
	 * ADC when carry is false is the same as ADD.
	 */
	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rb */
		true_steps[0] = false_steps[0] =
		    _ALB | _E(Rb);

		/* ALU_R <- Rx + ALU_B, CC <- ALU */
		true_steps[1] = false_steps[1] =
		    _ALW | _E(dreg) | ALS(A_PLUS_B) | ACC;
		true_steps[1] |= ALC;

		/* Rx <- ALU_R */
		true_steps[2] = false_steps[2] =
		    _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, A_PLUS_B, dreg), false_steps,
		    NSTEPS(false_steps));

		fill_carry_insn(ALU_OPCODE(true, A_PLUS_B, dreg),
		    false, false_steps, NSTEPS(false_steps));
		fill_carry_insn(ALU_OPCODE(true, A_PLUS_B, dreg),
		    true, true_steps, NSTEPS(true_steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_SUB_SBC --
 *
 * Generate the SUB and SBC instructions.
 *
 * Encoding:
 * SUB (Rx - Rb)
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 0   1   0 |  dst  |
 * +-------------------------------+
 *
 * SBC (Rx - Rb)
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 0   1   0 |  dst  |
 * +-------------------------------+
 *
 * SUB (Rb - Rx)
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 0   0   1 |  dst  |
 * +-------------------------------+
 *
 * SBC (Rb - Rx)
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 0   0   1 |  dst  |
 * +-------------------------------+
 */
static void
gen_SUB_SBC(void)
{
	unsigned int dreg;
	signal_word true_steps[3];
	signal_word false_steps[3];
	unsigned int prev = total_insns;

	printf("Generating SUB/SBC (Rx <- Rx - Rb)...");

	/*
	 * SBC when carry is true is the same as SUB (because carry
	 * is inverse-borrow in the ALU).  XXX DOUBLE-CHECK
	 */
	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rb */
		true_steps[0] = false_steps[0] =
		    _ALB | _E(Rb);

		/* ALU_R <- Rx - ALU_B, CC <- ALU */
		true_steps[1] = false_steps[1] =
		    _ALW | _E(dreg) | ALS(A_MINUS_B) | ALC | ACC;
		false_steps[1] &= ~ALC;

		/* Rx <- ALU_R */
		true_steps[2] = false_steps[2] =
		    _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, A_MINUS_B, dreg), true_steps,
		    NSTEPS(true_steps));

		fill_carry_insn(ALU_OPCODE(true, A_MINUS_B, dreg), false,
		    false_steps, NSTEPS(false_steps));
		fill_carry_insn(ALU_OPCODE(true, A_MINUS_B, dreg), true,
		    true_steps, NSTEPS(true_steps));
	}

	printf("done (%u insns).\n", total_insns - prev);

	/*
	 * As above, except B_MINUS_A.  This makes for one one
	 * bit of irregular assmbler syntax:
	 *
	 *	SUB	Rb, Ra
	 *
	 * Ra is the destination register in this case.
	 */
	prev = total_insns;

	printf("Generating SUB/SBC (Rx <- Rb - Rx)...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rb */
		true_steps[0] = false_steps[0] =
		    _ALB | _E(Rb);

		/* ALU_R <- ALU_B - Rx, CC <- ALU */
		true_steps[1] = false_steps[1] =
		    _ALW | _E(dreg) | ALS(B_MINUS_A) | ALC | ACC;
		false_steps[1] &= ~ALC;

		/* Rx <- ALU_R */
		true_steps[2] = false_steps[2] =
		    _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, B_MINUS_A, dreg), true_steps,
		    NSTEPS(true_steps));

		fill_carry_insn(ALU_OPCODE(true, B_MINUS_A, dreg), false,
		    false_steps, NSTEPS(false_steps));
		fill_carry_insn(ALU_OPCODE(true, B_MINUS_A, dreg), true,
		    true_steps, NSTEPS(true_steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_AND --
 *
 * Generate the AND instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 1   1   0 |  dst  |
 * +-------------------------------+
 */
static void
gen_AND(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating AND...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rx */
		steps[0] = _ALB | _E(dreg);

		/* ALU_R <- Rx & ALU_B, CC <- ALU */
		steps[1] = _ALW | _E(dreg) | ALS(A_AND_B) | ACC;

		/* Rx <- ALU_R */
		steps[2] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, A_AND_B, dreg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_OR --
 *
 * Generate the OR instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 1   0   1 |  dst  |
 * +-------------------------------+
 */
static void
gen_OR(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating OR...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rx */
		steps[0] = _ALB | _E(dreg);

		/* ALU_R <- Rx | ALU_B, CC <- ALU */
		steps[1] = _ALW | _E(dreg) | ALS(A_OR_B) | ACC;

		/* Rx <- ALU_R */
		steps[2] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, A_OR_B, dreg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_XOR --
 *
 * Generate the XOR instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 1   0   0 |  dst  |
 * +-------------------------------+
 */
static void
gen_XOR(void)
{
	unsigned int dreg;
	signal_word steps[3];
	unsigned int prev = total_insns;

	printf("Generating XOR...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_B <- Rx */
		steps[0] = _ALB | _E(dreg);

		/* ALU_R <- Rx ^ ALU_B, CC <- ALU */
		steps[1] = _ALW | _E(dreg) | ALS(A_XOR_B) | ACC;

		/* Rx <- ALU_R */
		steps[2] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, A_XOR_B, dreg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_NOT --
 *
 * Generate the NOT instructions.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 0 | 1   1   1 |  dst  |
 * +-------------------------------+
 */
static void
gen_NOT(void)
{
	unsigned int dreg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating NOT...");

	for (dreg = Ra; dreg <= Rd; dreg++) {
		/* ALU_R <- ~Rx, CC <- ALU */
		steps[0] = _ALW | _E(dreg) | ALS(B_MINUS_A) | ACC;

		/* Rx <- ALU_R */
		steps[1] = _W(dreg) | _ALE;

		fill_insn(ALU_OPCODE(false, NOT_A, dreg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_CMP --
 *
 * Generate the CMP instructions.
 * These instructions are subtractions that won't write back
 * to the register, and set the flags accordingly.
 *
 * Encoding:
 * CMP Rb, Rx
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 1   0   1 |  reg  |
 * +-------------------------------+
 *
 * CMP Rx, Rb
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 1   1   0 |  reg  |
 * +-------------------------------+
 */
static void
gen_CMP(void)
{
	unsigned int reg;
	signal_word steps[2];
	unsigned int prev = total_insns;

	printf("Generating CMP Rb, Rx...");

	for (reg = Ra; reg <= Rd; reg++) {
		/* ALU_B <- Rx */
		steps[0] = _ALB | _E(reg);

		/* ALU_R <- ALU_B - Rx, CC <- ALU */
		steps[1] = _ALW | _E(reg) | ALS(B_MINUS_A) | ALC | ACC;

		fill_insn(ALU_OPCODE(true, A_OR_B, reg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);

	prev = total_insns;

	printf("Generating CMP Rx, Rb...");

	for (reg = Ra; reg <= Rd; reg++) {
		/* ALU_B <- Rx */
		steps[0] = _ALB | _E(reg);

		/* ALU_R <- Rx - ALU_B, CC <- ALU */
		steps[1] = _ALW | _E(reg) | ALS(A_MINUS_B) | ALC | ACC;

		fill_insn(ALU_OPCODE(true, A_AND_B, reg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_TST --
 *
 * Generate the TST instructions.
 * These instructions are additions with zero that won't write back
 * to the register, and set the flags accordingly.
 *
 * Encoding:
 * +-------------------------------+
 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +-------------------------------+
 * | 1   1 | 1 | 1   1   1 |  reg  |
 * +-------------------------------+
 */
static void
gen_TST(void)
{
	unsigned int reg;
	signal_word steps[1];
	unsigned int prev = total_insns;

	printf("Generating TST...");

	for (reg = Ra; reg <= Rd; reg++) {
		/* ALU_R <- Rx + 0, CC <- ALU */
		steps[0] = _ALW | _E(reg) | ALS(A_PLUS_B) | ACC;

		fill_insn(ALU_OPCODE(true, NOT_A, reg), steps,
		    NSTEPS(steps));
	}

	printf("done (%u insns).\n", total_insns - prev);
}

/*
 * gen_ALU --
 *
 * Generate the ALU-class instructions.
 *
 * Remember the different encoding used by the ALU; Rb is always one
 * of the source operands and is NEVER the destination operand.  This
 * allows for a fairly regular syntax, except for the B_MINUS_A SUB/SUBC
 * encoding.
 */
static void
gen_ALU(void)
{
	gen_INC();
	gen_DEC();
	gen_ADD_ADC();
	gen_SUB_SBC();
	gen_AND();
	gen_OR();
	gen_XOR();
	gen_NOT();
	gen_CMP();
	gen_TST();
}

/*
 * gen_insns --
 *
 * Generate all of the instruction classes.
 */
static void
gen_insns(void)
{
	gen_MOV();
	gen_LD();
	gen_ST();
	gen_ALU();
}

#define	BANK_NAME_FMT	"bank-%c.bin"

static void
emit_bank(unsigned int b)
{
	uint8_t bankbuf[ROM_ENTRY_COUNT];
	char bankname[sizeof(BANK_NAME_FMT)];
	unsigned int i;
	FILE *bank;
	size_t rv;

	assert(b < 4);

	/*
	 * Bank A gets bits 0-7.
	 * Bank B gets bits 8-15.
	 * Bank C gets bits 16-23.
	 * Bank D gets bits 24-31.
	 */
	for (i = 0; i < ROM_ENTRY_COUNT; i++) {
		bankbuf[i] = (decode_rom[i] >> (b * 8)) & 0xff;
	}

	sprintf(bankname, BANK_NAME_FMT, 'A' + b);

	printf("Writing %s...", bankname);

	bank = fopen(bankname, "wb");
	assert(bank != NULL);

	rv = fwrite(bankbuf, ROM_ENTRY_COUNT, 1, bank);
	assert(rv == 1);
	(void)fclose(bank);

	printf("done.\n");
}

static void
emit_banks(void)
{
	unsigned int b;

	for (b = 0; b < 4; b++) {
		emit_bank(b);
	}
}

static void
emit_unused_slot_report(void)
{
	unsigned int carry, alu_fn, opcode, class, sreg, dreg;
	bool banner_printed = false;

	for (opcode = 0; opcode < 256; opcode++) {
		if (opcode_used(opcode)) {
			continue;
		}

		if (!banner_printed) {
			printf("Unused opcodes:\n");
			banner_printed = true;
		}

		class = (opcode >> 6) & 3;
		if (class < ALU) {
			dreg = (opcode >> 3) & 7;
			sreg =  opcode       & 7;
			printf("\t%s %s, %s\n", opcode_classes[class],
			    register_names[dreg],
			    register_names[sreg]);
		} else {
			dreg = opcode & 3;
			alu_fn = (opcode >> 2) & 7;
			carry = opcode & (1U << 5);
			printf("\t%s%s %s %s\n",
			    opcode_classes[class],
			    carry ? " C" : "",
			    alu_function_names[alu_fn],
			    register_names[dreg]);
		}
	}
}

int
main(int argc, char *argv[])
{

	decode_rom_init();
	gen_insns();
	decode_rom_fini();

	printf("Populated %u out of %u instruction slots.\n",
	    total_insns, MAX_INSNS);

	emit_banks();
	emit_unused_slot_report();

	return 0;
}
