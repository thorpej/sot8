# sot8
My 8-bit "System on a Table" computer built from TTL logic chips.

This work is derived from the "JCPU" by James Bates, who did an excellent
[YouTube series](https://www.youtube.com/playlist?list=PL_i7PfWMNYobSPpg1_voiDe6qBcjvuVui)
on his work.  You can find his design files and other materials
[here on GitHub](https://github.com/jamesbates/jcpu).

Bates' work, in turn, was inspired by the [Ben Eater breadboard computer](https://eater.net/8bit).

Like Eater, Bates built his system on breadboards.  The SoT8 will instead
be built on a single PCB.

This is all still a work-in-progress, so keep checking back in for updates.

## Improvements on the Bates design

The SoT8 has several improvements over the Bates design, described here:

### Decode ROM organization

The Bates CPU places the opcode in bits 0-7 of the decode ROM address,
and the microcode step in bits 8-10.  The SoT8 rearranges this to place
the microcode step in bits 0-2 and the opcode in bits 3-10.  This places
all of the micrcode steps for a given instruction sequentially in the
decode ROM, which simplifies microcode generation and seems like a more
natural way to organize the decode ROM.

### Address space identifier

The Bates CPU has a single `PGM` signal to select program memory, and if
that signal is not asserted, then memory accesses hit data memory.  I wanted
the SoT8 to be able to support additional address spaces (specifically,
an I/O space), so I replaced the single `PGM` signal with a pair of signals,
`AS0` and `AS1`.  These signals allow us to select one of 4 address spaces:

* D-space (data memory)
* I-space (program memory)
* S-space (a stack region)
* I/O-space (for external peripherals)

These signals are decoded by the Memory Module.

### Immediate Value register

In order to support an IMMEDIATE addressing mode for more instructions
(e.g. _call_), I have added an Immediate Value (IV) register to the Special
Registers Module.  When loading an immediate value from the instruction
stream, the value is always stored in the IV register, in addition to being
stored in another destination register as needed.

This adds `_IVE` and `IVW` signals.

### Changes to the _call_ instruction

The Bates CPU implicitly uses _Rc_ to hold the target of the _call_
instruction.  The reason for this is because his _call_ instruction is
encoded as "`PUSH PC`", which is actually "`ST [SPa], PC`", which consumes
both register slots in the opcode.  This is inefficient because the
typical use of _call_ is to call a function at a label.  A label, of course,
is simply an immediate value, which means that the typical usage requires
2 instructions:

```
	mov	%r2, $some_label
	call	%r2
```

What I've done is removed the magic from "`PUSH PC`".  This allows the
atypical case to still be used in an open-coded manner:

```
	push	%pc
	mov	%pc, %r2
```

...but allows typical usage to be encoded in 2 bytes instead of 3,
which is important when you only have 256 bytes of program space!

Our syntax is:

```
	call	$some_label
```

which uses the encoding "`MOV SPa, PC`" plus an immediate value.  This
does what you might expect: pushes the PC onto the stack and then moves
the immediate value into the program counter.  This instruction uses the
IV register to hold the _call_ destination while the _push_ portion of
the instruction is being executed.

The _call_ instruction is the slowest on the SoT8, using all 6 available
microcode steps, but it's faster than the pattern enforced by the original
Bates design.

### Changes to the ALU

The Bates CPU hard-codes the B-operand of the ALU to either 0 or _Rb_.
This is largely because the opcode doesn't have room for both the ALU
function **and** 2 registers.  However, Bates baked this into the hardware;
_Rb_ has a back-door input intot he ALU.  I don't like this approach for
a couple of reasons:

1. It makes the _Rb_ register different from all of the other General Purpose
Registers, and I'd really rather that not be the case.
2. It limits our ability to use the ALU for some other things (see below).

So, instead of the selector bank that selects 0 or _Rb_, I have added an
_ALU\_B_ register that is cleared at the end of every instruction (by
snooping for the assertion of `_ucSR`).  This provides the hard-coded 0
for instructions that require it.  Then, instead of the `ALB` signal
selecting _Rb_ directly, we burn a microcode cycle copying _Rb_ into the
_ALU\_B_ register (using the `ALB` signal to enable writing to _ALU\_B_).
This is slower than the Bates design, but has the advantage of making
_Rb_ magical only in the microcode, and unlocks some other uses of the ALU
that we'll see below.

Side note: because this means that the ALU now has 2 internal registers
(B-operand and the result register), I have renamed the result register
to _ALU\_R_.

### Changes to the condition codes

The condition codes work somewhat differently on the SoT8 compared to
the Bates design.  The CC register is not located in the ALU in this
design.  Instead, the CC register resides in the Control Unit and can be
updated from two different places: the ALU (by asserting the `ACC` signal)
or the processor bus (by asserting the `BCC` signal).  This allows us to
do a couple of useful things:

1. Selectively update the flags from the ALU, in case we're using the ALU
to perform some implicit operation (like computing an effective address).
2. Update the Z and N flags during a "`LD`" or "`MOV`" into one of the
general purpose regsiters.

The latter allows us to reduce the following pattern:

```
	ld	%r0, #1[%sp]
	tst	%r0
	jz	$somewhere
```

to this:

```
	ld	%r0, #1[%sp]
	jz	$somewhere
```

### SP-relative loads/stores

Because this design has so much stack space and a more useful _call_
instruction, it seemed like a good idea to be able to pass function
arguments on the stack.  But accessing them in the callee is really
tough without the ability to load from an offset relative to the stack
pointer.  Happily, the _ALU\_B_ register makes it really easy for us
to this; all we need to do is fine a pair of opcods that we can hijack
for it.  Limiting these to the GPRs seems pretty reasonable, so these
are encoded as "`LD SPa, Rx`" and "`ST Rx, SPa`", followed by an immediate
value to use as the offset.  Note that in this encoding, the source and
destination registers are swapped; that's the price we pay for only having
8 bits of opcode!

### PUSH #IMM instruction

Since we can use the stack to pass arguments around, it is useful to
be able to push an immediate value onto the stack directly:

```
	push	#0xff
```

..rather than having to do:

```
	mov	%r2, #0xff
	pusH	%r2
```

### SPA (Stack Pointer Adjust) instruction

Now that we've pushed all those arguments onto the stack, we would like
to be able to pop them off without having to clobber a GPR or burn precious
program space.  We encode this as "`MOV SPa, IMM`" followed by an immediate
value.

```
	spa	#3	; pop the 3 arguments
```

### I-space loads (and stores!)

Loads from I-space are useful for e.g. loading tables in from ROM.
The Bates CPU did this by treating Rc as a magic register for loads
and stores.  I wasn't willing to do that without a different opcode.
We hijack the "`LD Rx, PC`" and "`ST PC, Rx`" encodings to do this, and
use _Ra_ as the enforced address register.

### INB / OUTB instructions for I/O space access

To support I/O space, INB and OUTB instructions are added to
the LD and ST classes.  These instructions all use an 8-bit
immediate address.  These instructions only allow I/O using
the general purpose registers, thus saving us a few opcodes
for future use.
