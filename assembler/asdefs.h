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

#include <sys/types.h>
#include <sys/queue.h>
#include <stdbool.h>
#include <stdint.h>

struct label {
	LIST_ENTRY(label)	link;
	char *			ident;
	long			value;
	int			lineno;
	bool			resolved;
};
LIST_HEAD(label_head, label);

typedef enum {
	OPR_REG		= 0,
	OPR_IMM		= 1,
	OPR_LABEL	= 2,
	OPR_ALU		= 3,
	OPR_CC		= 4,
} operand_type;

struct operand {
	operand_type		type;
	union {
		unsigned int	regnum;
		struct label *	label;
		long		immval;
		struct {
			unsigned int fn;
			bool	carry;
		} alu;
		unsigned int	ccode;
	};
};

struct insn {
	unsigned int		opc;
	int			lineno;
	struct operand *	operands[3];
};

typedef enum {
	PGN_LABEL	= 0,
	PGN_INSN	= 1,
} node_type;

struct program_node {
	TAILQ_ENTRY(program_node) link;
	node_type		type;
	union {
		void *		object;
		struct label *	label;
		struct insn *	insn;
	};
};
TAILQ_HEAD(program_node_head, program_node);
