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
 * gendisplay --
 *
 * Generates the character ROM for the 4-digit 7-segment displays on
 * the Register Module.
 */

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define	BIT(x)		((uint8_t)1 << (x))

/*
 * We're using the Wurth WL-T7DS common-cathode character cells,
 * which have the following segment arrangement:
 *
 *        D
 *     +-----+
 *   C |     | E
 *     |  G  |
 *     +-----+
 *   B |     | A
 *     |  F  |
 *     +-----+    DP
 *
 * Those segments are connected to the character ROM outputs like so:
 *
 * bit 0	A
 * bit 1	B
 * bit 2	C
 * bit 3	D
 * bit 4	E
 * bit 5	F
 * bit 6	G
 * bit 7	DP
 */

#define	A	BIT(0)
#define	B	BIT(1)
#define	C	BIT(2)
#define	D	BIT(3)
#define	E	BIT(4)
#define	F	BIT(5)
#define	G	BIT(6)
#define	DP	BIT(7)

static const char character_data[10] = {
/*
 *        D
 *     +-----+
 *   C |     | E
 *     |  G  |
 *     +     +
 *   B |     | A
 *     |  F  |
 *     +-----+    DP
 */
	[0] = A | B | C | D | E | F,

/*
 *        D
 *           +
 *   C       | E
 *        G  |
 *           +
 *   B       | A
 *        F  |
 *           +    DP
 */
	[1] = A | E,

/*
 *        D
 *     +-----+
 *   C       | E
 *        G  |
 *     +-----+
 *   B |       A
 *     |  F   
 *     +-----+    DP
 */
	[2] = B | D | E | F | G,

/*
 *        D
 *     +-----+
 *   C       | E
 *        G  |
 *     +-----+
 *   B       | A
 *        F  |
 *     +-----+    DP
 */
	[3] = A | D | E | F | G,

/*
 *        D
 *     +     +
 *   C |     | E
 *     |  G  |
 *     +-----+
 *   B       | A
 *        F  |
 *           +    DP
 */
	[4] = A | C | E | G,

/*
 *        D
 *     +-----+
 *   C |       E
 *     |  G   
 *     +-----+
 *   B       | A
 *        F  |
 *     +-----+    DP
 */
	[5] = A | C | D | F | G,

/*
 *        D
 *     +      
 *   C |       E
 *     |  G   
 *     +-----+
 *   B |     | A
 *     |  F  |
 *     +-----+    DP
 */
	[6] = A | B | C | F | G,

/*
 *        D
 *     +-----+
 *   C       | E
 *        G  |
 *           +
 *   B       | A
 *        F  |
 *           +    DP
 */
	[7] = A | D | E,

/*
 *        D
 *     +-----+
 *   C |     | E
 *     |  G  |
 *     +-----+
 *   B |     | A
 *     |  F  |
 *     +-----+    DP
 */
	[8] = A | B | C | D | E | F | G,

/*
 *        D
 *     +-----+
 *   C |     | E
 *     |  G  |
 *     +-----+
 *   B       | A
 *        F  |
 *           +    DP
 */
	[9] = A | C | D | E | G,
};

#define	MINUS		G
#define	BLANK		0

#define	ROM_ADDR(v, d, s)						\
	((v) | ((unsigned int)(d) << 8) | ((s) ? (1U << 10) : 0))

#define	ROM_SIZE	(1U << 11)

static uint8_t character_rom[ROM_SIZE];

static int
cell2char(uint8_t cell)
{
	int i;

	for (i = 0; i < 10; i++) {
		if (cell == character_data[i]) {
			return '0' + i;
		}
	}
	if (cell == MINUS) {
		return '-';
	}
	assert(cell == BLANK);
	return ' ';
}

static void
fill_unsigned(void)
{
	unsigned int i;
	uint8_t ones, tens, hundreds;
	uint8_t cell[4];

	printf("Generating unsigned numbers...\n");

	for (i = 0; i <= UINT8_MAX; i++) {
		ones = i % 10;
		tens = (i / 10) % 10;
		hundreds = (i / 100) % 10;

		cell[0] = character_data[ones];
		cell[1] = i >= 10 ? character_data[tens] : BLANK;
		cell[2] = i >= 100 ? character_data[hundreds] : BLANK;
		cell[3] = BLANK;

		printf("%3u: '%c%c%c%c'\n", i,
		    cell2char(cell[3]),
		    cell2char(cell[2]),
		    cell2char(cell[1]),
		    cell2char(cell[0]));

		character_rom[ROM_ADDR(i, 0, false)] = cell[0];
		character_rom[ROM_ADDR(i, 1, false)] = cell[1];
		character_rom[ROM_ADDR(i, 2, false)] = cell[2];
		character_rom[ROM_ADDR(i, 3, false)] = cell[3];
	}

	printf("Done!\n");
}

static void
fill_signed(void)
{
	unsigned int i, a;
	uint8_t ones, tens, hundreds;
	int8_t v;
	uint8_t cell[4];

	printf("Generating signed numbers...\n");

	for (i = 0; i <= UINT8_MAX; i++) {
		v = (int8_t)i;
		a = abs(v);
		ones = a % 10;
		tens = (a / 10) % 10;
		hundreds = (a / 100) % 10;

		cell[0] = character_data[ones];
		if (v >= 10 || v <= -10) {
			cell[1] = character_data[tens];
		} else {
			cell[1] = v < 0 ? MINUS : BLANK;
		}
		if (v >= 100 || v <= -100) {
			cell[2] = character_data[hundreds];
			cell[3] = v <= -100 ? MINUS : BLANK;
		} else {
			cell[2] = v <= -10 ? MINUS : BLANK;
			cell[3] = BLANK;
		}

		printf("%3u: '%c%c%c%c'\n", i,
		    cell2char(cell[3]),
		    cell2char(cell[2]),
		    cell2char(cell[1]),
		    cell2char(cell[0]));

		character_rom[ROM_ADDR(i, 0, true)] = cell[0];
		character_rom[ROM_ADDR(i, 1, true)] = cell[1];
		character_rom[ROM_ADDR(i, 2, true)] = cell[2];
		character_rom[ROM_ADDR(i, 3, true)] = cell[3];
	}

	printf("Done!\n");
}

#define	ROM_NAME	"display-rom.bin"

static void
emit_rom(void)
{
	FILE *rom;
	size_t rv;

	printf("Writing %s...", ROM_NAME);

	rom = fopen(ROM_NAME, "wb");
	assert(rom != NULL);

	rv = fwrite(character_rom, ROM_SIZE, 1, rom);
	assert(rv == 1);
	(void)fclose(rom);

	printf("done.\n");
}

int
main(int argc, char *argv[])
{

	fill_unsigned();
	fill_signed();
	emit_rom();

	return 0;
}
