# SoT8 Register Module

The Register Module contains the 4 general purpose registers (alternately
named _Ra_ - _Rd_ and _%r0_ - _%r3_).  In addition to the 4 general purpose
registers, the Register Module contains both a binary LED display and
a 4-digit 7-segment display for each register.

The registers themselves are fairly simple.  Each one consists of a
74HCT377 and a 74HCT245.  The inputs to the 74HCT377 are connected to
the processor bus, and the active-low write-enable (e.g. `_RaW`) signal
is connected to the _CLKEN pin.  The outputs are connected to the 74HCT245,
which is in-turn connected to the bus.  The 74HCT245's outputs are enabled
with the active-low reigster-enable (e.g. `_RaE`) signal.

The LED displays are connected to the lines between the 74HCT377 and
the 74HCT245, allowing us to view the current contents of each register.
The bit signals drive the binary display LEDs directly, but the 4-digit
7-segment displays work a bit differently.

The outputs of the registers are connected to a 28C64 EEPROM, which
contains the character data for the 7-segment displays, in addition
to a 2-bit digit selector and a 1-bit "negative number enable" selector.
Astute readers will notice that a 28C16 would have been sufficient;
however, the 28C16 is out-of-production and increasingly hard to
source, so I went with the more readily available 28C64.

The 28C64 is multiplexed across all 4 digits using a 74HCT161 4-bit
counter, driven by a 555 timer pulsing the counter at 718 Hz.  The
4-bit counter provides the 2-bit digit selector to all 4 displays.
Each pair of registers has a 74HCT139 dual 4-bit decoder that drives
the common-cathode 7-segment displays.

With a little more combinational logic, the character ROMs could have
been shared by all 4 register displays, but the 4-EEPROM apporach was
a lot less complicated.
