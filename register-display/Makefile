ROMIMG=		display-rom.bin
CLEANFILES=	gendisplay gendisplay.o $(ROMIMG)
CFLAGS=		-O1 -g -Wall

$(ROMIMG): gendisplay
	-./gendisplay

gendisplay: gendisplay.o
	cc -o gendisplay gendisplay.o

gendisplay.o: gendisplay.c

clean:
	-rm -f $(CLEANFILES)
