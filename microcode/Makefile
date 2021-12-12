UCODE=		bank-A.bin bank-B.bin bank-C.bin bank-D.bin
CLEANFILES=	genucode genucode.o $(UCODE)
CFLAGS=		-O1 -g -Wall

$(UCODE): genucode
	-./genucode

genucode: genucode.o
	cc -o genucode genucode.o

genucode.o: genucode.c sot8defs.h

clean:
	-rm -f $(CLEANFILES)
