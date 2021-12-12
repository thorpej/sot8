OBJECTS=	y.tab.o lex.yy.o
GENSRCS=	lex.yy.c y.tab.c y.tab.h
DEPHDRS=	asdefs.h ../microcode/sot8defs.h
CLEANFILES=	$(PRODUCT) $(OBJECTS) $(GENSRCS)
CFLAGS=		-O2 -g # -Wall
PRODUCT=	sot8as

$(PRODUCT): $(OBJECTS)
	cc -o $(PRODUCT) $(OBJECTS)

clean:
	-rm -f $(CLEANFILES)

y.tab.o: $(DEPHDRS) y.tab.c y.tab.h

lex.yy.o: $(DEPHDRS) lex.yy.c

lex.yy.c: scan.l
	lex scan.l

y.tab.c y.tab.h: gram.y
	yacc -d -t gram.y
