
OBJS=sha1_routines.o sha2_routines.o
INCLUDES=sha1_routines.h sha2_routines.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -g -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

all: $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f $(OBJS)
