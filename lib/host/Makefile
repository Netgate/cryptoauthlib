
OBJS=atca_host.o
INCLUDES=atca_host.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

.if defined(DEBUG)
CCFLAGS+=-g
.endif

all: libatca_host.a

libatca_host.a: $(OBJS) $(INCLUDES)
	$(AR) -rc libatca_host.a $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libatca_host.a $(OBJS)

install: all
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib/host
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/host
