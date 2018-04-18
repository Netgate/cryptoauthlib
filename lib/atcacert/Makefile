
OBJS=atcacert_client.o atcacert_date.o atcacert_def.o atcacert_der.o
OBJS+=atcacert_host_hw.o
INCLUDES=atcacert.h atcacert_client.h atcacert_date.h atcacert_def.h
INCLUDES+=atcacert_der.h atcacert_host_hw.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

.if defined(DEBUG)
CCFLAGS+=-g
.endif

all: libatca_cert.a

libatca_cert.a: $(OBJS) $(INCLUDES)
	$(AR) -rc libatca_cert.a $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libatca_cert.a $(OBJS)

install: all
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib/atcacert
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/atcacert
