
OBJS=atca_crypto_sw_ecdsa.o atca_crypto_sw_rand.o atca_crypto_sw_sha1.o
OBJS+=atca_crypto_sw_sha2.o
INCLUDES=atca_crypto_sw.h atca_crypto_sw_ecdsa.h atca_crypto_sw_rand.h
INCLUDES+=atca_crypto_sw_sha1.h atca_crypto_sw_sha2.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

.if defined(DEBUG)
CCFLAGS+=-g
.endif

all: libatca_crypto.a

libatca_crypto.a: hashes $(OBJS) $(INCLUDES)
	$(AR) -rc libatca_crypto.a hashes/sha1_routines.o hashes/sha2_routines.o $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libatca_crypto.a $(OBJS)

install: all
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib/crypto
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/crypto

SUBDIR=         hashes

.include <bsd.subdir.mk>
