
OBJS=atca_basic.o atca_basic_aes.o atca_basic_checkmac.o atca_basic_counter.o
OBJS+=atca_basic_derivekey.o atca_basic_ecdh.o atca_basic_gendig.o
OBJS+=atca_basic_genkey.o atca_basic_hmac.o atca_basic_info.o atca_basic_kdf.o
OBJS+=atca_basic_lock.o atca_basic_mac.o atca_basic_nonce.o
OBJS+=atca_basic_privwrite.o atca_basic_random.o atca_basic_read.o
OBJS+=atca_basic_secureboot.o atca_basic_selftest.o atca_basic_sha.o
OBJS+=atca_basic_sign.o atca_basic_updateextra.o atca_basic_verify.o
OBJS+=atca_basic_write.o atca_helpers.o
INCLUDES=atca_basic.h atca_helpers.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -g -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

.if defined(ATCA_HAL_I2C)
CCFLAGS+=-DATCA_HAL_I2C
.endif

all: libatca_basic.a

libatca_basic.a: $(OBJS) $(INCLUDES)
	$(AR) -rc libatca_basic.a $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libatca_basic.a $(OBJS)

install: all
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib/basic
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/basic
