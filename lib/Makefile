
OBJS=atca_cfgs.o atca_command.o atca_device.o atca_execution.o atca_iface.o
INCLUDES=atca_bool.h atca_cfgs.h atca_command.h atca_compiler.h
INCLUDES+=atca_device.h atca_devtypes.h atca_execution.h atca_iface.h
INCLUDES+=atca_status.h cryptoauthlib.h


LDFLAGS=-L./atcacert -L./basic -L./crypto -L./hal -L./host -whole-archive
LDFLAGS+=-static -latca_cert -latca_basic -latca_crypto -latca_hal -latca_host
LDFLAGS+=--no-whole-archive -lcompiler_rt
CCFLAGS=-O2 -fPIC -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I.

.if defined(DEBUG)
CCFLAGS+=-g
.endif

all: libcryptoauth.so

libcryptoauth.so: atcacert basic crypto hal host $(OBJS) $(INCLUDES)
	$(LD) -Bshareable --hash-style=both --enable-new-dtag \
	    -o libcryptoauth.so $(OBJS) $(LDFLAGS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libcryptoauth.so $(OBJS)

install: all
	install -m 0644 libcryptoauth.so $(PREFIX)/usr/local/lib
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/
	@make -C atcacert install
	@make -C basic install
	@make -C crypto install
	@make -C hal install
	@make -C host install

SUBDIR=		atcacert
SUBDIR+=	basic
SUBDIR+=	crypto
SUBDIR+=	hal
SUBDIR+=	host

.include <bsd.subdir.mk>
