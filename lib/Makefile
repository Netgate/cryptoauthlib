
OBJS=atca_cfgs.o atca_command.o atca_device.o atca_execution.o atca_iface.o
INCLUDES=atca_bool.h atca_cfgs.h atca_command.h atca_compiler.h
INCLUDES+=atca_device.h atca_devtypes.h atca_execution.h atca_iface.h
INCLUDES+=atca_status.h cryptoauthlib.h


LDFLAGS=-L./basic -L./crypto -L./hal -L./host -whole-archive -static
LDFLAGS+=-latca_basic -latca_crypto -latca_hal -latca_host --no-whole-archive
CCFLAGS=-O2 -fPIC -g -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I.

all: libcryptoauth.so

libcryptoauth.so: basic crypto hal host $(OBJS) $(INCLUDES)
	$(LD) -shared -o libcryptoauth.so $(OBJS) $(LDFLAGS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libcryptoauth.so $(OBJS)

install: all
	install -m 0644 libcryptoauth.so $(PREFIX)/usr/local/lib
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/
	@make -C basic install
	@make -C crypto install
	@make -C hal install
	@make -C host install

SUBDIR=		basic
SUBDIR+=	crypto
SUBDIR+=	hal
SUBDIR+=	host

.include <bsd.subdir.mk>
