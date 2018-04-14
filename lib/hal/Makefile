
OBJS=atca_hal.o hal_linux_timer.o
INCLUDES=atca_hal.h atca_start_config.h atca_start_iface.h

LDFLAGS=
CCFLAGS=-O2 -fPIC -g -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I../

.if defined(__FreeBSD__) && defined(ATCA_HAL_I2C)
CCFLAGS+=-DATCA_HAL_I2C -DMAX_I2C_BUSES=4
OBJS+=hal_freebsd_i2c.o
INCLUDES+=hal_freebsd_i2c.h
.endif

all: libatca_hal.a

libatca_hal.a: $(OBJS) $(INCLUDES)
	$(AR) -rc libatca_hal.a $(OBJS)

.c.o: $(INCLUDES)
	$(CC) $(CCFLAGS) -c $<

clean:
	rm -f libatca_hal.a $(OBJS)

install: all
	install -dm 0755 $(PREFIX)/usr/local/include/cryptoauthlib/hal
	install -m 0644 $(INCLUDES) $(PREFIX)/usr/local/include/cryptoauthlib/hal
