PREFIX?=	/usr/local

OBJS=atca_cfgs.o
OBJS+=atca_command.o
OBJS+=atca_device.o
OBJS+=atca_execution.o
OBJS+=atca_iface.o
OBJS+=atcacert/atcacert_client.o
OBJS+=atcacert/atcacert_date.o
OBJS+=atcacert/atcacert_def.o
OBJS+=atcacert/atcacert_der.o
OBJS+=atcacert/atcacert_host_hw.o
OBJS+=basic/atca_basic.o
OBJS+=basic/atca_basic_aes.o
OBJS+=basic/atca_basic_checkmac.o
OBJS+=basic/atca_basic_counter.o
OBJS+=basic/atca_basic_derivekey.o
OBJS+=basic/atca_basic_ecdh.o
OBJS+=basic/atca_basic_gendig.o
OBJS+=basic/atca_basic_genkey.o
OBJS+=basic/atca_basic_hmac.o
OBJS+=basic/atca_basic_info.o
OBJS+=basic/atca_basic_kdf.o
OBJS+=basic/atca_basic_lock.o
OBJS+=basic/atca_basic_mac.o
OBJS+=basic/atca_basic_nonce.o
OBJS+=basic/atca_basic_privwrite.o
OBJS+=basic/atca_basic_random.o
OBJS+=basic/atca_basic_read.o
OBJS+=basic/atca_basic_secureboot.o
OBJS+=basic/atca_basic_selftest.o
OBJS+=basic/atca_basic_sha.o
OBJS+=basic/atca_basic_sign.o
OBJS+=basic/atca_basic_updateextra.o
OBJS+=basic/atca_basic_verify.o
OBJS+=basic/atca_basic_write.o
OBJS+=basic/atca_helpers.o
OBJS+=crypto/atca_crypto_sw_ecdsa.o
OBJS+=crypto/atca_crypto_sw_rand.o
OBJS+=crypto/atca_crypto_sw_sha1.o
OBJS+=crypto/atca_crypto_sw_sha2.o
OBJS+=crypto/hashes/sha1_routines.o
OBJS+=crypto/hashes/sha2_routines.o
OBJS+=hal/atca_hal.o
OBJS+=hal/hal_linux_os.o
OBJS+=hal/hal_linux_timer.o
OBJS+=host/atca_host.o



INCLUDES=atca_bool.h
INCLUDES+=atca_cfgs.h
INCLUDES+=atca_command.h
INCLUDES+=atca_compiler.h
INCLUDES+=atca_device.h
INCLUDES+=atca_devtypes.h
INCLUDES+=atca_execution.h
INCLUDES+=atca_iface.h
INCLUDES+=atca_status.h
INCLUDES+=cryptoauthlib.h
ATCACERT_INCLUDES+=atcacert/atcacert.h
ATCACERT_INCLUDES+=atcacert/atcacert_client.h
ATCACERT_INCLUDES+=atcacert/atcacert_date.h
ATCACERT_INCLUDES+=atcacert/atcacert_def.h
ATCACERT_INCLUDES+=atcacert/atcacert_der.h
ATCACERT_INCLUDES+=atcacert/atcacert_host_hw.h
BASIC_INCLUDES+=basic/atca_basic.h
BASIC_INCLUDES+=basic/atca_helpers.h
CRYPTO_INCLUDES+=crypto/atca_crypto_sw.h
CRYPTO_INCLUDES+=crypto/atca_crypto_sw_ecdsa.h
CRYPTO_INCLUDES+=crypto/atca_crypto_sw_rand.h
CRYPTO_INCLUDES+=crypto/atca_crypto_sw_sha1.h
CRYPTO_INCLUDES+=crypto/atca_crypto_sw_sha2.h
HAL_INCLUDES+=hal/atca_hal.h
HAL_INCLUDES+=hal/atca_start_config.h
HAL_INCLUDES+=hal/atca_start_iface.h
HOST_INCLUDES+=host/atca_host.h


LDFLAGS=
CCFLAGS=-O2 -fPIC -Wall -Wextra -Wshadow -Wcast-qual -Wcast-align
CCFLAGS+=-Wwrite-strings -Wredundant-decls -Wnested-externs -Winline
CCFLAGS+=-I.

.if defined(DEBUG)
CCFLAGS+=-g
.endif

.if defined(ATCA_PRINTF) || defined(ATCAPRINTF)
CCFLAGS+=-DATCAPRINTF
.endif

.if defined(__FreeBSD__) && defined(ATCA_HAL_I2C)
CCFLAGS+=-DATCA_HAL_I2C -DMAX_I2C_BUSES=4
OBJS+=hal/hal_freebsd_i2c.o
HAL_INCLUDES+=hal/hal_freebsd_i2c.h
.else
.if defined(ATCA_HAL_I2C)
CCFLAGS+=-DATCA_HAL_I2C
.endif
.endif


ALL_INCLUDES=$(INCLUDES) $(ATCACERT_INCLUDES) $(BASIC_INCLUDES)
ALL_INCLUDES+=$(CRYPTO_INCLUDES) $(HAL_INCLUDES) $(HOST_INCLUDES)

all: libcryptoauth.so libcryptoauth.a

libcryptoauth.so: $(OBJS) $(ALL_INCLUDES)
	$(CC) -shared -o libcryptoauth.so $(OBJS) $(LDFLAGS)

libcryptoauth.a: $(OBJS) $(ALL_INCLUDES)
	$(AR) rcs libcryptoauth.a $(OBJS)

.c.o: $(ALL_INCLUDES)
	$(CC) $(CCFLAGS) -o $@ -c $<

clean:
	rm -f libcryptoauth.so libcryptoauth.a $(OBJS)

install: all
	install -m 0644 libcryptoauth.a $(DESTDIR)$(PREFIX)/lib
	install -m 0644 -s libcryptoauth.so $(DESTDIR)$(PREFIX)/lib
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib
	install -m 0644 $(INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib/atcacert
	install -m 0644 $(ATCACERT_INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/atcacert/
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib/basic
	install -m 0644 $(BASIC_INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/basic/
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib/crypto
	install -m 0644 $(CRYPTO_INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/crypto/
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib/hal
	install -m 0644 $(HAL_INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/hal/
	install -dm 0755 $(DESTDIR)$(PREFIX)/include/cryptoauthlib/host
	install -m 0644 $(HOST_INCLUDES) $(DESTDIR)$(PREFIX)/include/cryptoauthlib/host/
