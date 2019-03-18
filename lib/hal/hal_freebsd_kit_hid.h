/**
 * \file
 * \brief ATCA Hardware abstraction layer for FreeBSD using kit protocol over a USB HID device.
 *
 * \copyright (c) 2018-2019, Rubicon Communications, LLC (Netgate).
 *
 */

#ifndef HAL_FREEBSD_KIT_HID_H_
#define HAL_FREEBSD_KIT_HID_H_

/** \defgroup hal_ Hardware abstraction layer (hal_)
 *
 * \brief
 * These methods define the hardware abstraction layer for communicating with a CryptoAuth device
 *
   @{ */

// Kit USB defines
#define HID_DEVICES_MAX     10      //! Maximum number of supported Kit USB devices
#define HID_PACKET_MAX      256     //! Maximum number of bytes for a HID send/receive packet (typically 64)
#define	HID_PACKET_SIZE     64      //! Default packet size
#define HID_VENDORID        0x03eb
#define HID_PRODUCTID       0x2312

// Each device that is found will have a read handle and a write handle
typedef struct hid_device
{
    int fd;
    int psize;
    char *ibuf;
    char dev[16];
} hid_device_t;

// A structure to hold HID information
typedef struct atcahid
{
    hid_device_t kits[HID_DEVICES_MAX];
    int8_t       num_kits_found;
} atcahid_t;

/** @} */
#endif /* HAL_FREEBSD_KIT_HID_H_ */
