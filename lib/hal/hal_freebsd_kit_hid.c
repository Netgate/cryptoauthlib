/**
 * \file
 * \brief ATCA Hardware abstraction layer for FreeBSD using kit protocol over a USB HID device.
 *
 * \copyright (c) 2018 Rubicon Communications, LLC (Netgate).
 *
 */

#include <sys/param.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libusb.h>

#include "atca_hal.h"
#include "hal_freebsd_kit_hid.h"
#include "hal/kit_protocol.h"

/** \defgroup hal_ Hardware abstraction layer (hal_)
 *
 * \brief
 * These methods define the hardware abstraction layer for communicating with a CryptoAuth device
 *
   @{ */

// File scope globals
atcahid_t _gHid;

/** \brief discover hid buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge.This function is currently not implemented.
 * \param[in] cdc_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 * \return ATCA_UNIMPLEMENTED
 */

ATCA_STATUS hal_kit_hid_discover_buses(int hid_buses[] __unused,
    int max_buses __unused)
{
    return ATCA_UNIMPLEMENTED;
}

/** \brief discover any CryptoAuth devices on a given logical bus number.This function is currently not implemented.
 * \param[in] busNum - logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg[] - pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] *found - number of devices found on this bus
 * \return ATCA_UNIMPLEMENTED
 */
ATCA_STATUS hal_kit_hid_discover_devices(int busNum __unused,
    ATCAIfaceCfg cfg[] __unused, int *found __unused)
{
    return ATCA_UNIMPLEMENTED;
}

/** \brief HAL implementation of Kit USB HID init
 *  \param[in] hal pointer to HAL specific data that is maintained by this HAL
 *  \param[in] cfg pointer to HAL specific configuration data that is used to initialize this HAL
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_init(void* hal, ATCAIfaceCfg* cfg)
{
    ATCAHAL_t *phal;
    int err, i, index, fd, len;
    libusb_device *dev, **devs;
    libusb_device_handle *devh;

    phal = (ATCAHAL_t*)hal;

    // Check the input variables
    if ((cfg == NULL) || (phal == NULL))
    {
        return ATCA_BAD_PARAM;
    }

    // Initialize the _gHid structure
    memset(&_gHid, 0, sizeof(_gHid));
    for (i = 0; i < HID_DEVICES_MAX; i++)
    {
        _gHid.kits[i].fd = -1;
        _gHid.kits[i].psize = 0;
        memset(_gHid.kits[i].dev, 0, sizeof(_gHid.kits[i].dev));
    }

    _gHid.num_kits_found = 0;

    if (libusb_init(NULL) < 0)
    {
        return ATCA_COMM_FAIL;
    }

    if (libusb_get_device_list(NULL, &devs) < 0)
    {
        libusb_exit(NULL);
        return ATCA_COMM_FAIL;
    }

    i = 0;
    index = 0;
    while ((dev = devs[i++]) != NULL)
    {
        struct libusb_device_descriptor desc;
        char devp[64], driver[16];

        err = libusb_get_device_descriptor(dev, &desc);
        if (err < 0)
            break;
        if (desc.idVendor != HID_VENDORID || desc.idProduct != HID_PRODUCTID)
            continue;
        if (desc.bMaxPacketSize0 < HID_PACKET_SIZE ||
            desc.bMaxPacketSize0 > HID_PACKET_MAX - 1)
            continue;

        err = libusb_open(dev, &devh);
        if (err < 0)
            break;
        err = libusb_get_driver(devh, 0, driver, sizeof(driver));
        if (err < 0)
            break;
        libusb_close(devh);
        len = snprintf(devp, sizeof(devp), "/dev/%s", driver);
        if (len < 0 || len >= (int)sizeof(devp))
            break;
        fd = open(devp, O_RDWR | O_CLOEXEC);
	if ((err = fd) == -1)
            break;

        _gHid.kits[index].fd = fd;
        _gHid.kits[index].psize = desc.bMaxPacketSize0;
        _gHid.kits[index].ibuf = calloc(1, _gHid.kits[index].psize);
	if (_gHid.kits[index].ibuf == NULL)
            exit(51);	/* should not happen. */
        memcpy(_gHid.kits[index].dev, dev, sizeof(_gHid.kits[index].dev));
	index++;
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(NULL);

    if (err < 0)
        return ATCA_COMM_FAIL;

    // Save the results of this discovery of HID
    if (index > 0)
    {
        _gHid.num_kits_found = index;
        phal->hal_data = &_gHid;
    }

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of Kit HID post init
 *  \param[in] iface  instance
 *  \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_post_init(ATCAIface iface)
{
    ATCA_STATUS status = ATCA_SUCCESS;
    atcahid_t* pHid = atgetifacehaldat(iface);
    ATCAIfaceCfg *pCfg = atgetifacecfg(iface);
    int i = 0;

    if ((pHid == NULL) || (pCfg == NULL))
    {
        return ATCA_BAD_PARAM;
    }

    // Perform the kit protocol init
    for (i = 0; i < pHid->num_kits_found; i++)
    {
        status = kit_init(iface);
        if (status != ATCA_SUCCESS)
        {
            BREAK(status, "kit_init() Failed");
        }
    }

    return status;
}

/** \brief HAL implementation of send over Kit protocol.This function is called by the top layer.
 *  \param[in] iface     instance
 *  \param[in] txdata    pointer to bytes to send
 *  \param[in] txlength  number of bytes to send
 *  \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS kit_phy_send(ATCAIface iface, uint8_t* txdata, int txlength)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    atcahid_t* pHid = (atcahid_t*)atgetifacehaldat(iface);
    ssize_t bytes_written;

    if ((txdata == NULL) || (cfg == NULL) || (pHid == NULL))
    {
        return ATCA_BAD_PARAM;
    }

    if (pHid->kits[cfg->atcahid.idx].fd == -1 ||
        txlength > pHid->kits[cfg->atcahid.idx].psize)
    {
        return ATCA_COMM_FAIL;
    }

    // Send the data to the kit USB device
    if (txlength > 0)
    {
        bytes_written = write(pHid->kits[cfg->atcahid.idx].fd, txdata,
	    txlength);
        if (bytes_written != txlength)
        {
            return ATCA_TX_FAIL;
        }
    }

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of kit protocol receive.This function is called by the top layer.
 * \param[in]    iface   instance
 * \param[out]   rxdata  pointer to space to receive the data
 * \param[inout] rxsize  ptr to expected number of receive bytes to request
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS kit_phy_receive(ATCAIface iface, uint8_t* rxdata, int* rxsize)
{
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    atcahid_t* pHid = (atcahid_t*)atgetifacehaldat(iface);
    bool continue_read = true;
    char *ibuf;
    int psize;
    ssize_t bytes_read, total_bytes_read;

    if ((rxdata == NULL) || (rxsize == NULL) || (cfg == NULL) || (pHid == NULL))
    {
        return ATCA_BAD_PARAM;
    }

    if (pHid->kits[cfg->atcahid.idx].fd == -1)
    {
        return ATCA_COMM_FAIL;
    }

    ibuf = pHid->kits[cfg->atcahid.idx].ibuf;
    psize = pHid->kits[cfg->atcahid.idx].psize;

    // Receive the data from the kit USB device
    total_bytes_read = 0;
    do
    {
        memset(ibuf, 0, psize);
        bytes_read = read(pHid->kits[cfg->atcahid.idx].fd, ibuf, psize);
        if (bytes_read < 0)
        {
            return ATCA_RX_FAIL;
        }

        if (total_bytes_read < *rxsize)
        {
            memcpy(rxdata + total_bytes_read, ibuf,
                MIN(*rxsize - total_bytes_read, bytes_read));
        }
        total_bytes_read += bytes_read;

        // Check if the kit protocol message has been received
        if (strstr((char*)rxdata, "\n") != NULL)
        {
            continue_read = false;
        }
    }
    while (continue_read == true);

    // Save the total bytes read
    if (total_bytes_read > *rxsize)
        total_bytes_read = *rxsize;
    *rxsize = total_bytes_read;

    return ATCA_SUCCESS;
}

/** \brief Number of USB HID devices found
 *  \param[out] num_found
 *  \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS kit_phy_num_found(int8_t* num_found)
{
    *num_found = _gHid.num_kits_found;

    return ATCA_SUCCESS;
}

/** \brief HAL implementation of kit protocol send over USB HID
 *  \param[in] iface     instance
 *  \param[in] txdata    pointer to bytes to send
 *  \param[in] txlength  number of bytes to send
 *  \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_send(ATCAIface iface, uint8_t* txdata, int txlength)
{
    // Call the kit_send() function that will call phy_send() implemented below
    return kit_send(iface, txdata, txlength);
}

/** \brief HAL implementation of send over USB HID
 * \param[in]    iface   instance
 * \param[in]    rxdata  pointer to space to receive the data
 * \param[inout] rxsize  ptr to expected number of receive bytes to request
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_receive(ATCAIface iface, uint8_t* rxdata, uint16_t* rxsize)
{
    // Call the kit_receive() function that will call phy_receive() implemented below
    return kit_receive(iface, rxdata, rxsize);
}

/** \brief Call the wake for kit protocol over USB HID
 * \param[in] iface  ATCAIface instance that is the interface object to send the bytes over
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_wake(ATCAIface iface)
{
    // Call the kit_wake() function that will call phy_send() and phy_receive()
    return kit_wake(iface);
}

/** \brief Call the idle for kit protocol over USB HID
 * \param[in] iface  ATCAIface instance that is the interface object to send the bytes over
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_idle(ATCAIface iface)
{
    // Call the kit_idle() function that will call phy_send() and phy_receive()
    return kit_idle(iface);
}

/** \brief Call the sleep for kit protocol over USB HID
 * \param[in] iface  ATCAIface instance that is the interface object to send the bytes over
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_sleep(ATCAIface iface)
{
    // Call the kit_sleep() function that will call phy_send() and phy_receive()
    return kit_sleep(iface);
}

/** \brief Close the physical port for HID
 * \param[in] hal_data  The hardware abstraction data specific to this HAL
 * \return ATCA_SUCCESS on success, otherwise an error code.
 */
ATCA_STATUS hal_kit_hid_release(void* hal_data)
{
    atcahid_t* phaldat = (atcahid_t*)hal_data;
    int i;

    if (phaldat == NULL)
    {
        return ATCA_BAD_PARAM;
    }

    // Close all kit USB devices
    for (i = 0; i < phaldat->num_kits_found; i++)
    {
        memset(_gHid.kits[i].dev, 0, sizeof(_gHid.kits[i].dev));
        _gHid.kits[i].psize = 0;
        if (_gHid.kits[i].ibuf != NULL)
        {
            free(_gHid.kits[i].ibuf);
            _gHid.kits[i].ibuf = NULL;
        }
        if (_gHid.kits[i].fd != -1)
        {
            close(_gHid.kits[i].fd);
            _gHid.kits[i].fd = -1;
        }
    }

    return ATCA_SUCCESS;
}

/** @} */
