#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include "cryptoauthlib.h"
#include "atca_hal.h"
#include "hal_linux_i2c.h"

atcai2c_t i2c_dev;

/**
 * \brief Initialize an I2C interface using given config.
 *
 * \param[in] hal  opaque pointer to HAL data
 * \param[in] cfg  interface configuration
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
	printf("[%s] Enter\n", __FUNCTION__);

	unsigned long funcs = 0;
	int file;
	char filename_i2c_bus[32];
	ATCAHAL_t *phal = (ATCAHAL_t*)hal;

	memset(&i2c_dev, 0x0, sizeof(i2c_dev));

	// Check the input variables
	if ((hal == NULL) || (cfg == NULL)) {
		return ATCA_BAD_PARAM;
	}

	sprintf(filename_i2c_bus, "/dev/i2c-%d", cfg->atcai2c.bus);

	printf("[%s] slave_address: 0x%X\n", __FUNCTION__, cfg->atcai2c.slave_address);
	printf("[%s] bus file: %s\n", __FUNCTION__, filename_i2c_bus);

	//
	// Open the i2c device, check capabitlities and set slave address
	//
	if ((file = open(filename_i2c_bus, O_RDWR)) == -1) {
		perror("Failed to open the i2c bus");
		return ATCA_ASSERT_FAILURE;
	}

	if (ioctl(file, I2C_FUNCS, &funcs) == -1) {
		perror("Failed to get capabitlities");
		return ATCA_IOCTL_ERROR;
	}

	if (funcs & I2C_FUNC_I2C) {
		printf("I2C_FUNC_I2C supported\n");
	} else {
		printf("I2C_FUNC_I2C NOT supported\n");

		return ATCA_IOCTL_ERROR;
	}

	if (ioctl(file, I2C_SLAVE, cfg->atcai2c.slave_address >> 1) == -1) {
		perror("Failed to set i2c slave address");
		return ATCA_IOCTL_ERROR;
	}

	i2c_dev.fd = file;
	phal->hal_data = &i2c_dev;

	return ATCA_SUCCESS;
}

/**
 * \brief HAL implementation of I2C post init.
 *
 * \param[in] iface  ATCAIface instance
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
	printf("[%s] Enter\n", __FUNCTION__);

	return ATCA_SUCCESS;
}

/**
 * \brief Receive byte(s) via I2C.
 *
 * \param[in] iface     interface of the logical device to receive data
 *                      from
 * \param[in] rxdata    pointer to where bytes will be received
 * \param[in] rxlength  pointer to expected number of receive bytes to
 *                      request
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = atgetifacehaldat(iface);
	int file           = i2c_dev->fd;
	int rcv;

	rcv = read(file, rxdata, *rxlength);
	if (rcv == -1) {
		perror("Unable to receive");
		return ATCA_COMM_FAIL;
	}

	*rxlength = rcv;

	return ATCA_SUCCESS;
}


/**
 * \brief Send byte(s) via I2C.
 *
 * \param[in] iface     interface of the logical device to send data to
 * \param[in] txdata    pointer to bytes to send
 * \param[in] txlength  number of bytes to send
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = atgetifacehaldat(iface);
	int file           = i2c_dev->fd;

	//
	// For this implementation of I2C with CryptoAuth chips, txdata is
	// assumed to have ATCAPacket format. Other device types that don't
	// require i/o tokens on the front end of a command need a different
	// hal_i2c_send and wire it up instead of this one. This covers devices
	// such as ATSHA204A and ATECCx08A that require a word address value
	// pre-pended to the packet. txdata[0] is using _reserved byte of the
	// ATCAPacket
	//
	txdata[0] = 0x03;   // insert the Word Address Value, Command token
	txlength++;         // account for word address value byte.

	if (write(file, txdata, txlength) == -1) {
		perror("Unable to send");
		return ATCA_COMM_FAIL;
	}

	return ATCA_SUCCESS;
}

/**
 * \brief Send Wake flag via I2C.
 *
 * \param[in] iface  interface of the logical device to wake up
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = atgetifacehaldat(iface);
	ATCAIfaceCfg *cfg  = atgetifacecfg(iface);
	int file           = i2c_dev->fd;
	int retries        = cfg->rx_retries;
	int status;
	char buf[4]        = { 0 };
	// This is expected word received after wake-up
	char expected[4]   = { 0x04, 0x11, 0x33, 0x43 };

	// Sending message with address 0 on the bus (if bus freq is 100KHz) should
	// wake the device.
	struct i2c_msg msg = {
		.addr  = 0,
		.flags = I2C_M_IGNORE_NAK,
		.len   = 0,
		.buf   = NULL,
	};

	struct i2c_rdwr_ioctl_data data = {
		.msgs  = &msg,
		.nmsgs = 1,
	};

	// We need only to create wake pulse. Device will respond with NAK,
	// so we are not checking for the status code
	ioctl(file, I2C_RDWR, &data);

	atca_delay_us(cfg->wake_delay);

	// Check for the wake message from the device
	msg.addr  = cfg->atcai2c.slave_address >> 1;
	msg.len   = sizeof(buf);
	msg.buf   = buf;
	msg.flags = I2C_M_RD;

	do {
		status = ioctl(file, I2C_RDWR, &data);
	} while (retries-- > 0 && (status == -1));

	if (status == -1) {
		printf("Wake retries (%d) exceeded. Communication failure\n", cfg->rx_retries);
		return ATCA_COMM_FAIL;
	}

	if (memcmp(buf, expected, sizeof(buf)) == 0) {
		return ATCA_SUCCESS;
	} else {
		printf("Wake magic packet is not correct. Failure\n");
		return ATCA_COMM_FAIL;
	}
}

/**
 * \brief Send Sleep flag via I2C.
 *
 * \param[in] iface  interface of the logical device to sleep
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = atgetifacehaldat(iface);
	int file           = i2c_dev->fd;
	uint8_t data[1];

	data[0] = 0x01; // sleep word address value

	if (write(file, data, sizeof(data)) == -1) {
		// Device is not responding with ACK in this case,
		// so apparently we're getting the i/o error all the time.
		// For now it's better to ignore the output of this write.
	}

	return ATCA_SUCCESS;
}

/**
 * \brief Send Idle flag via I2C.
 *
 * \param[in] iface  interface of the logical device to idle
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = atgetifacehaldat(iface);
	int file = i2c_dev->fd;

	uint8_t data[1];

	data[0] = 0x02; // idle word address value

	if (write(file, data, sizeof(data)) == -1) {
		perror("Unable to send idle command");
		return ATCA_COMM_FAIL;
	}

	return ATCA_SUCCESS;
}

/**
 * \brief Manages reference count on given bus and releases resource if
 *        no more reference(s) exist.
 *
 * \param[in] hal_data  opaque pointer to hal data structure - known only
 *                      to the HAL implementation
 *
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_release(void *hal_data)
{
	printf("[%s] Enter\n", __FUNCTION__);

	atcai2c_t* i2c_dev = hal_data;

	int file = i2c_dev->fd;

	close(file);

	return ATCA_SUCCESS;
}


/** \brief discover i2c buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge
 * \param[in] i2c_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 */

ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
{
	printf("[%s] Enter\n", __FUNCTION__);

	return ATCA_UNIMPLEMENTED;
}

/** \brief discover any CryptoAuth devices on a given logical bus number
 * \param[in]  busNum  logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg     pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] found   number of devices found on this bus
 */

ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found )
{
	printf("[%s] Enter\n", __FUNCTION__);

	return ATCA_UNIMPLEMENTED;
}

