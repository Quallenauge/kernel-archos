/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) SPI touchscreen driver.
 * For use with Cypress Gen4 and Solo parts.
 * Supported parts include:
 * CY8CTMA884/616
 * CY8CTMA4XX
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include "cyttsp4_core.h"

#include <linux/spi/spi.h>
#include <linux/delay.h>

#define CY_SPI_WR_OP                0x00 /* r/~w */
#define CY_SPI_RD_OP                0x01
#define CY_SPI_A8_BIT               0x02
#define CY_SPI_WR_HEADER_BYTES      2
#define CY_SPI_RD_HEADER_BYTES      1
#define CY_SPI_SYNC_BYTE            0
#define CY_SPI_SYNC_ACK             0x62 /* from TRM *A protocol */
#define CY_SPI_DATA_SIZE            (3 * 256)
#define CY_SPI_BITS_PER_WORD        8
#define CY_SPI_MAX_REG              512


struct cyttsp4_spi {
	struct cyttsp4_bus_ops bus_ops;
	struct spi_device *spi_client;
	void *ttsp_client;
	u8 *wr_buf ____cacheline_aligned;
	u8 *rd_buf ____cacheline_aligned;
};

static void _cyttsp4_spi_pr_buf(struct cyttsp4_spi *ts, u8 *dptr, int size,
	const char *data_name)
{
	return;
}

static int cyttsp4_spi_xfer(u8 op, struct cyttsp4_spi *ts,
	u16 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer[2];
	int retval;

	memset(ts->wr_buf, 0, length + CY_SPI_WR_HEADER_BYTES);
	memset(ts->rd_buf, 0, length + CY_SPI_RD_HEADER_BYTES);
	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);
	xfer[0].tx_buf = ts->wr_buf;
	xfer[0].rx_buf = ts->rd_buf;
	switch (op) {
	case CY_SPI_WR_OP:
		if ((length + CY_SPI_WR_HEADER_BYTES) > CY_SPI_DATA_SIZE) {
			dev_vdbg(ts->bus_ops.dev,
				"%s: length+%d=%d is greater than SPI max=%d\n",
				__func__, CY_SPI_WR_HEADER_BYTES,
				length + CY_SPI_WR_HEADER_BYTES,
				CY_SPI_DATA_SIZE);
			retval = -EINVAL;
			goto cyttsp4_spi_xfer_exit;
		}
		/* header byte 0 */
		if (reg / (CY_SPI_MAX_REG / 2))
			ts->wr_buf[0] = CY_SPI_WR_OP + CY_SPI_A8_BIT;
		else
			ts->wr_buf[0] = CY_SPI_WR_OP;
		/* header byte 1 */
		ts->wr_buf[1] = reg % (CY_SPI_MAX_REG / 2);
		if (buf != NULL)
			memcpy(&ts->wr_buf[2], buf, length);
		xfer[0].len = length + CY_SPI_WR_HEADER_BYTES;
		spi_message_add_tail(&xfer[0], &msg);
		break;
	case CY_SPI_RD_OP:
		if (buf == NULL) {
			dev_err(ts->bus_ops.dev,
			"%s: read return buf=%p\n", __func__, buf);
			retval = -EINVAL;
			goto cyttsp4_spi_xfer_exit;
		}
		if ((length + CY_SPI_RD_HEADER_BYTES) > CY_SPI_DATA_SIZE) {
			dev_vdbg(ts->bus_ops.dev,
				"%s: length+%d=%d is greater than SPI max=%d\n",
				__func__, CY_SPI_RD_HEADER_BYTES,
				length + CY_SPI_RD_HEADER_BYTES,
				CY_SPI_DATA_SIZE);
			retval = -EINVAL;
			goto cyttsp4_spi_xfer_exit;
		}
		/* header byte 0 */
		ts->wr_buf[0] = CY_SPI_RD_OP;
		xfer[0].len = CY_SPI_RD_HEADER_BYTES;
		spi_message_add_tail(&xfer[0], &msg);
		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
		break;
	default:
		dev_dbg(ts->bus_ops.dev,
			"%s: bad op code=%d\n", __func__, op);
		retval = -EINVAL;
		goto cyttsp4_spi_xfer_exit;
	}

	retval = spi_sync(ts->spi_client, &msg);
	if (retval < 0) {
		dev_vdbg(ts->bus_ops.dev,
			"%s: spi_sync() error %d, len=%d, op=%d\n",
			__func__, retval, xfer[0].len, op);

		/*
		 * do not return here since probably a bad ACK sequence
		 * let the following ACK check handle any errors and
		 * allow silent retries
		 */
	}

	if (ts->rd_buf[CY_SPI_SYNC_BYTE] != CY_SPI_SYNC_ACK) {
		/* signal ACK error so silent retry */
		retval = 1;
		switch (op) {
		case CY_SPI_WR_OP:
			_cyttsp4_spi_pr_buf(ts, ts->wr_buf,
				length + CY_SPI_WR_HEADER_BYTES, "spi_wr_buf");
			break;
		case CY_SPI_RD_OP:
			_cyttsp4_spi_pr_buf(ts, ts->rd_buf,
				length + CY_SPI_RD_HEADER_BYTES, "spi_rd_buf");
			break;
		default:
			/*
			 * should not get here due to error check
			 * in first switch
			 */
			break;
		}
	} else
		retval = 0;

cyttsp4_spi_xfer_exit:
	return retval;
}

static s32 cyttsp4_spi_read_block_data(void *handle, u16 addr,
	size_t length, void *data, int spi_addr, bool use_subaddr)
{
	struct cyttsp4_spi *ts =
		container_of(handle, struct cyttsp4_spi, bus_ops);
	int retval;

	retval = cyttsp4_spi_xfer(CY_SPI_WR_OP, ts, addr, NULL, 0);
	if (retval == 0)
		retval = cyttsp4_spi_xfer(CY_SPI_RD_OP, ts, addr, data, length);
	if (retval < 0)
		dev_err(ts->bus_ops.dev,
			"%s: cyttsp4_spi_read_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	else if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

	return retval;
}

static s32 cyttsp4_spi_write_block_data(void *handle, u16 addr,
	size_t length, const void *data, int spi_addr, bool use_subaddr)
{
	struct cyttsp4_spi *ts =
		container_of(handle, struct cyttsp4_spi, bus_ops);
	int retval;

	retval = cyttsp4_spi_xfer(CY_SPI_WR_OP, ts, addr, (void *)data, length);
	if (retval < 0)
		dev_err(ts->bus_ops.dev,
			"%s: cyttsp4_spi_write_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	else if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

	return retval;
}

static int __devinit cyttsp4_spi_probe(struct spi_device *spi)
{
	struct cyttsp4_spi *ts = NULL;
	int retval = 0;

	/* Set up SPI */
	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	retval = spi_setup(spi);
	if (retval < 0) {
		pr_err("%s: SPI setup error %d\n",
			__func__, retval);
		goto cyttsp4_spi_probe_exit;
	}

	ts = kzalloc(sizeof(struct cyttsp4_spi), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, ts kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto cyttsp4_spi_probe_exit;
	}

	ts->wr_buf = kzalloc(CY_SPI_DATA_SIZE + CY_SPI_WR_HEADER_BYTES,
		GFP_KERNEL);
	if (ts->wr_buf == NULL) {
		pr_err("%s: Error, ts->wr_buf kzalloc.\n", __func__);
		kfree(ts);
		ts = NULL;
		retval = -ENOMEM;
		goto cyttsp4_spi_probe_exit;
	}

	ts->rd_buf = kzalloc(CY_SPI_DATA_SIZE + CY_SPI_RD_HEADER_BYTES,
		GFP_KERNEL);
	if (ts->rd_buf == NULL) {
		pr_err("%s: Error, ts->rd_buf kzalloc.\n", __func__);
		kfree(ts->wr_buf);
		ts->wr_buf = NULL;
		kfree(ts);
		ts = NULL;
		retval = -ENOMEM;
		goto cyttsp4_spi_probe_exit;
	}


	ts->spi_client = spi;
	dev_set_drvdata(&spi->dev, ts);
	ts->bus_ops.write = cyttsp4_spi_write_block_data;
	ts->bus_ops.read = cyttsp4_spi_read_block_data;
	ts->bus_ops.dev = &spi->dev;

	ts->ttsp_client = cyttsp4_core_init(&ts->bus_ops, &spi->dev,
		spi->irq, spi->modalias);
	if (ts->ttsp_client == NULL) {
		retval = -ENODATA;
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
	} else
		dev_info(ts->bus_ops.dev,
			"%s: Registration complete\n", __func__);

cyttsp4_spi_probe_exit:
	return retval;
}

static int __devexit cyttsp4_spi_remove(struct spi_device *spi)
{
	struct cyttsp4_spi *ts = dev_get_drvdata(&spi->dev);

	cyttsp4_core_release(ts->ttsp_client);
	if (ts != NULL) {
		if (ts->wr_buf != NULL) {
			kfree(ts->wr_buf);
			ts->wr_buf = NULL;
		}
		if (ts->rd_buf != NULL) {
			kfree(ts->rd_buf);
			ts->rd_buf = NULL;
		}
		kfree(ts);
		ts = NULL;
	}
	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_PM_SLEEP)
#if defined(CONFIG_PM)
static int cyttsp4_spi_suspend(struct spi_device *spi, pm_message_t message)
{
	return cyttsp4_suspend(dev_get_drvdata(&spi->dev));
}

static int cyttsp4_spi_resume(struct spi_device *spi)
{
	return cyttsp4_resume(dev_get_drvdata(&spi->dev));
}
#endif
#endif

static struct spi_driver cyttsp4_spi_driver = {
	.driver = {
		.name = CY_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
#if defined(CONFIG_PM_SLEEP)
		.pm = &cyttsp4_pm_ops,
#endif
#endif
	},
	.probe = cyttsp4_spi_probe,
	.remove = __devexit_p(cyttsp4_spi_remove),
#if !defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_PM_SLEEP)
#if defined(CONFIG_PM)
	.suspend = cyttsp4_spi_suspend,
	.resume = cyttsp4_spi_resume,
#endif
#endif
};

static int __init cyttsp4_spi_init(void)
{
	return spi_register_driver(&cyttsp4_spi_driver);
}
module_init(cyttsp4_spi_init);

static void __exit cyttsp4_spi_exit(void)
{
	spi_unregister_driver(&cyttsp4_spi_driver);
}
module_exit(cyttsp4_spi_exit);

MODULE_ALIAS(CY_SPI_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) SPI driver");
MODULE_AUTHOR("Cypress");

