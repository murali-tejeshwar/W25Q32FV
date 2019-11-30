#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/of.h>

#define W25Q32FV_CMD_WRITE_ENABLE 		0x06
#define W25Q32FV_CMD_SECTOR_ERASE 		0x20
#define W25Q32FV_CMD_PAGE_PROGRAM 		0x02
#define W25Q32FV_CMD_READ_DATA    		0x03
#define W25Q32FV_CMD_READ_STATUS_REG1		0x05

/* private structure of the device */
typedef struct {
	struct spi_device *spi;
	struct mtd_info mtd;
	struct mutex lock;
}w25q32fv_flash;

/* wrapper function for enabling write operation on the device */
static inline int w25q32fv_write_enable(w25q32fv_flash *dev, u_char wen)
{
	return spi_write(dev->spi, &wen, sizeof(u_char));
}

/* wrapper function for checking if the device is busy */
static inline int w25q32fv_device_busy(w25q32fv_flash *dev, u_char db, u_char *db_out)
{
	return spi_write_then_read(dev->spi, &db, sizeof(u_char), db_out, sizeof(u_char));
}

/* wrapper function for erasing a sector */
static int w25q32fv_sector_erase(w25q32fv_flash *dev, u32 erase_addr)
{
	u_char db[2], se[4], wen;
	int ret;

	/* check if the device is busy */
	db[0] = W25Q32FV_CMD_READ_STATUS_REG1;
	ret = w25q32fv_device_busy(dev, db[0], &db[1]);
	if (ret)
		return ret;
	if (db[1] & 1)
		return -EBUSY;

	/* write enable */
	wen = W25Q32FV_CMD_WRITE_ENABLE;
	ret = w25q32fv_write_enable(dev, wen);
	if (ret)
		return ret;

	/* erase the sector */
	se[0] = W25Q32FV_CMD_SECTOR_ERASE;
	se[1] = erase_addr >> 16;
	se[2] = erase_addr >> 8;
	se[3] = erase_addr;
	ret = spi_write(dev->spi, se, sizeof(se));
	if (ret)
		return ret;

	/* put the caller to sleep for 400 milliseconds(time taken by the hardware to erase a sector) */
	if (msleep_interruptible(400))
		return -EINTR;

	return 0;
}

static int w25q32fv_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret;
	u32 start_addr, end_addr;
	w25q32fv_flash *dev = mtd->priv;

	pr_info("%s() invoked...\n", __func__);

	/* sanity check */
	if ((uint32_t)instr->len % mtd->erasesize || (uint32_t)instr->addr % mtd->erasesize)
		return -EINVAL;

	start_addr = instr->addr;
	end_addr = start_addr + instr->len;

	ret = mutex_lock_interruptible(&dev->lock);
	if (ret)
		return ret;

	while (start_addr < end_addr) {
		ret = w25q32fv_sector_erase(dev, start_addr);
		if (ret) {
			instr->fail_addr = start_addr;
			goto release_lock;
		}
		start_addr += mtd->erasesize;
	}

release_lock:
	mutex_unlock(&dev->lock);

	return ret;
}

/* wrapper function for reading data from the device */
static int w25q32fv_do_read(w25q32fv_flash *dev, u32 addr, size_t len, size_t *retlen, u_char *to_buf)
{
	struct spi_message message;
	struct spi_transfer transfer[2];
	u_char db[2], rd[4];
	int ret;

	/* check if the device is busy */
	db[0] = W25Q32FV_CMD_READ_STATUS_REG1;
	ret = w25q32fv_device_busy(dev, db[0], &db[1]);
	if (ret)
		return ret;
	if (db[1] & 1)
		return -EBUSY;

	/* read data from the device */
	memset(transfer, 0, sizeof(transfer));
	rd[0] = W25Q32FV_CMD_READ_DATA;
	rd[1] = addr >> 16;
	rd[2] = addr >> 8;
	rd[3] = addr;
	transfer[0].tx_buf = rd;
	transfer[0].len = sizeof(rd);
	transfer[1].rx_buf = to_buf;
	transfer[1].len = len;
	spi_message_init_with_transfers(&message, transfer, 2);
	ret = spi_sync(dev->spi, &message);
	if (ret)
		return ret;

	*retlen = len;

	return 0;
}

static int w25q32fv_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	int ret;
	w25q32fv_flash *dev = mtd->priv;

	pr_info("%s() invoked...\n", __func__);

	ret = mutex_lock_interruptible(&dev->lock);
	if (ret)
		return ret;

	ret = w25q32fv_do_read(dev, from, len, retlen, buf);

	mutex_unlock(&dev->lock);

	return ret;
}

/* wrapper function for writing data to the device */
static int w25q32fv_do_write(w25q32fv_flash *dev, u32 addr, size_t len, size_t *retlen, const u_char *from_buf)
{
	struct spi_message message;
	struct spi_transfer transfer[2];
	u_char db[2], wen, wr[4];
	int ret;

	/* check if the device is busy */
	db[0] = W25Q32FV_CMD_READ_STATUS_REG1;
	ret = w25q32fv_device_busy(dev, db[0], &db[1]);
	if (ret)
		return ret;
	if (db[1] & 1)
		return -EBUSY;

	/* write enable */
	wen = W25Q32FV_CMD_WRITE_ENABLE;
	ret = w25q32fv_write_enable(dev, wen);
	if (ret)
		return ret;

	/* write data to the device */
	memset(transfer, 0, sizeof(transfer));
	wr[0] = W25Q32FV_CMD_PAGE_PROGRAM;
	wr[1] = addr >> 16;
	wr[2] = addr >> 8;
	wr[3] = addr;
        transfer[0].tx_buf = wr;
        transfer[0].len = sizeof(wr);
        transfer[1].tx_buf = from_buf;
        transfer[1].len = len;
	spi_message_init_with_transfers(&message, transfer, 2);
        ret = spi_sync(dev->spi, &message);
	if (ret)
		return ret;

	/* put the caller to poll for 3 milliseconds(time taken by the hardware to program a page) */
	mdelay(3);

	*retlen += len;

	return 0;
}

static int w25q32fv_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int ret;
	size_t remaining, wr_len;
	w25q32fv_flash *dev = mtd->priv;

	pr_info("%s() invoked...\n", __func__);

	/* sanity check */
	if ((uint32_t)to % mtd->writesize)
		return -EINVAL;

	remaining = len;

	ret = mutex_lock_interruptible(&dev->lock);
	if (ret)
		return ret;

	while (remaining) {
		wr_len = remaining > mtd->writesize ? mtd->writesize : remaining;
		ret = w25q32fv_do_write(dev, to, wr_len, retlen, buf);
		if (ret)
			goto release_lock;

		to += wr_len;
		buf += wr_len;
		remaining -= wr_len;
	}

release_lock:
	mutex_unlock(&dev->lock);

	return ret;
}

static int w25q32fv_spi_probe(struct spi_device *spi)
{
	int ret;
	struct flash_platform_data *platdata;
	u32 size, pagesize, sectorsize;
	w25q32fv_flash *dev;

	pr_info("%s() invoked...\n", __func__);

	/* retrieve details of the device, from the device node */
	ret = of_property_read_u32(spi->dev.of_node, "size", &size);
	if (ret)
		return ret;

	ret = of_property_read_u32(spi->dev.of_node, "pagesize", &pagesize);
	if (ret)
		return ret;

	ret = of_property_read_u32(spi->dev.of_node, "sectorsize", &sectorsize);
	if (ret)
		return ret;

	platdata = (struct flash_platform_data *)dev_get_platdata(&spi->dev);

	dev = devm_kzalloc(&spi->dev, sizeof(w25q32fv_flash), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->spi = spi;
	mutex_init(&dev->lock);
	dev->mtd.type = MTD_NORFLASH;
	dev->mtd.flags = MTD_CAP_NORFLASH;
	dev->mtd.size = size;
	dev->mtd.erasesize = sectorsize;
	dev->mtd.writesize = pagesize;
	dev->mtd._erase = w25q32fv_erase;
	dev->mtd._read = w25q32fv_read;
	dev->mtd._write = w25q32fv_write;
	dev->mtd.owner = THIS_MODULE;
	dev->mtd.priv = dev;
	dev->mtd.dev.parent = &spi->dev;
	dev->mtd.name = platdata && platdata->name ? platdata->name : "winbond-w25q32fv";

	/* register with MTD subsystem */
	ret = mtd_device_register(&dev->mtd, platdata ? platdata->parts : NULL, platdata ? platdata->nr_parts : 0);
	if (ret)
		goto err_do_kfree;

	spi_set_drvdata(spi, dev);

	return 0;

err_do_kfree:
	devm_kfree(&spi->dev, dev);	/* undo devm_kzalloc() */

	return ret;
}

static int w25q32fv_spi_remove(struct spi_device *spi)
{
	w25q32fv_flash *dev = spi_get_drvdata(spi);

	pr_info("%s() invoked...\n", __func__);

	/* unregister from MTD subsystem */
	return mtd_device_unregister(&dev->mtd);
}

static const struct of_device_id of_match[] = {
        { .compatible = "winbond,w25q32fv" },
        {/* end of list */},
};
MODULE_DEVICE_TABLE(of, of_match);

static const struct spi_device_id winbond_ids[] = {
	{ .name = "w25q32fv", 0x0 },
	{/* end of list */},
};
MODULE_DEVICE_TABLE(spi, winbond_ids);

static struct spi_driver winbond_w25q32fv_drv = {
	.probe = w25q32fv_spi_probe,
	.remove = w25q32fv_spi_remove,
	.id_table = winbond_ids,
	.driver = {
		.name = "w25q32fv-slave-spi",
		.of_match_table = of_match,
		.owner = THIS_MODULE,
	},
};
module_spi_driver(winbond_w25q32fv_drv);

MODULE_AUTHOR("Murali Tejeshwar Janaswami");
MODULE_DESCRIPTION("SPI slave driver for W25Q32FV");
MODULE_LICENSE("GPL");
