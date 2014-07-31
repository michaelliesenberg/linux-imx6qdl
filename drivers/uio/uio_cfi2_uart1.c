/* uio_cfi2_uart1:
	UIO driver to access the cfi2 fpga with multiple uart implementations
	-> this uio driver is to have access to uart1 and the corresponding interrupt line
	-> this driver can only be used in combination with the uio_cfi2_main driver

   Copyright (C) 2013 DH electronics GmbH
     Author: Ludwig Zenz <lzenz@dh-electronics.de>

   This is a straight-forward UIO driver, where interrupts are disabled
   by the interrupt handler and re-enabled via a write to the UIO device
   by the userspace-part.

   The only part that may seem odd is the use of a logical OR when
   storing and restoring enabled interrupts. This is done because the
   userspace-part could directly modify the Interrupt Enable Register
   at any time. To reduce possible conflicts, the kernel driver uses
   a logical OR to make more controlled changes (rather than blindly
   overwriting previous values).

   Race conditions exist if the userspace-part directly modifies the
   Interrupt Enable Register while in operation. The consequences are
   that certain interrupts would fail to be enabled or disabled. For
   this reason, the userspace-part should only directly modify the
   Interrupt Enable Register at the beginning (to get things going).
   The userspace-part can safely disable interrupts at any time using
   a write to the UIO device.
*/

#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/workqueue.h>


#include <asm/io.h>
#include <linux/ioport.h>

#define DRIVER_NAME 	"uio_cfi2_uart1"

#define MYFPGA_BASE1    0x01000000
#define MYFPGA_SIZE     0x00020000

#define ADDROFF_UART1	0x00000A00
#define SIZE_UART1	0x00000200

#define DHCOM_GPIOL 58  /* irq uart0 */
#define DHCOM_GPIOM 97  /* irq uart1 */
#define DHCOM_GPIOJ 96  /* irq uart2 */

static unsigned long cfi2_uart1_flags = 0;
static spinlock_t cfi2_uart1_lock;

struct uio_platdata {
	struct uio_info *uioinfo;
};

static int cfi2_uart1_irqcontrol(struct uio_info *info, s32 irq_on)
{

	/* Allow user space to enable and disable the interrupt
	 * in the interrupt controller, but keep track of the
	 * state to prevent per-irq depth damage.
	 *
	 * Serialize this operation to support multiple tasks.
	 */

	if (irq_on) { 	if (test_and_clear_bit(0, &cfi2_uart1_flags)) {
			    enable_irq(info->irq);
			}
	} else { 	if (!test_and_set_bit(0, &cfi2_uart1_flags)) {
			    disable_irq(info->irq);
			}
 	}

	return 0;
}

static irqreturn_t cfi2_uart1_irqhandler(int irq, struct uio_info *info)
{
	if (!test_and_set_bit(0,  &cfi2_uart1_flags))
		disable_irq_nosync(info->irq);

	return IRQ_HANDLED;
}

static struct resource cfi2_uart1_fpga_resources[] = {
    {
        .start = MYFPGA_BASE1 + ADDROFF_UART1,
        .end   = MYFPGA_BASE1 + ADDROFF_UART1 + SIZE_UART1 - 1,
        .name  = "cfi2_uart1",
        .flags = IORESOURCE_MEM
    }
};


static struct uio_info cfi2_uart1_uio_info = {
   .name = DRIVER_NAME,
   .version = "0.1",
   .irq = 0,
   .irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
   .handler = cfi2_uart1_irqhandler,
   .irqcontrol = cfi2_uart1_irqcontrol,
};


static int uio_cfi2_fpga_probe(struct platform_device *pdev)
{
	struct uio_info *uioinfo = pdev->dev.platform_data;
	struct uio_platdata *pdata;
	struct uio_mem *uiomem;
	int ret = -ENODEV;
	int i;

	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_dbg(&pdev->dev, "%s: err_uioinfo\n", __func__);
		goto err_uioinfo;
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_pdata\n", __func__);
		goto err_alloc_pdata;
	}

	pdata->uioinfo = uioinfo;

	uiomem = &uioinfo->mem[0];

	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = resource_size(r);
		uiomem->name = r->name;
		++uiomem;
	}

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	pdata->uioinfo->priv = pdata;

	ret = uio_register_device(&pdev->dev, pdata->uioinfo);

	if (ret) {
		kfree(pdata);
err_alloc_pdata:
err_uioinfo:
		return ret;
	}

	platform_set_drvdata(pdev, pdata);

	return 0;
}

static int uio_cfi2_fpga_remove(struct platform_device *pdev)
{
	struct uio_platdata *pdata = platform_get_drvdata(pdev);
	uio_unregister_device(pdata->uioinfo);
	kfree(pdata);
	return 0;
}

static struct platform_driver uio_cfi2_fpga_driver = {
	 .probe = uio_cfi2_fpga_probe,
	 .remove = uio_cfi2_fpga_remove,
 	 .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},

 };

static struct platform_device *cfi2_uart1_uio_pdev;

static int __init cfi2_uart1_init_module(void)
{
    int rc;
    int irq;

    rc = gpio_request(DHCOM_GPIOM, cfi2_uart1_uio_info.name);
    if (rc) {
        printk("Failed to request gpio: %d\n", rc);
    }

    rc = gpio_direction_input(DHCOM_GPIOM);
    if (rc) {
        printk("Failed to get gpio as input: %d\n", rc);
        gpio_free(DHCOM_GPIOM);
    }

    irq = gpio_to_irq(DHCOM_GPIOM);
    if (irq <= 0) {
        printk("Failed to get gpio as irq: %d\n", irq);
        gpio_free(DHCOM_GPIOM);
    }

    cfi2_uart1_uio_info.irq = irq;

    spin_lock_init(&cfi2_uart1_lock);
    cfi2_uart1_flags = 0; /* interrupt is enabled to begin with */

    /* register memory mapping */
    cfi2_uart1_uio_pdev = platform_device_register_resndata (NULL,
                                                         DRIVER_NAME,
                                                         -1,
                                                         cfi2_uart1_fpga_resources,
                                                         1,
                                                         &cfi2_uart1_uio_info,
                                                         sizeof(struct uio_info)
                                                        );
    if (IS_ERR(cfi2_uart1_uio_pdev)) {
        return PTR_ERR(cfi2_uart1_uio_pdev);
    }

    return platform_driver_register(&uio_cfi2_fpga_driver);
}

static void __exit cfi2_uart1_exit_module(void)
{
    platform_device_unregister(cfi2_uart1_uio_pdev);
    platform_driver_unregister(&uio_cfi2_fpga_driver);
}

module_init(cfi2_uart1_init_module);
module_exit(cfi2_uart1_exit_module);

MODULE_DESCRIPTION("UIO driver for the CFI2 FPGA UART1");
MODULE_AUTHOR("Ludwig Zenz <lzenz@dh-electronics.de>");
MODULE_LICENSE("GPL v2");
