// Copyright 2017
// Original Author: Huide Zhou (prettyage.new@gmail.com)

#include "ft_driver.h"
#include "ft_macros.h"
#include <linux/interrupt.h>

MODULE_LICENSE("MIT");
MODULE_AUTHOR("Huide Zhou <prettyage.new@gmail.com>");
MODULE_DESCRIPTION("Driver for PCIe Fibre-Test board.");

/* Forward Static PCI driver functions for kernel module */
static int probe(struct pci_dev *dev, const struct pci_device_id *id);
static void remove(struct pci_dev *dev);

// static int  init_chrdev (struct aclpci_dev *aclpci);

//Fill in kernel structures with a list of ids this driver can handle
static struct pci_device_id idTable[] = {
    { PCI_DEVICE(VENDOR_ID, DEVICE_ID) },
    { 0, },
};
MODULE_DEVICE_TABLE(pci, idTable);

//PCI driver structure
static struct pci_driver fpgaDriver = {
    .name = DRIVER_NAME,
    .id_table = idTable,
    .probe = probe,
    .remove = remove,
};

//file operations structure
struct file_operations fileOps = {
    .owner =    THIS_MODULE,
    .read =     fpga_read,
    .write =    fpga_write,
    .open =     fpga_open,
    .release =  fpga_close,
};

//IO COMMAND TYPE
enum FPGA_IO_CMD {
    BAR_IO = 0,
    BAR_IO_REPEAT,
    DMA_DATA,
    
    FPGA_IO_CMD_END
};

/*I/0 - should move to separate file at some point */
struct IOCmd_t {
    uint32_t cmd; //command word
    uint8_t barNum; //which bar we are read/writing to
    uint32_t devAddr; // address relative to BAR that we are read/writing to
    void * userAddr; // virtual address in user space to read/write from
};

void write_bar0_u32(struct DevInfo_t * dev_info, uint32_t offset, uint32_t value) {
    //prepare address
    void * startAddr = (void*) (dev_info->bar[0] + offset) ;
    //check offset validation
    if(offset<dev_info->barLengths[0]) {
        iowrite32(value, startAddr);
    }
}

uint32_t read_bar0_u32(struct DevInfo_t * dev_info, uint32_t offset) {
    //prepare address
    void * startAddr = (void*) (dev_info->bar[0] + offset) ;
    //check offset validation
    if(offset<dev_info->barLengths[0]) {
        return ioread32(startAddr);
    }
    return 0;
}

ssize_t rw_dispatcher(struct file *filePtr, char __user *buf, size_t count, bool rwFlag) {

    //Read the command from the buffer
    struct IOCmd_t iocmd; 
    void * startAddr;

    size_t bytesDone = 0;
    size_t bytesToTransfer = 0;
    size_t remainSize = 0;
    size_t currentIndex = 0;

    struct DevInfo_t * devInfo = (struct DevInfo_t *) filePtr->private_data;

    printk(KERN_INFO "[FT] rw_dispatcher: Entering function.\n");

    //fetch command
    copy_from_user(&iocmd, (void __user *) buf, sizeof(iocmd));
    //check command
    if((iocmd.cmd<BAR_IO) || (iocmd.cmd>=FPGA_IO_CMD_END)) {
        printk(KERN_WARNING "[FT] rw_dispatcher: Invalid command type: %u!\n", iocmd.cmd);
        return -1;
    }
    //check bar
    if((iocmd.cmd>=BAR_IO) && (iocmd.cmd<=BAR_IO_REPEAT)) {
        if((iocmd.barNum>=NUM_BARS)) {
            printk(KERN_WARNING "[FT] rw_dispatcher: Invalid bar number: %u!\n", iocmd.barNum);
            return -1;
        }
        if((iocmd.devAddr>=devInfo->barLength[iocmd.barNum])) {
            printk(KERN_WARNING "[FT] rw_dispatcher: Invalid device address %u in bar %u!\n", iocmd.devAddr, iocmd.barNum);
            return -1;
        }
        if((iocmd.cmd==BAR_IO) && ((iocmd.devAddr+count)>=devInfo->barLength[iocmd.barNum])) {
            printk(KERN_WARNING "[FT] rw_dispatcher: Invalid count %u from device address %u in bar %u!\n", (unsigned int)count, iocmd.devAddr, iocmd.barNum);
            return -1;
        }
        //Map the device address to the iomaped memory
        startAddr = (void*) (devInfo->bar[iocmd.barNum] + iocmd.devAddr) ;
    }

    //lock device
    if (down_interruptible(&devInfo->sem)) {
        printk(KERN_WARNING "[FT] rw_dispatcher: Unable to get semaphore!\n");
        return -1;
    }

    //operate depending command TYPE
    switch (iocmd.cmd) {
        case BAR_IO: {
            printk(KERN_INFO "[FT] rw_dispatcher: Reading/writing %u bytes from user address 0x%p to device address %u.\n",
                   (unsigned int) count, iocmd.userAddr, iocmd.devAddr);
            while (count > 0){
                bytesToTransfer = (count > BUFFER_SIZE) ? BUFFER_SIZE : count;
                remainSize = (bytesToTransfer+3)/4;
                if (rwFlag) {
                    //First read from device into kernel memory 
                    currentIndex = 0;
                    while (remainSize > 0) {
                        devInfo->buffer[currentIndex] = ioread32(startAddr + bytesDone + currentIndex*4);
                        remainSize--; currentIndex++;
                    }
                    //Then into user space
                    copy_to_user(iocmd.userAddr + bytesDone, (char*)devInfo->buffer, bytesToTransfer);
                }
                else{
                    //First copy from user to buffer
                    copy_from_user((char*)devInfo->buffer, iocmd.userAddr + bytesDone, bytesToTransfer);
                    //Then into the device
                    currentIndex = 0;
                    while (remainSize > 0) {
                        iowrite32(devInfo->buffer[currentIndex], startAddr + bytesDone + currentIndex*4);
                        remainSize--; currentIndex++;
                    }
                }
                bytesDone += bytesToTransfer;
                count -= bytesToTransfer;
            }
            break;
        }
        case BAR_IO_REPEAT: {
            printk(KERN_INFO "[FT] rw_dispatcher: Reading/writing %u bytes repeatly from user address 0x%p to device address %u.\n",
                   (unsigned int) count, iocmd.userAddr, iocmd.devAddr);
            while (count > 0){
                bytesToTransfer = (count > BUFFER_SIZE) ? BUFFER_SIZE : count;
                remainSize = (bytesToTransfer+3)/4;
                if (rwFlag) {
                    //First read from device into kernel memory 
                    currentIndex = 0;
                    while (remainSize > 0) {
                        devInfo->buffer[currentIndex] = ioread32(startAddr);
                        remainSize--; currentIndex++;
                    }
                    //Then into user space
                    copy_to_user(iocmd.userAddr + bytesDone, (char*)devInfo->buffer, bytesToTransfer);
                }
                else{
                    //First copy from user to buffer
                    memset(devInfo->buffer,0,remainSize*4);
                    copy_from_user((char*)devInfo->buffer, iocmd.userAddr + bytesDone, bytesToTransfer);
                    //Then into the device
                    currentIndex = 0;
                    while (remainSize > 0) {
                        iowrite32(devInfo->buffer[currentIndex], startAddr);
                        remainSize--; currentIndex++;
                    }
                }
                bytesDone += bytesToTransfer;
                count -= bytesToTransfer;
            }
            break;
        }
        case DMA_DATA: {
            break;
        }
        default: {
            break;
        }
    }

    up(&devInfo->sem);
    return bytesDone;
}

static irqreturn_t ft_irq_handler(int irq, void *arg) {
    //fetch private data
    struct DevInfo_t * devInfo = (struct DevInfo_t *)arg;
    uint32_t irq_v = read_bar0_u32(devInfo,0x28);//get interrupt vector
    //check irq
    if((irq_v&0x4A)==0) {
        return IRQ_NONE;
    }
    //handle irq
    return IRQ_HANDLED;
}

int fpga_open(struct inode *inode, struct file *filePtr) {
    //Get a handle to our devInfo and store it in the file handle
    struct DevInfo_t * devInfo = 0;

    printk(KERN_INFO "[FT] fpga_open: Entering function.\n");

    devInfo = container_of(inode->i_cdev, struct DevInfo_t, cdev);

    if (down_interruptible(&devInfo->sem)) {
        printk(KERN_WARNING "[FT] fpga_open: Unable to get semaphore!\n");
        return -1;
    }

    filePtr->private_data = devInfo;

    //Record the PID of who opened the file
    //TODO: sort out where this is used
    devInfo->userPID = current->pid;

    //Return semaphore
    up(&devInfo->sem);

    printk(KERN_INFO "[FT] fpga_open: Leaving function.\n");

    return 0;
}

int fpga_close(struct inode *inode, struct file *filePtr) {

    struct DevInfo_t * devInfo = (struct DevInfo_t *)filePtr->private_data;

    if (down_interruptible(&devInfo->sem)) {
        printk(KERN_WARNING "[FT] fpga_close: Unable to get semaphore!\n");
        return -1;
    }

    //TODO: some checking of who is closing.
    up(&devInfo->sem);

    return 0;
}

//Pass-through to main dispatcher
ssize_t fpga_read(struct file *filePtr, char __user *buf, size_t count, loff_t *pos) {
    return rw_dispatcher(filePtr, buf, count, true);
}

//Pass-through to main dispatcher
ssize_t fpga_write(struct file *filePtr, const char __user *buf, size_t count, loff_t *pos) {
    return rw_dispatcher(filePtr, (char __user *) buf, count, false);
}

static int setup_chrdev(struct DevInfo_t *devInfo) {
    /*
    Setup the /dev/deviceName to allow user programs to read/write to the driver.
    */

    int devMinor = 0;
    int devMajor = 0; 
    int devNum = -1;

    int result = alloc_chrdev_region(&devInfo->cdevNum, devMinor, 1 /* one device*/, BOARD_NAME);
    if (result < 0) {
        printk(KERN_WARNING "[FT] Can't get major ID\n");
        return -1;
    }
    devMajor = MAJOR(devInfo->cdevNum);
    devNum = MKDEV(devMajor, devMinor);
    
    //Initialize and fill out the char device structure
    cdev_init(&devInfo->cdev, &fileOps);
    devInfo->cdev.owner = THIS_MODULE;
    devInfo->cdev.ops = &fileOps;
    result = cdev_add(&devInfo->cdev, devNum, 1 /* one device */);
    if (result) {
        printk(KERN_NOTICE "[FT] Error %d adding char device for FIBRE_TEST driver with major/minor %d / %d", result, devMajor, devMinor);
        return -1;
    }

    return 0;
}

static int map_bars(struct DevInfo_t *devInfo) {
    /*
    Map the device memory regions into kernel virtual address space.
    Report their sizes in devInfo.barLengths
    */
    int ct = 0;
    unsigned long barStart, barEnd, barLength;
    for (ct = 0; ct < NUM_BARS; ct++){
        printk(KERN_INFO "[FT] Trying to map BAR #%d of %d.\n", ct, NUM_BARS);
        barStart = pci_resource_start(devInfo->pciDev, ct);
        barEnd = pci_resource_end(devInfo->pciDev, ct);
        barLength = barEnd - barStart + 1;

        devInfo->barLengths[ct] = barLength;

        //Check for empty BAR
        if (!barStart || !barEnd) {
            devInfo->barLengths[ct] = 0;
            printk(KERN_INFO "[FT] Empty BAR #%d.\n", ct);
            continue;
        }

        //Check for messed up BAR
        if (barLength < 1) {
            printk(KERN_WARNING "[FT] BAR #%d length is less than 1 byte.\n", ct);
            continue;
        }

        // If we have a valid bar region then map the device memory or
        // IO region into kernel virtual address space  
        devInfo->bar[ct] = pci_iomap(devInfo->pciDev, ct, barLength);

        if (!devInfo->bar[ct]) {
            printk(KERN_WARNING "[FT] Could not map BAR #%d.\n", ct);
            return -1;
        }

        printk(KERN_INFO "[FT] BAR%d mapped at 0x%p with length %lu.\n", ct, devInfo->bar[ct], barLength);
    }
    return 0;
}  

static int unmap_bars(struct DevInfo_t * devInfo) {
    /* Release the mapped BAR memory */
    int ct = 0;
    for (ct = 0; ct < NUM_BARS; ct++) {
        if (devInfo->bar[ct]) {
            pci_iounmap(devInfo->pciDev, devInfo->bar[ct]);
            devInfo->bar[ct] = NULL;
        }
    }
    return 0;
}

static int enable_int(struct DevInfo_t * devInfo) {
    //try register irq
    int ret = request_irq(devInfo->pciDev->irq, ft_irq_handler, IRQF_SHARED,
        DRIVER_NAME, (void *)devInfo);
    //clear mask if successful
    if(ret==0) {
        write_bar0_u32(devInfo, 0x20, 0x7ffffffc);
    }
    return ret;
}

static int disable_int(struct DevInfo_t * devInfo) {
    //try unregister irq
    free_irq(devInfo->pciDev->irq, (void*)devInfo);
    return 0;
}

static int prepare_dma_buffer(struct DevInfo_t * devInfo) {
    //mask 32
    if(pci_set_dma_mask(pdev, DMA_BIT_MASK(32))==0) {
        pci_set_consistent_dma_mask(devInfo->pciDev, DMA_BIT_MASK(32));
    }
    else {
        return -1;
    }
    //dma for transfer data to device
    devInfo->dma_tx_buffer = __get_free_pages(GFP_KERNEL | __GFP_DMA | __GFP_ZERO, DMA_PAGE_NUM_T);//allocate memory
    write_bar0_u32(devInfo,0x1c,PAGE_SIZE*DMA_PAGE_NUM_T);//set size
    //dma for receive data from device
    devInfo->dma_rx_buffer = __get_free_pages(GFP_KERNEL | __GFP_DMA | __GFP_ZERO, DMA_PAGE_NUM_R);//allocate memory
    write_bar0_u32(devInfo,0x18,PAGE_SIZE*DMA_PAGE_NUM_R);//set size
    return 0;
}

static int release_dma_buffer(struct DevInfo_t * devInfo) {
    free_pages ((unsigned long) devInfo->dma_tx_buffer, DMA_PAGE_NUM_T);
    free_pages ((unsigned long) devInfo->dma_rx_buffer, DMA_PAGE_NUM_R);
}

static int probe(struct pci_dev *dev, const struct pci_device_id *id) {
    /*
    From : http://www.makelinux.net/ldd3/chp-12-sect-1
    This function is called by the PCI core when it has a struct pci_dev that it thinks this driver wants to control.
    A pointer to the struct pci_device_id that the PCI core used to make this decision is also passed to this function. 
    If the PCI driver claims the struct pci_dev that is passed to it, it should initialize the device properly and return 0. 
    If the driver does not want to claim the device, or an error occurs, it should return a negative error value.
    */

    //Initalize driver info 
    struct DevInfo_t *devInfo = 0;
    int ret = 0;

    printk(KERN_INFO "[FT] Entered driver probe function.\n");
    printk(KERN_INFO "[FT] vendor = 0x%x, device = 0x%x \n", dev->vendor, dev->device); 

    //Allocate and zero memory for devInfo

    devInfo = kzalloc(sizeof(struct DevInfo_t), GFP_KERNEL);
    if (!devInfo) {
        printk(KERN_WARNING "Couldn't allocate memory for device info!\n");
        return -1;
    }

    //Copy in the pci device info
    devInfo->pciDev = dev;

    //Save the device info itself into the pci driver
    dev_set_drvdata(&dev->dev, (void*) devInfo);

    //Setup the char device
    setup_chrdev(devInfo);    

    //Initialize other fields
    devInfo->userPID = -1;
    devInfo->buffer = (uint32_t*) kmalloc (BUFFER_SIZE * sizeof(char), GFP_KERNEL);

    //Enable the PCI
    if (pci_enable_device(dev)){
        printk(KERN_WARNING "[FT] pci_enable_device failed!\n");
        return -1;
    }

    pci_set_master(dev);
    pci_request_regions(dev, DRIVER_NAME);

    //Memory map the BAR regions into virtual memory space
    if(map_bars(devInfo)!=0) {
        printk(KERN_WARNING "[FT] mapping bars failed!\n");
        return -1;
    }
    
    //initialize send command
    write_bar0_u32(devInfo, SEND_ADDR, 0);

    //prepare DMA buffer
    if(prepare_dma_buffer(devInfo)!=0) {
        printk(KERN_WARNING "[FT] prepare DMA buffer failed!\n");
        return -1;
    }

    //enable interrupt
    if(enable_int(devInfo)!=0) {
        printk(KERN_WARNING "[FT] enable interrupt failed!\n");
        return -1;
    }

    //prepare semaphore of the driver
    sema_init(&devInfo->sem, 1);

    return 0;
}

static void remove(struct pci_dev *dev) {
    //prepare device info
    struct DevInfo_t *devInfo = 0;
    
    printk(KERN_INFO "[FT] Entered FIBRE_TEST driver remove function.\n");
    
    devInfo = (struct DevInfo_t*) dev_get_drvdata(&dev->dev);
    if (devInfo == 0) {
        printk(KERN_WARNING "[FT] remove: devInfo is 0");
        return;
    }

    //Clean up the char device
    cdev_del(&devInfo->cdev);
    unregister_chrdev_region(devInfo->cdevNum, 1);
    
    //free irq
    disable_int(devInfo);

    //release dma memory
    release_dma_buffer(devInfo);

    //Release memory
    unmap_bars(devInfo);

    //TODO: does order matter here?
    pci_clear_master(pdev);
    pci_release_regions(dev);
    pci_disable_device(dev);

    kfree((char*)devInfo->buffer);
    kfree(devInfo);

}

static int fpga_init(void) {
    printk(KERN_INFO "[FT] Loading FIBRE_TEST driver!\n");
    return pci_register_driver(&fpgaDriver);
}

static void fpga_exit(void) {
    printk(KERN_INFO "[FT] Exiting FIBRE_TEST driver!\n");
    pci_unregister_driver(&fpgaDriver);
}

module_init(fpga_init);
module_exit(fpga_exit);
