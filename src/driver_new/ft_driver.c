// Copyright 2017

#include "ft_driver.h"
#include "ft_macros.h"
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/delay.h>

MODULE_LICENSE("MIT");
MODULE_DESCRIPTION("Driver for PCIe Fibre-Test board.");

/* Forward Static PCI driver functions for kernel module */
static int probe(struct pci_dev *dev, const struct pci_device_id *id);
static void remove(struct pci_dev *dev);

//fileio.c functions for device
int fpga_open(struct inode *inode, struct file *file);
int fpga_close(struct inode *inode, struct file *file);
ssize_t fpga_read(struct file *file, char __user *buf, size_t count, loff_t *pos);
ssize_t fpga_write(struct file *file, const char __user *buf, size_t count, loff_t *pos);
long fpga_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);
loff_t fpga_seek(struct file *file, loff_t offset, int pos);

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
    .owner          = THIS_MODULE,
	.llseek         = fpga_seek,
    .read           = fpga_read,
    .write          = fpga_write,
    .unlocked_ioctl = fpga_ioctl,
    .open           = fpga_open,
    .release        = fpga_close
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

int wait_for_flag(int *flag, int desire_value, int single_wait_ms, int wait_max) {
    int remain = wait_max;//remaining wait cycles
    if(remain<0) {
        remain = 1;//delay at least once
    }
    //first check
    if(*flag == desire_value) {
        return remain;
    }
    //wait
    while(remain>0) {
        msleep_interruptible(single_wait_ms);//delay
        if(*flag == desire_value) {
            break;
        }
        remain--;
    }
    return remain;
}

static irqreturn_t ft_irq_handler(int irq, void *arg) {
    //fetch private data
    struct DevInfo_t * devInfo = (struct DevInfo_t *)arg;
    uint32_t irq_v = read_bar0_u32(devInfo,0x24);//get interrupt vector
    //printk(KERN_INFO "[FT] ft_irq_handler: 0x%x from irq %d.\n",irq_v,irq);
    //check irq
    if((irq_v&0x7)==0) {
        return IRQ_NONE;
    }
    write_bar0_u32(devInfo,0x24,7);//clear interrupt vector
    //handle irq
    irq_v = read_bar0_u32(devInfo,0x28);//get interrupt vector
    write_bar0_u32(devInfo,0x28,irq_v);
    //printk(KERN_INFO "[FT] ft_irq_handler: 0x%x from irq %d.\n",irq_v,irq);
    if(irq_v&0x8) {
        //dma write finished
        write_bar0_u32(devInfo,0x28,8);//clear flag
        devInfo->flag_dma_tx = 1;
        printk(KERN_INFO "[FT] DMA tx interrupt\n");
        wake_up_interruptible(&devInfo->wait_dma_tx);//wake up writing process
    }
    if(irq_v&0x2) {
        //dma read finished
        write_bar0_u32(devInfo,0x28,2);//clear flag
    }
    if(irq_v&0x40) {
        //trigger
        write_bar0_u32(devInfo,0x28,0x40);//clear flag
        if(devInfo->userPID>0) {
            //kill(devInfo->userPID,TRIGGER_SIGNAL);//send signal
        }
    }
    return IRQ_HANDLED;
}

static int enable_int(struct DevInfo_t * devInfo) {
    //try register irq
    int ret = 0;
    ret = request_irq(devInfo->pciDev->irq, ft_irq_handler, IRQF_SHARED, DRIVER_NAME, (void *)devInfo);
    if(ret==0) {
        //clear mask if successful
        write_bar0_u32(devInfo, 0x20, 0x7ffffffc);
        //prepare DMA transmit variables
        devInfo->flag_dma_tx = -1;
        devInfo->flag_dma_rx = -1;
    }
    return ret;
}

static int disable_int(struct DevInfo_t * devInfo) {
    //try unregister irq
    free_irq(devInfo->pciDev->irq, (void*)devInfo);
    return 0;
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
    devInfo->userPID = current->pid;

    //Return semaphore
    up(&devInfo->sem);

    printk(KERN_INFO "[FT] fpga_open: Leaving function.\n");

    return 0;
}

int fpga_reprobe(struct DevInfo_t *devInfo) {
    //printk(KERN_INFO "[FT] reprobe: entering.\n");

    //set pci master
    pci_set_master(devInfo->pciDev);
    
    //initialize send command
    write_bar0_u32(devInfo, SEND_ADDR, 0);

    //mask 32
    if(pci_set_dma_mask(devInfo->pciDev, DMA_BIT_MASK(32))==0) {
        pci_set_consistent_dma_mask(devInfo->pciDev, DMA_BIT_MASK(32));
    }
    else {
        return -1;
    }

    //dma size
    write_bar0_u32(devInfo,0x1c,PAGE_SIZE*DMA_PAGE_NUM_T);//set size
    write_bar0_u32(devInfo,0x18,PAGE_SIZE*DMA_PAGE_NUM_R);//set size

    //enable interrupt
    if(enable_int(devInfo)!=0) {
        printk(KERN_WARNING "[FT] enable interrupt failed!\n");
        return -1;
    }
    //write_bar0_u32(devInfo, 0x20, 0x7ffffffe);
    write_bar0_u32(devInfo, 0x28, 0x4A);//clear flag

    //clear rx semaphore
    while (down_trylock(&devInfo->sem_dma_rx)==0) {}
    devInfo->dma_rx_tail = devInfo->dma_rx_head;

    return 0;
}

//reset FPGA
int fpga_reset(struct DevInfo_t *devInfo) {
    printk(KERN_INFO "[FT] fpga_reset: Entering function.\n");

    //lock driver
    if (down_interruptible(&devInfo->sem)) {
        printk(KERN_WARNING "[FT] fpga_reset: Unable to get semaphore!\n");
        return -1;
    }

    //reset DMA mappings
    //printk(KERN_WARNING "[FT] flags: %d %d\n",devInfo->flag_dma_rx,devInfo->flag_dma_tx);
    if(devInfo->flag_dma_rx>=0) {
        devInfo->flag_stop = 1;
        if(wait_for_flag(&devInfo->flag_stop, 2, 1, 1000)==0) {
            printk(KERN_WARNING "[FT] fpga_reset: Failed to stop receiving!\n");
            pci_unmap_single(devInfo->pciDev, devInfo->dma_rx_mem, DMA_PAGE_NUM_R*PAGE_SIZE, PCI_DMA_FROMDEVICE);
        }
        devInfo->flag_dma_rx = -1;
    }
    if(devInfo->flag_dma_tx>=0) {
        pci_unmap_single(devInfo->pciDev, devInfo->dma_tx_mem, DMA_PAGE_NUM_T*PAGE_SIZE, PCI_DMA_TODEVICE);
        devInfo->flag_dma_tx = -1;
    }

    //free irq first
    disable_int(devInfo);

    //reset FPGA
    write_bar0_u32(devInfo, 0x2C, 1);
    msleep(1);
    write_bar0_u32(devInfo, 0x2C, 0);
    msleep(1);

    //re-probe
    if (fpga_reprobe(devInfo)!=0) {
        printk(KERN_WARNING "[FT] fpga_reset: Unable to reprobe device!\n");
        return -1;
    }

    //unlock file
    up(&devInfo->sem);

    printk(KERN_INFO "[FT] fpga_reset: Leaving function.\n");
    return 0;
}

int fpga_close(struct inode *inode, struct file *filePtr) {

    struct DevInfo_t * devInfo = 0;
    devInfo = (struct DevInfo_t *)filePtr->private_data;
    printk(KERN_INFO "[FT] fpga_close: Entering function.\n");

    if (down_interruptible(&devInfo->sem)) {
        printk(KERN_WARNING "[FT] fpga_close: Unable to get semaphore!\n");
        return -1;
    }

    //some checking of who is closing.
    if(current->pid==devInfo->userPID) {
        devInfo->userPID = -1;
    }

    up(&devInfo->sem);
    printk(KERN_INFO "[FT] fpga_close: Entering function.\n");

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

//control
long fpga_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg) {
    //fetch private data
    struct DevInfo_t * devInfo = 0;
    struct Bar0Cmd_t bar0_cmd;
    uint32_t tmp;
    //printk(KERN_INFO "[FT] fpga_ioctl: Entering function with command %u.\n",cmd);
    devInfo = (struct DevInfo_t *)filePtr->private_data;
    //consider command
    if(cmd==FT_RESET) {
        //reset FPGA
        return fpga_reset(devInfo);
    }
    else if(cmd==FT_READ_BAR0_U32) {
        //read bar0
        copy_from_user(&bar0_cmd, (void __user *) arg, sizeof(struct Bar0Cmd_t));//fetch command
        tmp = read_bar0_u32(devInfo,bar0_cmd.addr);//read
        copy_to_user(bar0_cmd.value, (char*)&tmp, sizeof(uint32_t));//copy to user
        //printk(KERN_INFO "[FT] fpga_ioctl: read %u from %u.\n", tmp, bar0_cmd.addr);
        return 0;
    }
    else if(cmd==FT_WRITE_BAR0_U32) {
        //write bar0
        copy_from_user(&bar0_cmd, (void __user *) arg, sizeof(struct Bar0Cmd_t));//fetch command
        copy_from_user((char*)&tmp, bar0_cmd.value, sizeof(uint32_t));//copy from user
        write_bar0_u32(devInfo,bar0_cmd.addr,tmp);//write
        //printk(KERN_INFO "[FT] fpga_ioctl: write %u to %u.\n", tmp, bar0_cmd.addr);
        return 0;
    }
    else if(cmd==FT_STOP_DMA_RX) {
        //stop
        if(devInfo->flag_dma_rx>=0) {
            devInfo->flag_stop = 1;
            if(wait_for_flag(&devInfo->flag_stop, 2, 1, 1000)==0) {
                printk(KERN_WARNING "[FT] fpga_ioctl: Failed to stop receiving!\n");
                pci_unmap_single(devInfo->pciDev, devInfo->dma_rx_mem, DMA_PAGE_NUM_R*PAGE_SIZE, PCI_DMA_FROMDEVICE);
            }
            devInfo->flag_dma_rx = -1;
        }
        return 0;
    }
    //invalid command
    printk(KERN_WARNING "[FT] fpga_ioctl: Unknown command %u!\n", cmd);
    return -1;
}

//seek
loff_t fpga_seek(struct file *file, loff_t offset, int pos) {
    //seek 0,0 for reset
    return offset;
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

static int prepare_dma_buffer(struct DevInfo_t * devInfo) {
    int i = 0;
    //mask 32
    if(pci_set_dma_mask(devInfo->pciDev, DMA_BIT_MASK(32))==0) {
        pci_set_consistent_dma_mask(devInfo->pciDev, DMA_BIT_MASK(32));
    }
    else {
        return -1;
    }
    //dma for transfer data to device
    printk(KERN_INFO "[FT] DMA transmit buffer size: %u.\n", PAGE_SIZE*DMA_PAGE_NUM_T);
    devInfo->dma_tx_buffer = (char*)__get_free_pages(GFP_KERNEL | __GFP_DMA | __GFP_ZERO, DMA_PAGE_ODR_T);//allocate memory
    write_bar0_u32(devInfo,0x1c,PAGE_SIZE*DMA_PAGE_NUM_T);//set size
    //dma for receive data from device
    printk(KERN_INFO "[FT] DMA receive buffer size: %u.\n", PAGE_SIZE*DMA_PAGE_NUM_R);
    for(i=0;i<DMA_BUFFER_NUM_R;i++) {
        devInfo->dma_rx_buffer[i] = (char*)__get_free_pages(GFP_KERNEL | __GFP_DMA | __GFP_ZERO, DMA_PAGE_ODR_R);//allocate memory
    }
    devInfo->dma_rx_head = 0; devInfo->dma_rx_tail = 0;
    write_bar0_u32(devInfo,0x18,PAGE_SIZE*DMA_PAGE_NUM_R);//set size
    return 0;
}

static int release_dma_buffer(struct DevInfo_t * devInfo) {
    int i = 0;
    free_pages ((unsigned long) devInfo->dma_tx_buffer, DMA_PAGE_ODR_T);
    for(i=0;i<DMA_BUFFER_NUM_R;i++) {
        free_pages ((unsigned long) devInfo->dma_rx_buffer[i], DMA_PAGE_ODR_R);
    }
    return 0;
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
    uint32_t status = 0;

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

    //status
    status = read_bar0_u32(devInfo, 0x3C);
    printk(KERN_INFO "[FT] Status: 0x%x\n", status);

    //prepare DMA buffer
    if(prepare_dma_buffer(devInfo)!=0) {
        printk(KERN_WARNING "[FT] prepare DMA buffer failed!\n");
        return -1;
    }

    //prepare wait queue
    init_waitqueue_head (&devInfo->wait_dma_tx);
    //prepare semaphore
    sema_init (&devInfo->sem_dma_rx, 0);

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
    printk(KERN_INFO "[FT] cleanup char device.\n");
    cdev_del(&devInfo->cdev);
    unregister_chrdev_region(devInfo->cdevNum, 1);
    
    //free irq
    printk(KERN_INFO "[FT] Disable interrupt.\n");
    disable_int(devInfo);

    //release dma memory
    printk(KERN_INFO "[FT] Release DMA buffer.\n");
    release_dma_buffer(devInfo);

    //Release memory
    printk(KERN_INFO "[FT] Unmap bars.\n");
    unmap_bars(devInfo);

    //TODO: does order matter here?
    printk(KERN_INFO "[FT] Clear master.\n");
    pci_clear_master(dev);
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
