// Copyright 2017
// Original Author: Huide Zhou (prettyage.new@gmail.com)

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sched.h>


#define VENDOR_ID 0x10ee
#define DEVICE_ID 0x7

#define DRIVER_NAME "FIBRE_TEST"
#define BOARD_NAME "pcie_ft1"
#define NUM_BARS 1  //we use up to BAR0


//fileio.c functions for device
int fpga_open(struct inode *inode, struct file *file);
int fpga_close(struct inode *inode, struct file *file);
ssize_t fpga_read(struct file *file, char __user *buf, size_t count, loff_t *pos);
ssize_t fpga_write(struct file *file, const char __user *buf, size_t count, loff_t *pos);


/* Maximum size of driver buffer (allocated with kalloc()).
 * Needed to copy data from user to kernel space */
static const size_t BUFFER_SIZE    = PAGE_SIZE;
static const size_t DMA_PAGE_NUM_T = 2;
static const size_t DMA_PAGE_NUM_R = 1;

//Keep track of bits and bobs that we need for the driver
struct DevInfo_t {
  /* the kernel pci device data structure */
  struct pci_dev *pciDev;
  
  /* upstream root node */
  struct pci_dev *upstream;
  
  /* kernel's virtual addr. for the mapped BARs */
  void * __iomem bar[NUM_BARS];
  
  /* length of each memory region. Used for error checking. */
  size_t barLengths[NUM_BARS];

  /* temporary buffer. If allocated, will be BUFFER_SIZE. */
  uint32_t *buffer;

  /* DMA buffer. If allocated, will be DMA_PAGE_NUM*PAGE_SIZE. */
  char *dma_tx_buffer, *dma_rx_buffer;
  dma_addr_t dma_tx_mem, dam_rx_mem;
  
  /* Mutex for this device. */
  struct semaphore sem;

  /* semaphores for interrupt */
  struct semaphore sem_dma_tx,sem_dma_rx,sem_trigger;
  
  /* PID of process that called open() */
  int userPID;
  
  /* character device */
  dev_t cdevNum;
  struct cdev cdev;
  struct class *myClass;
  struct device *device;

};
