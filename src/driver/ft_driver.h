// Copyright 2017
// Original Author: Huide Zhou <prettyage.new@gmail.com>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/signal.h>

#define VENDOR_ID 0x10ee
#define DEVICE_ID 0x7
#define NUM_BARS 1

/* Maximum size of driver buffer (allocated with kalloc()).
 * Needed to copy data from user to kernel space */
static const size_t BUFFER_SIZE    = PAGE_SIZE;
#define DMA_PAGE_ODR_T 1
static const size_t DMA_PAGE_NUM_T = 1<<DMA_PAGE_ODR_T;
#define DMA_PAGE_ODR_R 5
static const size_t DMA_PAGE_NUM_R = 1<<DMA_PAGE_ODR_R;
#define DMA_BUFFER_NUM_R 4

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
  char *dma_tx_buffer, *dma_rx_buffer[DMA_BUFFER_NUM_R];
  int dma_rx_head, dma_rx_tail;
  dma_addr_t dma_tx_mem, dma_rx_mem;
  
  /* Mutex for this device. */
  struct semaphore sem;

  /* handle of DMA transmit interrupt */
  wait_queue_head_t wait_dma_tx, wait_dma_rx;
  int flag_dma_tx, flag_dma_rx;
  
  /* PID of process that called open() */
  int userPID;
  
  /* character device */
  dev_t cdevNum;
  struct cdev cdev;
  struct class *myClass;
  struct device *device;

};
