// Copyright 2017

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
#define NUM_CHLS 4

/* size to allocate for DMA */
#define DMA_PAGE_ODR_T 1
static const size_t DMA_PAGE_NUM_T = 1<<DMA_PAGE_ODR_T;
#define DMA_PAGE_ODR_R 5
static const size_t DMA_PAGE_NUM_R = 1<<DMA_PAGE_ODR_R;
#define DMA_BUFFER_NUM_T 16
#define DMA_BUFFER_NUM_R 16

/** buffer of one channel */
struct channel_buffer {
    char *buffer;
    uint32_t chl;
};

//Keep track of bits and bobs that we need for the driver
struct DevInfo_t {
  /* the kernel pci device data structure */
  struct pci_dev *pciDev;
  
  /* upstream root node */
  struct pci_dev *upstream;
  
  /* Mutex for this device. */
  struct semaphore sem;
  
  /* kernel's virtual addr. for the mapped BARs */
  void * __iomem bar[NUM_BARS];
  
  /* length of each memory region. Used for error checking. */
  size_t barLengths[NUM_BARS];

  /* size of buffers */
  size_t dma_tx_size, dma_rx_size, tx_buffer_size, rx_buffer_size;

  /* DMA buffer. If allocated, will be DMA_PAGE_NUM*PAGE_SIZE. */
  char *dma_tx_buffer, *dma_rx_buffer;
  dma_addr_t dma_tx_mem, dma_rx_mem;

  /* handle of DMA transmit interrupt */
  wait_queue_head_t wait_dma_tx;
  struct semaphore sem_dma_rx;
  int flag_dma_tx, flag_dma_rx;
  int flag_stop;

  /* buffers for receive */
  struct channel_buffer rx_buffer[DMA_BUFFER_NUM_R];
  int rx_push, rx_pull;
  int rx_buf_cnt;
  
  /* PID of process that called open() */
  int userPID;

  /* current using channel */
  uint32_t current_chl;
  
  /* character device */
  dev_t cdevNum;
  struct cdev cdev;
  struct class *ft_class;
  struct device *ft_device[NUM_CHLS];
};
