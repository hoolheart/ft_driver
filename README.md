ft_driver
=========

Driver for fibre-test PCIe board.

## Quick Start

The driver is currently written for the Fibre-Test PCIe board based on Xilix FPGA. 


## Building driver

- Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
- Move into the driver folder and compile the driver: ```cd src/driver; make```
- Run the load_driver script as root: ```sudo load_driver.sh```

## Building library

- Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
- Move into the library folder and compile the driver: ```cd src/ft_card_lib; make```

## Use pre-built binaries

**Note** all pre-built binaries are built on CentOS 7 64bit platform.

- Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
- Move into the release driver folder: ```cd release/driver```
- Run the load_driver script as root: ```sudo load_driver.sh```
- The pre-built library and its header file can be found in folder: ```release/ft_card_lib```

## Test project

### Test for registers

- Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
- Move into the test folder and compile the test project: ```cd test/register; make```
- Run test: ```./test_ft1```

### Test for use of library

- Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
- Move into the test folder and compile the test project: ```cd test/test_lib; make```
- Run the script: ```./run_test.sh```

