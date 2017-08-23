ft_driver
=========

Driver for fibre-test PCIe board.  

## Quick Start

The driver is currently written for the Fibre-Test PCIe board based on Xilix FPGA. 


## Driver Software

1. Clone/download the repository into some convenient location.  
2. Open a shell prompt and move into the repo. directory: ```cd /path/to/repo```
3. Move into the driver folder and compile the driver: ```cd src/linux; make```
4. Run the load_driver script as root: ```sudo load_driver.sh```
5. Move back into the test directory.  Compile the test program with: ```g++ -std=c++11 test.cpp -o test.out``` and then run the executable for some simple tests.


 
