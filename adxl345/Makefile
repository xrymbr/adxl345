
# 1. 使用不同的开发板内核�? 一定要修改KERN_DIR
# 2. KERN_DIR中的内核要事先配置、编�? 为了能编译内�? 要先设置下列环境变量:
# 2.1 ARCH,          比如: export ARCH=arm64
# 2.2 CROSS_COMPILE, 比如: export CROSS_COMPILE=aarch64-linux-gnu-
# 2.3 PATH,          比如: export PATH=$PATH:/home/book/100ask_roc-rk3399-pc/ToolChain-6.3.1/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin 
# 注意: 不同的开发板不同的编译器上述3个环境变量不一定相�?
#       请参考各开发板的高级用户使用手�?

KERN_DIR = /home/book/100ask_imx6ull-sdk/Linux-4.9.88

all:
	make -C $(KERN_DIR) M=`pwd` modules 
	$(CROSS_COMPILE)gcc -o adxl345test adxl345test.c


clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order adxl345test

# 参考内核源码drivers/char/ipmi/Makefile
# 要想把a.c, b.c编译成ab.ko, 可以这样指定:
# ab-y := a.o b.o
# obj-m += ab.o


adxl345drv-y := adxl345drv_1.o
obj-m += adxl345drv.o

