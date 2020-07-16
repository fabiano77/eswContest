CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
ROOTFS=$(CP_PATH)/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi

ARCH=arm
CC=$(CROSS_COMPILE)gcc	#C컴파일러
CXX=$(CROSS_COMPILE)g++	#C++컴파일러
OBJ = main.o car_lib.c

OBJNAME = main
$(OBJNAME): $(OBJ)
	$(CXX) -o $(OBJNAME) $(OBJ)

clean:
	rm -rf *.o $(OBJNAME)

