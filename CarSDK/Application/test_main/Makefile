CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

ARCH=arm
CC=$(CROSS_COMPILE)gcc
OBJ = main.o car_lib.c

OBJNAME = main
$(OBJNAME): $(OBJ)
	$(CC) -o $(OBJNAME) $(OBJ)

clean:
	rm -rf *.o $(OBJNAME)

