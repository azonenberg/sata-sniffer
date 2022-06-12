CFLAGS=-g -mcpu=cortex-m7 -DSTM32H735 -O2
CXXFLAGS=$(CFLAGS) --std=c++17 -fno-exceptions -fno-rtti -g --specs=nano.specs \
	-I../stm32-cpp/devices/inc/ \
	-I../stm32-cpp/src/ \
	-I../../ \
	-Iconfig
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
all:
	$(CXX) *.cpp -c $(CXXFLAGS)
	$(CC) ../stm32-cpp/src/cpu/*.S -c $(CFLAGS)
#	$(CXX) ../../microkvs/kvs/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../microkvs/driver/STM32*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../embedded-cli/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/cli/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/contrib/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/crypt/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/drivers/base/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/drivers/stm32/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/net/arp/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/net/icmpv4/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/net/ipv4/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/net/tcp/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/net/ethernet/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../../staticnet/ssh/*.cpp -c $(CXXFLAGS)
#	$(CXX) ../stm32-cpp/src/cli/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/src/newlib-stubs/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/src/peripheral/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/src/util/*.cpp -c $(CXXFLAGS)
	$(CXX) ../stm32-cpp/devices/src/stm32h735.cpp -c $(CXXFLAGS)
	$(CXX) $(CXXFLAGS) *.o -Wl,-T ../stm32-cpp/devices/link/stm32h735.ld -o firmware.elf
	arm-none-eabi-objcopy -O binary --only-section=.text --only-section=.data firmware.elf firmware.bin
	./imagesize.sh
