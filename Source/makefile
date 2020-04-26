CXX ?= g++
NAVIO = ../Navio
INCLUDES = -I ../../Navio
PIGPIO = -L$(PIGPIO_PATH)
INCLUDES = -I ../../Navio -I$(PIGPIO_PATH)
all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=gnu++11 $(INCLUDES) $(LIB) main.cpp macro.h onboard.h stateDerivative.h -L$(NAVIO)
	navio -o main -o 

clean:
	rm -f main
