CC=g++

HEADER = include/defines.h 
CFLAGS = -Wall -Iinclude

build: Simulation/sim.cpp $(HEADER)
	$(CC) $(CFLAGS) -o Simulation/sim Simulation/sim.cpp

clean: 
	rm -rf Client/*o Server/*o Client/collect_data Server/server
