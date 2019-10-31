all: bme680.o main.cpp
	$(CXX) $? -o bme680.out

bme680.o: bme680.c bme680.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.out