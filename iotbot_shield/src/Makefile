
CXX	 := g++
HDR    := IOTShield.h
SRC    := iotshield_test.cpp IOTShield.cpp
LIBS   := -lmraa
BINARY := iotshield_test

$(BINARY): $(SRC) $(HDR)
	$(CXX) ${SRC} $(LIBS) -o $(BINARY)
	
clean:
	rm $(BINARY)
