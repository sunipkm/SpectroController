CC=gcc
CXX=g++

EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -I clkgen/include -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS)

EDLDFLAGS= -lm -lpthread -lmenu -lncurses $(LDFLAGS)

CPPOBJS=Adafruit/MotorShield.o
# MOTORSHIELDTEST=examples/motorshield.o
MOTORSHIELDTEST=main.o

LIBCLKGEN = clkgen/libclkgen.a

COBJS=i2cbus/i2cbus.o \
		gpiodev/gpiodev.o

controller: $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST) $(LIBCLKGEN)
	$(CXX) -o $@.out $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST) $(LIBCLKGEN) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

$(LIBCLKGEN):
	cd clkgen && make && make ..

.PHONY: clean doc

doc:
	doxygen .doxyconfig

clean:
	rm -vf $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST)
	rm -vf *.out

spotless: clean
	rm -vrf doc
	cd clkgen && make clean && cd ..

sync:
	rsync locsst@10.7.3.210:codes/SpectroController/* ./