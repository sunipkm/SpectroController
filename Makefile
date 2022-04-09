CC=gcc
CXX=g++

EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -I./include -I clkgen/include -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS)

EDLDFLAGS= -lm -lpthread -lmenu -lncurses $(LDFLAGS)

CPPOBJS=Adafruit/MotorShield.o \
		src/iomotor.o \
		src/scanmotor.o

MOTORSHIELDTEST=src/main.o

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

install:
	cp controller.out /usr/local/bin/controller
	cp posinfo.bin /usr/local/bin/posinfo.bin

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