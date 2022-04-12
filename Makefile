CC=gcc
CXX=g++

INSTALLDIR=/usr/local/monochromatord
LOGDIR=/var/log/monochromatord

EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -I./include -I clkgen/include -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS) -DINSTALL_DIR=\"$(INSTALLDIR)\" -DLOG_FILE_DIR=\"$(LOGDIR)\"

EDLDFLAGS= -lm -lpthread -lmenu -lncurses $(LDFLAGS)

CPPOBJS=Adafruit/MotorShield.o \
		src/iomotor.o \
		src/scanmotor.o

GUIMAIN=src/main.o

LIBCLKGEN = clkgen/libclkgen.a

COBJS=i2cbus/i2cbus.o \
		gpiodev/gpiodev.o

controller: $(COBJS) $(CPPOBJS) $(GUIMAIN) $(LIBCLKGEN)
	$(CXX) -o $@.out $(COBJS) $(CPPOBJS) $(GUIMAIN) $(LIBCLKGEN) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

$(LIBCLKGEN):
	cd clkgen && make && make ..

install:
	mkdir -p $(INSTALLDIR)
	cp controller.out $(INSTALLDIR)/controller
	cp posinfo.bin $(INSTALLDIR)/posinfo.bin | true
	ln -s $(INSTALLDIR)/controller /usr/local/bin/controller

uninstall:
	rm -f /usr/local/bin/controller

.PHONY: clean doc

doc:
	doxygen .doxyconfig

clean:
	rm -vf $(COBJS) $(CPPOBJS) $(GUIMAIN)
	rm -vf *.out

spotless: clean
	rm -vrf doc
	cd clkgen && make clean && cd ..

sync:
	rsync locsst@10.7.3.210:codes/SpectroController/* ./