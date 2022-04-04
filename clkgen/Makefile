CC := gcc
INCLUDE_DIR = -I include/

OBJECTS = src/clkgen.o \
		  src/timer_gen.o

TESTOBJ = src/test.o
TESTPROG = test.out

LIBOBJ := libclkgen.a

LDFLAGS := -L./

ARCH=UNDEFINED
ifeq ($(ARCH),UNDEFINED)
	ARCH := $(shell uname -m)
endif

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X $(ARCH)"
	CC := clang
	CFLAGS := -arch $(ARCH) -Wall -O2
endif

ifeq ($(UNAME_S), Linux) #Linux
	ECHO_MESSAGE = "Linux"
	CFLAGS := -Wall -O2 -std=gnu11
endif

LIBS := -lclkgen -lpthread

all : $(LIBOBJ)
	@echo "Built library $(LIBOBJ) for $(ECHO_MESSAGE)"

test: $(TESTOBJ) $(LIBOBJ)
	@echo "Built for $(ECHO_MESSAGE), execute ./$(TESTPROG)"
	@$(CC) $(TESTOBJ) $(LDFLAGS) -o $(TESTPROG) $(LIBS)

$(LIBOBJ): $(OBJECTS)
	@ar -crus $(LIBOBJ) $(OBJECTS)

%.o : %.c
	@$(CC) -c $(CFLAGS) $(INCLUDE_DIR) -o $@ -c $<

.PHONY: clean

clean:
	rm -rf $(OBJECTS) $(TESTOBJ) $(TESTPROG) $(LIBOBJ)

