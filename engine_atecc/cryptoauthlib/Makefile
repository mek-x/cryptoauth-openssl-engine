CWD:=	$(shell pwd)
CFLAGS= -I. -I../.. -fPIC -g -O0
SRC:=	$(wildcard *.c)
CC=             gcc
HAL?=   -DATCA_HAL_KIT_CDC

DIRECTORIES=	$(shell find . -maxdepth 1 ! -path . -type d)
SUBDIRS=	$(DIRECTORIES)

.PHONY:	tgt_lib tgt_test clean

MODULES=	$(patsubst %.c,%.o,$(SRC))

all:	tgt_lib tgt_test $(MODULES) 

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

tgt_lib:
	make -w -C lib HAL='$(HAL)'

tgt_test:
	make -w -C test HAL='$(HAL)'

clean:
	rm -f *.o *.a
	$(foreach subdir,$(basename $(SUBDIRS)),$(MAKE) -w -C $(subdir) clean;)


