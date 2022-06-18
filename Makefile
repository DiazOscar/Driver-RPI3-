KERNEL:= $(shell uname -r)
KERNEL_SRC:=/lib/modules/${KERNEL}/build

red:=$(shell tput setaf 1)
green:=$(shell tput setaf 2)
reset:=$(shell tput sgr0)

ifndef MODULE
$(error $(red)variable MODULE no definida, usar: $(reset) MODULE=<nombre> make)
endif

$(info $(green)Usando kernel: ${KERNEL}$(reset))
$(info $(green)Variable MODULE = ${MODULE}$(reset))

obj-m += ${MODULE}.o

.PHONY : compile install uninstall
 
install: uninstall compile
	@tput setaf 1
	@echo "instalando modulo..."
	@tput sgr0
	sudo insmod ${MODULE}.ko 
	dmesg | tail 
	#sudo chmod go+rw /dev/leds

compile:
	@tput setaf 1
	@echo "compilando modulo..."
	@tput sgr0
	make -C ${KERNEL_SRC} M=${CURDIR} modules
	
uninstall:
	@if lsmod | grep ${MODULE} ; then\
	  tput setaf 1;\
	  echo "desistalando modulo..."; \
	  tput sgr0;\
	  sudo rmmod ${MODULE}; \
	  dmesg | tail -4 ;\
	fi
