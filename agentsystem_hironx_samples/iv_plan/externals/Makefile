# -*- Makefile -*-

SUBDIRS = ikfast PQP_v1.3

#ifeq ($(USER), leus)
#	SSH_USER = eus
#else
#	SSH_USER = $(USER)
#endif

ARCHIVE_DIR = http://rtm-ros-robotics.googlecode.com/svn/wiki

PQP_ACV = PQP_v1.3.tgz

# VPYTHON_SRC = visual-5.32_release.patched101113.tgz

# compiled binary archives
ARCH=$(shell uname -m)
ifeq ($(ARCH),x86_64)
VPYTHON_ACV = visual-5.32_release.patched.x86_64.tgz
else
VPYTHON_ACV = visual-5.32_release.patched.x86.tgz
endif


.PHONY: all $(SUBDIRS)

all: download untar $(SUBDIRS) 

download: pqp-download vpython-download
pqp-download:
	$(if $(shell find ./ -name $(PQP_ACV)), $(donothing), $(pqp-download))
vpython-download:
	$(if $(shell find ./ -name $(VPYTHON_ACV)), $(donothing), $(vpython-download))

define pqp-download
	wget $(ARCHIVE_DIR)/$(PQP_ACV)
endef
define vpython-download
	wget $(ARCHIVE_DIR)/$(VPYTHON_ACV)
endef

untar: pqp-untar vpython-untar
pqp-untar:
	$(if $(shell find ./ -maxdepth 1 -name PQP_v1.3), $(donothing), $(pqp-untar))
vpython-untar:
	$(if $(shell find ./ -maxdepth 1 -name visual), $(donothing), $(vpython-untar))
define pqp-untar
	tar xvfz $(PQP_ACV)
endef
define vpython-untar
	tar xvfz $(VPYTHON_ACV)
endef

define do-nothing
endef

$(SUBDIRS):
	$(MAKE) -C $@

vpython-src:
	(cd "visual-5.32_release.patched101113" && ./configure && make)

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean ;\
	done; \
	rm -rf visual PQP_v1.3

dist-clean: clean
	rm -f PQP_ACV VPYTHON_ACV
