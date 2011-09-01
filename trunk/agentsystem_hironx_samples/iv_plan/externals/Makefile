SUBDIRS = ikfast PQP_v1.3

#ifeq ($(USER), leus)
#	SSH_USER = eus
#else
#	SSH_USER = $(USER)
#endif

ARCHIVEDIR = http://rtm-ros-robotics.googlecode.com/svn/wiki

PQP = PQP_v1.3.tgz

# compile source
#VPYTHON = visual-5.32_release.patched101113.tgz

# binary download
ARCH=$(shell uname -m)
ifeq ($(ARCH),x86_64)
VPYTHON = visual-5.71_release.patched.x86_64.tgz
else
VPYTHON = visual-5.71_release.patched.x86.tgz
endif


.PHONY: all $(SUBDIRS)

all: download untar $(SUBDIRS) 

download: pqp-download vpython-download
pqp-download:
	$(if $(shell find ./ -name PQP_v1.3.tgz), $(donothing), $(pqp-download))
vpython-download:
	$(if $(shell find ./ -name visual-5.32_release.patched.x86.tgz), $(donothing), $(vpython-download))

define pqp-download
	wget $(ARCHIVEDIR)/$(PQP)
endef
define vpython-download
	wget $(ARCHIVEDIR)/$(VPYTHON)
endef

untar: pqp-untar vpython-untar
pqp-untar:
	$(if $(shell find ./ -name PQP_v1.3), $(donothing), $(pqp-untar))
vpython-untar:
	$(if $(shell find ./ -name visual), $(donothing), $(vpython-untar))
define pqp-untar
	tar xvfz $(PQP)
endef
define vpython-untar
	tar xvfz $(VPYTHON)
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
	done
