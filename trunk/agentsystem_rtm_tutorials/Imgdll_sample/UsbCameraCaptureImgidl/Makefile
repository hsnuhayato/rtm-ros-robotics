# -*- Makefile -*-
IDLC     = `rtm-config --idlc`
IDLFLAGS = `rtm-config --idlflags` -I`rtm-config --prefix`/include/rtm/idl

all: Img_idl.py

Img_idl.py: Img.idl
	$(IDLC) -bpython $(IDLFLAGS) $<

clean: 
	rm -f *~
	rm -f *.log *.pyc
	rm -f *.cc *.hh
	rm -rf Img_idl.py Img__POA Img
# end of Makefile

