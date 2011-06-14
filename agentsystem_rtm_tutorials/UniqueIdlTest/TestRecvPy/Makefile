# -*- Makefile -*-
IDLC     = `rtm-config --idlc`
IDLFLAGS = `rtm-config --idlflags` -I`rtm-config --prefix`/include/rtm/idl
TARGET = idltest

all: $(TARGET)_idl.py

$(TARGET)_idl.py: $(TARGET).idl
	$(IDLC) -bpython $(IDLFLAGS) $<

clean: 
	rm -f *~
	rm -f *.log
	rm -f *.cc *.hh
	rm -rf $(TARGET)_idl.py $(TARGET)_idl.pyc Idltest__POA Idltest
# end of Makefile

