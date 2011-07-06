IDLC = `rtm-config --idlc`
IDL_INCDIR = /usr/local/include/rtm/idl
IDLS = JointDataTypes.idl RobotHardwareService.idl \
	SequencePlayerService.idl VideoStreamService.idl \
	ArmMotion.idl GraspController.idl

all: idlcompile

idlcompile: $(IDLS)
	omniidl -bpython -I$(IDL_INCDIR) $^

clean:
	rm -rf *_idl.py *.pyc _GlobalIDL _GlobalIDL__POA OpenHRP OpenHRP__POA RTC RTC__POA
