#!/usr/bin/env python

import sys
from omniORB import CORBA
import Hello

orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

ior = sys.argv[1]
obj = orb.string_to_object(ior)

hw = obj._narrow(Hello.World)

if hw is None:
    print "Object reference is not an Hello::World"
    sys.exit(1)

result  = hw.hello()

print "The object said '%s'." % (result)
