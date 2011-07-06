import os
import sys

# to be replaced with RTM tools
import roslib; roslib.load_manifest('iv_plan')

try:
    pkgdir = os.environ['IV_PLAN_ROOT']
except:
    pkgdir = '..'

sys.path.append(pkgdir+'/lib')

# for patched python visual
# shared install
# sys.path.append('/usr/local/lib/python2.6/site-packages/')
# user install
sys.path.append(pkgdir+'/externals/visual/site-packages/')

# sys.path.append(pkgdir+'/externals/visual/site-packages/')
