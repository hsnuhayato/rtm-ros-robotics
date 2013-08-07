#!/bin/bash

export PATH=`rospack find openrtm_aist`/bin:$PATH
export PATH=`rospack find rtshell`/bin:$PATH
_PYTHONVER=`python -c 'import sys; print "python%s"%sys.version[:3]'`
export PYTHONPATH=`rospack find openrtm_aist_python`/lib/${_PYTHONVER}/site-packages:`rospack find rtshell`/lib/${_PYTHONVER}/site-packages:`rospack find rtctree`/lib/${_PYTHONVER}/site-packages:`rospack find rtsprofile`/lib/${_PYTHONVER}/site-packages:$PYTHONPATH
if [ -f `rospack find rtshell`/share/rtshell/shell_support ]; then
    (cd `rospack find rtshell`/share/; . ./rtshell/shell_support)
else
    echo -e "Warning : Failed to load shell_support, try rosmake openrtm"
fi

export RTCTREE_NAMESERVERS=localhost
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
