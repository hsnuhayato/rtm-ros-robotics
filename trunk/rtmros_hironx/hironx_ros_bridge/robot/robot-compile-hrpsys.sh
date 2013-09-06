#!/bin/bash

function usage {
    echo >&2 "usage: $0 [hostname (default:hiro014)]"
    echo >&2 "          [-h|--help] print this message"
    exit 0
}

# command line parse
OPT=`getopt -o h -l help -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT

while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

## Comment out; not used.
#address=`host hrpsys-base.googlecode.com | awk '/^[[:alnum:].-]+ has address/ { print $4 ; exit }'` # this does not work for  Server certificate verification 

commands="
  . ~/.profile;
  echo \"* Download hrpsys *\";
  mkdir -p src;
  cd src;
  svn co http://hrpsys-base.googlecode.com/svn/trunk hrpsys-base-source;
  echo \"* Configure hrpsys *\";
  cd hrpsys-base-source;
  PKG_CONFIG_PATH=/opt/jsk/lib/pkgconfig:/usr/pkg/lib/pkgconfig CXX=QCC CC=qcc TVMET_DIR=/opt/jsk OPENRTM_IDL_DIR=/opt/jsk/include/OpenRTM-1.1/rtm/idl LDFLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_thread -lboost_filesystem\" cmake . -DLAPACK_LIBRARY_DIRS=/opt/jsk/lib -DLAPACK_INCLUDE_DIRS=/opt/jsk/include -DOPENRTM_DIR=/opt/jsk -DOPT_COLLADASUPPORT=NO -DEIGEN_INCLUDE_DIR=/opt/jsk/include -DCOMPILE_JAVA_STUFF=OFF -DCMAKE_SHARED_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_thread -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_EXE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_thread -lboost_filesystem -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_MODULE_LINKER_FLAGS=\"-L/usr/pkg/lib -lboost_system -lboost_signals -lboost_filesystem -lboost_thread -lboost_regex -lf2c -Wl,-u,MAIN__\" -DCMAKE_INSTALL_PREFIX=/opt/jsk -DBLAS_LIBRARY=/opt/jsk/lib/libblas.so -DLAPACK_LIBRARY=/opt/jsk/lib/liblapack.so -DG2C_LIBRARY=/opt/jsk/lib/libf2c.so -DCMAKE_CXX_FLAGS=\"-I/usr/pkg/include\" -DCMAKE_INSTALL_PREFIX=/opt/jsk -DENABLE_DOXYGEN=OFF ;
  echo \"* Compile hrpsys *\";
  make;
  echo \"* Install hrpsys *\";
  su -c 'make install; rm /opt/jsk/lib/libhrpIo.so';
  "

hostname=$1
hostname=${hostname:="hiro014"} 
echo "comands = $commands"
read -p "execute compile command @ $hostname (y/n)?"
if [ "$REPLY" == "y" ]; then
    ssh hiro@$hostname -t $commands
else
    echo "DO NOT RUN"
    echo "----"
    echo "$commands"
    echo "----"
    echo "EXITTING.."
fi

