#!/bin/sh

name=`echo $1 | sed 's/\.tex$//' | xargs basename`
temp=`mktemp /tmp/$name.XXXXXXXX`

nkf -e $1 > $temp
/usr/bin/platex $temp

rm $temp
