#!/bin/bash

length=`ls -la $1|awk '{print $5}'`
b=$((65536-$length%65536))
tr '\000' '\377' < /dev/zero | dd of=allones bs=1 count=$b

cat $1 allones > $2

rm -f allones




