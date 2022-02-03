#! /bin/bash

while true;
do
unison-2.48 -batch -auto nsra_control.prf
unison-2.48 -batch -auto nsra_viewer.prf

sleep 5s
done
