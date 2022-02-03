#! /bin/bash

while true;
do
unison-2.48 -batch -auto nsra_control.prf
unison-2.48 -batch -auto nsra_viewer.prf
unison -batch -auto nsra_management.prf

sleep 5s
done
