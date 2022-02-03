#! /bin/bash

while true;
do
unison -batch -auto -silent nsra_management.prf
unison-2.48 -batch -auto -silent nsra_control.prf
unison-2.48 -batch -auto -silent nsra_viewer.prf

sleep 5s
done
