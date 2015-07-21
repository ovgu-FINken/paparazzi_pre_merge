#!/bin/bash
while [ true ] ; do
	./send_to_ivy.sh start.txt
	sleep 30
	./send_to_ivy.sh land.txt
	sleep 45
done
