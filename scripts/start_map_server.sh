#!/bin/bash

while :
do
    echo "Waiting for map_server..."
    ros2 lifecycle list /map_server
    if [ $? -eq 0 ]; then
        break
    fi
    sleep 0.5
done

while :
do
    echo "Trying to configure map_server..."
    ros2 lifecycle set /map_server configure
    if [ $? -eq 0 ]; then
        break
    fi
    sleep 0.5
done

while :
do
    echo "Trying to activate map_server..."
    ros2 lifecycle set /map_server activate
    if [ $? -eq 0 ]; then
        break
    fi
    sleep 0.5
done
