#!/bin/bash

sleep 5

roslaunch m100_simulation spawn_quadrotor.launch x:=-2.0 y:=-20.0 z:=0.3

echo "Ready to go!"