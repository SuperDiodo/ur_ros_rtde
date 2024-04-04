#!/bin/bash

if [ $# -ne 2 ]
then
	echo "Wrong number of args supplied, please provide:\n <a> name of input xacro file\n <b> name of output urdf file \nExample of usage: sh generate_urdf.sh <robot.xacro> <robot.urdf>"
	exit
fi

extension_1=${1##*.}
if [ "$extension_1" != "xacro" ]
then
	echo "arg <$1> must be a .xacro file"
	exit
fi

if [ ! -f "$1" ]	
then
	echo "Cannot open file <$1>, it the relative path correct?"
	echo "xacro files in current directory:"
	for entry in "$(pwd)"/*
	do
	  	if [ "${entry##*.}" = "xacro" ]
		then
			echo "\t$entry"
			exit
		fi
	done
	exit
fi

extension_2=${2##*.}
if [ "$extension_2" != "urdf" ]
then
	echo "arg <$2> must be an .urdf file"
	exit
fi

xacro $1 > $2
