#!/bin/bash

if [ "$1" != "" ]; then
  echo "Full path to the file:"
  echo "$PWD/$1"
else 
  echo "$1: main model not specified"
  exit -1
fi

if [ "$2" != "" ]; then
  echo "Full path to the file:"
  echo "$PWD/$2"

  roslaunch squirrel_ser ser.launch model:="$PWD/$1" elm_model:="$PWD/$2" 
else 
  echo "$2: elm model not specified"
  exit -2
fi
