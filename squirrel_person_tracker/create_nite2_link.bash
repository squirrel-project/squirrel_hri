#!/bin/bash
if [ $# != 1 ]; then
	echo "Usage: $0 NITE_DIR" >&2
	exit -1
fi
NITE_DIR=$1
echo NiTE_DIR = $NITE_DIR
LINK_NAME=$HOME/.ros/NiTE2
echo LINK_NAME = $LINK_NAME
if [ -d $LINK_NAME -o -h $LINK_NAME ]; then
echo "Link $LINK_NAME to $NITE_DIR already exists."
else
ln -s $NITE_DIR $LINK_NAME
echo "Created link $LINK_NAME to $NITE_DIR."
fi
