#!/bin/sh
# if RAILBOT_WS is not set, bail
if [ -z "$RAILBOT_WS" ]; then
	echo "RAILBOT_WS is not set. Please set it in your ~/.profile or whatever file is suitable for your environment."
	return 1
fi

# if $RAILBOT_WS/install does not exist, bail
if [ ! -d "$RAILBOT_WS/install" ]; then
	echo "RAILBOT_WS/install does not exist. Please build the workspace first."
	return 1
fi

micromamba activate railbot

# if shell is bash, source the .bash file
if [ -n "$BASH_VERSION" ]; then
	. $RAILBOT_WS/install/local_setup.bash
# if shell is zsh, source the .zsh file
elif [ -n "$ZSH_VERSION" ]; then
	. $RAILBOT_WS/install/local_setup.zsh
# otherwise sh
else
	. $RAILBOT_WS/install/local_setup.sh
fi
