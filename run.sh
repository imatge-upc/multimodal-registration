#!/bin/bash
xhost + > /dev/null
XSOCK=/tmp/.X11-unix
CONTAININGDIR=$(pwd)
DATASETSDIR='/opt/Datasets'
RESULTSDIR='/opt/Sources/PhD/results'

for env_var in ${!PHD*}; do
	ENV_VARS_CALL+=" -e $env_var" ; 
	ENV_VARS_SHOW+=" $env_var=`eval echo \\$$env_var`"
done

echo 'Running PhD container with : ' $*
echo 'Environment variables passed: ' $ENV_VARS_SHOW


docker run -it \
	-v $XSOCK:$XSOCK \
	-v $CONTAININGDIR:/src \
	-v $DATASETSDIR:/datasets \
	-v $RESULTSDIR:/results \
	-e DISPLAY=unix$DISPLAY \
	${ENV_VARS_CALL} \
	--cap-add SYS_PTRACE --security-opt apparmor:unconfined \
	phd /bin/sh -c "$*"
