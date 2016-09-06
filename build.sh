#!/bin/bash
echo 'Building docker container located at'  $CONTAININGDIR
docker build -t phd .

