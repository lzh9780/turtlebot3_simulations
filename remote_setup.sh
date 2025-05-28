#!/bin/bash

HOST=$1
LOCAL_DATE=$(date +%m%d%H%M%Y.%S)
set password "turtlebot"

ssh ubuntu@${HOST} "sudo date $LOCAL_DATE"
expect "password:"
send "$password\r"
expect eof