#!/bin/bash

DIR=$(cd $(dirname $0);pwd)
enxname=$(ifconfig | grep -B 1 -E '192.168.55.100|192.168.44.100' | grep 'flags' | cut -d ':' -f1)

case "$(uname -s)" in
  Darwin)   # MacOS
    sudo route add -net 224.0.0.0 -netmask 240.0.0.0 -interface $enxname
    ;;
  Linux)
    sudo ifconfig $enxname multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $enxname
    ;;
  *)
    echo 'Other OS' 
    ;;
esac
