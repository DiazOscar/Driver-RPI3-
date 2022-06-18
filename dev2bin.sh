#!/bin/bash
cat $1 | xxd -b | cut -f 2 -d " " | cut -c3-8
