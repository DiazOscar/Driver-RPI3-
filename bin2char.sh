#!/bin/bash
printf \\$(printf '%03o' $((2#$1)))
