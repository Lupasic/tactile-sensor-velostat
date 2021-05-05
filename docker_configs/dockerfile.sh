#!/bin/bash

DIRNAME=$(dirname "$0")

for f in "$DIRNAME/dockerfile/"*.Dockerfile
do
    cat "${f}"
    echo
done