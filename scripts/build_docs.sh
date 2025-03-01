#! /bin/bash

set -e

cd ./docs
make clean && make html && cp -r build/html/ ./
