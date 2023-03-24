#!/bin/bash

git pull
git add .
git commit -m $2
git push origin $1
