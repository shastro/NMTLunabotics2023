#!/bin/bash
set -e

echo "commit message: "
read message
echo "branch"
read branch
git pull origin $branch
git add .
git commit -m "${message}"
git push origin $branch
