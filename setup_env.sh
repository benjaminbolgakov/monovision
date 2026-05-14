#!/usr/bin/env bash

echo "Checking for pyenv..."
if ! command -v pyenv &> /dev/null; then
    printf "Error: pyenv is not installed. Please install it first.\n"
    exit 1
else
	printf "Found 'pyenv' installation! Proceed..\n\n"
fi

printf "Install required python version:\n"
printf "'pyenv install -s 3.11.0'\n\n"

printf "Create virtual env from installed version:\n"
printf "'pyenv virtualenv 3.11.0 <name>'\n\n"
#python -m venv venv

printf "Activate:\n"
printf "'pyenv activate <name>'\n\n"

#echo "Activating environment and installing requirements..."
# source venv/bin/activate
# pip install --upgrade pip
# pip install -r requirements.txt

#echo "Done! Use 'source venv/bin/activate' to start developing."

#python -m pip install --upgrade pip setuptools wheel
#pip install -r requirements.txt
