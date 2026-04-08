#!/usr/bin/env bash

echo "Checking for pyenv..."
if ! command -v pyenv &> /dev/null; then
    echo "Error: pyenv is not installed. Please install it first."
    exit 1
fi

echo "Installing Python 3.11.0..."
pyenv install -s 3.11.0

echo "Creating virtual environment..."
python -m venv venv

echo "Activating environment and installing requirements..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "Done! Use 'source venv/bin/activate' to start developing."

#python -m pip install --upgrade pip setuptools wheel
#pip install -r requirements.txt
