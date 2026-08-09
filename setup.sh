#!/bin/bash
set -e

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Done."
