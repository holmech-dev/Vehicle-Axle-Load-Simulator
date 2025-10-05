#!/bin/bash

# Set the local MinGW path (adjust if needed)
export MINGW_DIR="$PWD/devTools/mingw64"

# Prepend g++ to the PATH only in this shell
export PATH="$MINGW_DIR/bin:$PATH"

# Optional: Show the compiler version to confirm
g++ --version