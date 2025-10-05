#!/bin/bash

#!/bin/bash

# Compile and link main.cpp
echo "🔧 Compiling main.cpp with g++ and linking..."
cmake --build . --config Release
if [ $? -ne 0 ]; then
    echo "❌ Failed to compile and link main.cpp"
    exit 1
fi
echo "✅ Build successful! Run with ./WheelLoadDistributor.exe "
