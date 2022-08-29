#! /bin/sh

pushd inputData
echo "Generating Input Data using Basilisk..."
python3 scenarioAerocapture.py
echo "Done generating sensor data"
popd

mkdir build
pushd build
echo "Starting Makefile generation..."
cmake ..

echo "Starting Build Process..."
make -j
echo "Finished Building Executables..."
popd
