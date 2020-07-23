SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR

mkdir -p build 
cd build
cmake ..
make -j 8
