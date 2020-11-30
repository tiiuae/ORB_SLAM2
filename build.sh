
echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
cmake .. \
  -DROS_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"

make -j

echo "Uncompress vocabulary ..."

cd ../Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
