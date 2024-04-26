cmake -S .\3rdparty\opencv\ -B .\3pbinary\build\opencv^
 -DCMAKE_BUILD_TYPE=Release^
 -DBUILD_SHARED_LIBS=OFF^
 -DWITH_CUDA=OFF^
 -DWITH_GSTREAMER=OFF^
 -DBUILD_TESTS=OFF^
 -DBUILD_DOC=OFF^
 -DBUILD_JAVA=OFF^
 -DBUILD_PERFTEST=OFF^
 -DBUILD_VTK=OFF^
 -DBUILD_WITH_STATIC_CRT=OFF^
 -DCMAKE_INSTALL_PREFIX=.\3pbinary\install\opencv

cmake --build .\3pbinary\build\opencv --target INSTALL --config Release