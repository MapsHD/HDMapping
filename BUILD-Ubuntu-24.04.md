# For Ubuntu 24.04 (and  WSL Ubuntu 24.04) use following commands to build the project: 

``` bash
git clone --recursive https://github.com/MapsHD/HDMapping.git
cd HDMapping
./ubuntu-24.04-apt-requirements.sh
cmake -B build -S .
cmake  --build build --config Release -j
```