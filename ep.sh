#/bin/bash

# docker run -it --device=/dev/ttyUSB0 -v "$(pwd)"/cc1101:/esp32/cc1101 esp-idf:v4.4 /bin/bash -c "/esp32/cc1101/ep.sh"

export LC_ALL=C.UTF-8
export LANG=C.UTF-8

cd ..
git clone https://github.com/micropython/micropython.git /esp32/micropython
cd /esp32/micropython
git submodule update --init --recursive
make -C mpy-cross

cd /esp32/micropython/ports/esp32
. $IDF_PATH/export.sh
make submodules

esptool.py -p /dev/ttyUSB0 -b 460800 erase_flash
make BOARD=GENERIC_C3 clean
make BOARD=GENERIC_C3 USER_C_MODULES=/esp32/cc1101/micropython.cmake deploy

ampy -p /dev/ttyUSB0 put /esp32/cc1101/main.py