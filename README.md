# micropython-rcswitch

Port of the [rc-switch](https://github.com/sui77/rc-switch) library to micropython.

### Receiving and decoding RC codes
Find out what codes your remote control sends out. Use your remote control to control esp32.
All you need is an esp32, a 315/433 MHz AM receiver (although there is no manual yet, but you can hack an existing device), and a remote control.
At the moment the library only supports signal reception and decoding.

The `cc1101` module uses SPI to communicate with the `esp32`. Configure the module so that it receives the signal in raw mode. Use pin `gd0` / `gd2` to initialize the `rc-switch` library.

### Instalation
This code is an esp-idf project. You will need esp-idf to compile it. Newer versions of esp-idf may introduce incompatibilities with this code; for your reference, the code was tested against verstion `v4.4`

```sh
make BOARD=GENERIC_C3 clean
make BOARD=GENERIC_C3 USER_C_MODULES=micropython.cmake deploy
```

For docker (use `ep.sh` for entrypoint)
```sh
docker run -it --device=/dev/ttyUSB0 -v "$(pwd)"/cc1101:/esp32/cc1101 esp-idf:v4.4 /bin/bash -c "/esp32/cc1101/ep.sh"
```

### How to use

```python
from rcswitch import RCSwitch

#callback function
def getCode(val):
    print(val)

rf = RCSwitch()
rf.enableReceive(10, getCode) # set the pin number and callback func
```
