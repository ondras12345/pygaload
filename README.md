# pygaload
A utility for [MegaLoad bootloaders][MegaLoad bootloader] (link seems dead,
[archive][MegaLoad bootloader archive]).

This version is based on
[`pygaload.py` from Robopoly/lasermodules][original file].


## Usage
```console
$ ./pygaload.py --help
usage: pygaload.py [-h] [-p DEV] [-b BAUD] [-V] [-t SEC] [-D] [-s STRING]
                   [-v]
                   [--procinfo-order PROCINFO_ORDER PROCINFO_ORDER PROCINFO_OR
DER PROCINFO_ORDER PROCINFO_ORDER]
                   programfile

positional arguments:
  programfile           file to download (in HEX format)

optional arguments:
  -h, --help            show this help message and exit
  -p DEV, --port DEV    Device port for communication (default:
                        /dev/ttyUSB0)
  -b BAUD, --baud-rate BAUD
                        Baud rate (default: 38400)
  -V, --verbose         Print verbose progress reports (-VV is DEBUG)
  -t SEC, --timeout SEC
                        How long to wait for bootloader response (default:
                        10 seconds)
  -D, --debug           Debugging mode (not for normal use)
  -s STRING, --send-reset STRING
                        String to send to invoke bootloader. Can include
                        C-style control characters such as \n and \r. For
                        example: --send-reset='reset\r\n' or --send-
                        reset='\x03'
  -v, --version         Print version info and exit
  --procinfo-order PROCINFO_ORDER PROCINFO_ORDER PROCINFO_ORDER PROCINFO_ORDER
 PROCINFO_ORDER
                        Order in which the bootloader sends information like
                        flash size. Default: `proc flash boot page eeprom'.
                        E.g. EvB 5.1 boards seem to need `page proc flash
                        boot eeprom'.
```

Example: upload `blink.hex` to an EvB 5.1 board:
```console
$ ./pygaload.py --verbose -p /dev/ttyUSB0 -b 57600 --procinfo-order page proc flash boot eeprom blink.hex
```


## License
The [original file][original file] contained the following note:

> This program is free software licensed under the terms of the GNU General
> Public License. See http://www.gnu.org/licenses/gpl.html for the license terms.


[original file]: https://github.com/Robopoly/lasermodules/blob/2333aa9272de6369d4d3a90a787136374d1e01da/pygaload.py
[MegaLoad bootloader]: http://www.microsyl.com/index.php/2010/03/30/megaload/
[MegaLoad bootloader archive]: https://web.archive.org/web/20210512054826/http://www.microsyl.com/index.php/2010/03/30/megaload/
