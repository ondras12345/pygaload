#!/usr/bin/env python3
"""This program acts as a client for a bootloader that implements the MegaLoad
protocol version 3, 4, or 5. Only FLASH programming is supported, not EEPROM
or lock bit programming.

This program is free software licensed under the terms of the GNU General
Public License. See http://www.gnu.org/licenses/gpl.html for the license terms.

The protocol is as follows:

* Upon boot, if autobaud is in effect, the character 0x55 ('U') is repeatedly
  transmitted until 0x55 is received in return. If 0x55 is not received after
  all baud rate divisors have been tried, the main FLASH code at address 0x0000
  is executed.

  NOTE: MegaLoad 3 sends the '>' character (0x3E) instead of 0x55 and expects
  '<' (0x3C) in return.

* If auto-OSCCAL is in effect, then 0x55 characters continue to be
  sent out, and 0x55 is expected in return.

  MegaLoad 3 does not do auto-OSCCAL.

* Following this, any received character is flushed and a '>' character (0x3E)
  is sent.  If a '<' character (0x3C) is received, then bootloading begins.
  Otherwise, the user's code at address 0x0000 is executed.

  MegaLoad 3 and MegaLoad 4 do not send this '>' character and do not wait for '<'.
  Bootloading begins immediately after the auto-OSCCAL step.

* Following this, the DeviceID is sent out. This is:
    Mega8:      'A' (0x41)
    Mega16:     'B' (0x42)
    Mega64:     'C' (0x43)
    Mega128:    'D' (0x44)
    Mega32:     'E' (0x45)
    Mega162:    'F' (0x46)
    Mega169:    'G' (0x47)
    Mega8515:   'H' (0x48)
    Mega8535:   'I' (0x49)
    Mega163:    'J' (0x4A)
    Mega323:    'K' (0x4B)
    Mega48:     'L' (0x4C)
    Mega88:     'M' (0x4D)
    Mega168:    'N' (0x4E)
    Tiny2313:   'O' (0x4F)
    Tiny13:     'P' (0x50)
    Mega165:    0x80
    Mega3250:   0x81
    Mega6450:   0x82
    Mega3290:   0x83
    Mega6490:   0x84
    Mega406:    0x85
    Mega640:    0x86
    Mega1280:   0x87
    Mega2560:   0x88

* Following the DeviceID, the FlashSize is sent out. This is:
    Flash1k:    'g' (0x67)
    Flash2k:    'h' (0x68)
    Flash4k:    'i' (0x69)
    Flash8k:    'l' (0x6C)
    Flash16k:   'm' (0x6D)
    Flash32k:   'n' (0x6E)
    Flash64k:   'o' (0x6F)
    Flash128k:  'p' (0x70)
    Flash256k:  'q' (0x71)
    Flash40k:   'r' (0x72)

* Following the FlashSize, the BootSize IN WORDS is sent out. This is:
    Boot128:    'a' (0x61)
    Boot256:    'b' (0x62)
    Boot512:    'c' (0x63)
    Boot1024:   'd' (0x64)
    Boot2048:   'e' (0x65)
    Boot4096:   'f' (0x66)

* Following the BootSize, the PageSize is sent out. This is:
    Page32:   'Q' (0x51)
    Page64:   'R' (0x52)
    Page128:  'S' (0x53)
    Page256:  'T' (0x54)
    Page512:  'V' (0x56)

  NOTE: MegaLoad 3 uses 'U' (0x55) for Page512.

* Following the PageSize, the EEPROMSize is sent out. This is:
    EEprom64:     '.' (0x2E)
    EEprom128:    '/' (0x2F)
    EEprom256:    '0' (0x30)
    EEprom512:    '1' (0x31)
    EEprom1024:   '2' (0x32)
    EEprom2048:   '3' (0x33)
    EEprom4096:   '4' (0x34)

* MegaLoad then reads any incoming character, which is ignored. NOTE: There is
  no waiting for a character...it's just a read of the UDR so no character needs
  to be sent.

* The FLASH loading process then begins. The character '!' (0x21) is transmitted.

  NOTE: MegaLoad 4 transmits the '>' character before the '!' character.

* The process of loading pages begins. For each page:
   - The 16-bit page number must be sent MSB-first. The page address is
     constructed from the page number by shifting it left by log2(PageSize) bits.
     For example, for the ATmega32, the page number is shifted left by 7 bits.

   - If the page number is 0xFFFF, the process is complete.

   - Once the page number/address is obtained, the page data is obtained.
     PageByte characters are received in sequence, LSB first. That is, each
     16-bit AVR program word is sent LSB-first, the way they are stored in
     HEX files.

   - An 8-bit checksum over all PageByte characters is computed. The next
     character received by MegaLoad is expected to be this checksum. If not,
     MegaLoad transmits '@' (0x40). It is possible to then retransmit this page
     if so desired, beginning with the 16-bit page number.

   - If the checksum is correct, the FLASH is written. If the write is successful,
     '!' (0x21) is written by MegaLoad, else '@' (0x40) is written.

* Once the FLASH process is complete, EEPROM loading begins. MegaLoad transmits ')' (0x29)
  followed by '!' (0x21).

  For each EEPROM character to write, MegaLoad expects the 16-bit address to be sent, MSB-first,
  followed by the 8-bit data byte, followed by the 8-bit checksum over all 3 bytes sent so
  far (two address bytes plus data byte).

  An EEPROM address of 0xFFFF terminates the EEPROM loading process immediately
  without expecting a data byte or checksum byte.

  If the checksum matches and the EEPROM write is successful, '!' (0x21) is transmitted by
  MegaLoad, or '@' (0x40) in all other cases.

* Once EEPROM writing is complete, Lock Bit programming begins. MegaLoad transmits '%' (0x25)
  then waits for two characters. The two characters are expected to be one's complements
  of each other. If not, nothing is done.

  If the two characters are one's complements of each other, the lock bits are written with
  the second byte received (i.e., the one's complement of the first byte received).

  There is no indication of whether this process was successful or not.
"""

import sys
import serial
import time
import array
import codecs
import logging
import argparse
from intelhex import IntelHex

VERSION_MAJOR = 1
VERSION_MINOR = 1

Default_DevicePort = "/dev/ttyUSB0"
Default_BaudRate = 38400
Default_Timeout = 10

_LOGGER = logging.getLogger(__name__)

##############################################################################

Processors = {0x41: 'ATmega8',
              0x42: 'ATmega16',
              0x43: 'ATmega64',
              0x44: 'ATmega128',
              0x45: 'ATmega32',
              0x46: 'ATmega162',
              0x47: 'ATmega169',
              0x48: 'ATmega8515',
              0x49: 'ATmega8535',
              0x4A: 'ATmega163',
              0x4B: 'ATmega323',
              0x4C: 'ATmega48',
              0x4D: 'ATmega88',
              0x4E: 'ATmega168',
              0x4F: 'ATtiny2313',
              0x50: 'ATtiny13',
              0x80: 'ATmega165',
              0x81: 'ATmega3250',
              0x82: 'ATmega6450',
              0x83: 'ATmega3290',
              0x84: 'ATmega6490',
              0x85: 'ATmega406',
              0x86: 'ATmega640',
              0x87: 'ATmega1280',
              0x88: 'ATmega2560'}

FlashSize = {
    0x67: 1024,
    0x68: 2048,
    0x69: 4096,
    0x6C: 8192,
    0x6D: 16384,
    0x6E: 32768,
    0x6F: 65536,
    0x70: 2*65536,
    0x71: 4*65536,
    0x72: 40*1024
}

# The boot size is reported in WORDS
BootSize = {
    0x61: 128,
    0x62: 256,
    0x63: 512,
    0x64: 1024,
    0x65: 2048,
    0x66: 4096
}

PageSize = {
    0x51: 32,
    0x52: 64,
    0x53: 128,
    0x54: 256,
    0x55: 512,  # MegaLoad 3 uses 0x55 for Page512
    0x56: 512
}

EEPROMSize = {
    0x2E: 64,
    0x2F: 128,
    0x30: 256,
    0x31: 512,
    0x32: 1024,
    0x33: 2048,
    0x34: 4096
}

PROCINFO_ORDER_DEFAULT = [
    "proc",
    "flash",
    "boot",
    "page",
    "eeprom",
]

PROCINFO_ORDER_EVB = [
    "page",
    "proc",
    "flash",
    "boot",
    "eeprom"
]


def openDevice(options):
    dev = serial.Serial()
    dev.port = options.DevicePort
    dev.baudrate = options.BaudRate
    # Two stop bits seems to work better in some cases.
    dev.stopbits = 2
    dev.timeout = 3

    dev.open()
    _LOGGER.info(f"Opened {options.DevicePort} ...")

    return dev


class Proc:
    """Representation of a processor with MegaLoad bootloader.

    Contains information like flash size, bootloader size and processor type.

    Check is_complete before use, otherwise it may contain incorrect or
    incomplete information.
    """

    class BadOrder(Exception):
        pass

    class BadCharacter(Exception):
        pass

    def __init__(self):
        self.loaderversion = None  # needs to be set externally
        self.proc = None
        self.page = None
        self.flash = None
        self.boot = None
        self.eeprom = None
        self.order_received = []

    def decode_character(self, character):
        """Decode character / byte sent by bootloader to processor attributes.

        This only detects things like flash size. It does send anything to the
        bootloader, so it cannot handle the initial sync.
        """
        _LOGGER.debug(f"Decoding character {character}")
        c = ord(character)

        # no character / byte has multiple meanings, so we don't have to check
        # order_received every time we receive something. We can just raise
        # an exception in is_valid.

        steps = [
            ("proc", Processors),
            ("page", PageSize),
            ("flash", FlashSize),
            # boot needs to be handled separately
            ("eeprom", EEPROMSize),
        ]

        # boot needs to be handled separately because it needs to be
        # multiplied by 2.
        if c in BootSize and self.boot is None:
            # *2 because BootSize is in WORDS
            self.boot = BootSize[c]*2
            self.order_received.append("boot")
            _LOGGER.debug(f"Got boot: {self.boot}")  # this is NOT in WORDS
            return "boot"

        for variable, lookup_table in steps:
            if c in lookup_table and getattr(self, variable) is None:
                setattr(self, variable, lookup_table[c])
                self.order_received.append(variable)
                _LOGGER.debug(f"Got {variable}: {getattr(self, variable)}")
                return variable

        raise self.BadCharacter(f"Unknown character: {character}")

    def is_complete(self, expected_order=None):
        """Return True if we have all the information.

        Raises BadOrder if expected_order is provided and does not match
        order_received.
        """
        if expected_order is not None and self.order_received != expected_order:
            raise self.BadOrder(
                    f"Expected {expected_order}; got {self.order_received}")

        if any(a is None for a in [self.proc, self.page, self.flash,
                                   self.boot, self.eeprom]
               ):
            return False

        return True


def doConnect(options):
    CONNECT = 1     # Waiting for bootloader sync character
    SYNCED3 = 2     # Received 0x3E sync character from MegaLoad 3 bootloader
    SYNCED4 = 3     # Received 0x55 sync character from MegaLoad 4 bootloader
    INFO = 4        # Started getting valid information
    GOTALL = 5      # Got all processor specs. Waiting for '!' or '>'

    state = CONNECT

    P = Proc()

    tic = time.time()
    while (time.time()-tic) < options.Timeout:
        c = options.dev.read(1)
        if not c:
            if options.Verbose:
                sys.stdout.write('.')
                sys.stdout.flush()
            continue

        _LOGGER.debug(f"char received: {c}")

        if state == CONNECT:
            if ord(c) == 0x55:  # For MegaLoad 4 and 5
                options.dev.write(b'\x55')
                state = SYNCED4
                P.loaderversion = 4
                continue
            elif ord(c) == 0x3E:  # For MegaLoad 3
                options.dev.write(b'\x3C')
                state = SYNCED3    # Maybe it's just junk, though, from a MegaLoad 4 bootloader
                P.loaderversion = 3
                _LOGGER.debug("MegaLoad 3 detected")
                continue

        # We could keep getting 0x55's from a MegaLoad 4 bootloader that
        # is doing auto-OSCCAL, or we got '>' from MegaLoad 5.
        if state == SYNCED4:
            if ord(c) == 0x55:    # We're in auto-OSCCAL
                continue
            elif ord(c) == 0x3E:  # MegaLoad 5 sends '>'
                options.dev.write('\x3C')
                P.loaderversion = 5
                continue

        if state in (SYNCED3, SYNCED4):
            try:
                P.decode_character(c)
                state = INFO
            except Proc.BadCharacter:
                # Nope...we've just got garbage.
                _LOGGER.debug(f"Got garbage: {c}")
                state = CONNECT
            finally:
                continue

        if state != GOTALL:
            P.decode_character(c)
            # we don't check expected_order yet
            if P.is_complete():
                state = GOTALL
                continue

        if state == GOTALL:
            if ord(c) == 0x21:         # '!' means we're all done
                # just check expected_order; it will for sure return True
                P.is_complete(expected_order=options.procinfo_order)
                _LOGGER.info(f'Using MegaLoad {P.loaderversion} protocol ...')
                return P
            elif ord(c) == 0x3E:     # '>' is MegaLoad 4, and comes before '!'
                continue

    _LOGGER.error('*** Timeout waiting for bootloader to connect')
    return None


def downloadFlash(options, proc, hex_data):
    # Basic checks:
    #     - make sure we're not going beyond max address where bootloader starts
    #     - make sure flash is an integer number of pages
    #     - make sure boot section is an integer number of pages

    lastAddr = max(hex_data)

    if lastAddr > (proc.flash - proc.boot):
        print('*** HEX file contents extends into bootloader')
        return 0

    NumPages = proc.flash // proc.page
    if NumPages*proc.page != proc.flash:
        print('*** FLASH size is not an integer number of pages')
        return 0

    NumPages -= proc.boot // proc.page
    if NumPages*proc.page + proc.boot != proc.flash:
        print('*** Bootloader size is not an integer number of pages')
        return 0

    emptypage = b'\xFF'*proc.page

    if options.Debug:
        dumpfid = open('dump.txt', 'wt')

    for pagenum in range(NumPages):
        startAddr = pagenum*proc.page
        endAddr = startAddr + proc.page

        if options.Verbose:
            print(f"\r        Page {pagenum} ...",)
            sys.stdout.flush()

        page = array.array('B', emptypage)

        has_data = False
        for addr in range(startAddr, endAddr):
            if addr in hex_data:
                page[addr - startAddr] = hex_data[addr]
                has_data = True

        if not has_data:
            # Next byte to write is past this page
            continue

        towrite = page.tobytes()

        if towrite != emptypage:
            for tries in range(3):
                if options.Debug:
                    print('Page:', pagenum, file=dumpfid)
                    for ix in range(len(towrite)):
                        print('%02X ' % towrite[ix], file=dumpfid)
                        if (ix & 0x0F) == 0x0F:
                            print("", file=dumpfid)
                else:
                    options.dev.write(bytes([(pagenum >> 8) & 0xFF]))
                    options.dev.write(bytes([pagenum & 0xFF]))
                    options.dev.write(towrite)

                checksum = 0
                for ix in range(len(towrite)):
                    checksum += towrite[ix]

                if options.Debug:
                    print("Checksum:", hex(checksum & 0xFF), file=dumpfid)
                else:
                    _LOGGER.debug(f"Writing checksum: {checksum & 0xFF}")
                    options.dev.write(bytes([checksum & 0xFF]))

                if options.Debug:
                    if 0:
                        print('Response to page #%d' % pagenum, '(! or @ or Enter):')
                        s = input()
                        if s:
                            c = ord(s[0])
                        else:
                            c = None
                    else:
                        c = ord('!')
                else:
                    c = options.dev.read(1)
                    c = ord(c) if c else None

                if c is not None:
                    if c == 0x21:
                        break     # successful write
                    elif c == 0x40:
                        if options.Verbose:
                            print('failed')
                            print(f"\r        Page {pagenum} ...",)
                            sys.stdout.flush()
                    else:
                        print('\n*** Unexpected response %02X to FLASH page write' % c)
                        return 0
                else:
                    print('\n*** No response from bootloader')
                    return 0
            else:
                print('\n*** Giving up after 3 tries')
                return 0

    # Flash writing is all done...we must send a page number of 0xFFFF
    options.dev.write(b'\xFF\xFF')

    if options.Verbose:
        print()
    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", dest="DevicePort",
                        help="Device port for communication (default: %s)" % Default_DevicePort,
                        metavar="DEV", default=Default_DevicePort)
    parser.add_argument("-b", "--baud-rate", type=int, dest="BaudRate", help="Baud rate (default: %d)" % Default_BaudRate,
                        metavar="BAUD", default=Default_BaudRate)
    parser.add_argument("-V", "--verbose", dest="Verbose", action="count", default=False, help="Print verbose progress reports (-VV is DEBUG)")
    parser.add_argument("-t", "--timeout", dest="Timeout", type=float, default=Default_Timeout,
                        help="How long to wait for bootloader response (default: %g seconds)" % Default_Timeout, metavar="SEC")
    parser.add_argument("-D", "--debug", dest="Debug", action="store_true", default=False, help="Debugging mode (not for normal use)")
    parser.add_argument("-s", "--send-reset", dest="SendReset", metavar="STRING", default=None,
                        help="String to send to invoke bootloader. "
                             "Can include C-style control characters such as "
                             "\\n and \\r. For example: "
                             "--send-reset='reset\\r\\n' or "
                             "--send-reset='\\x03'")
    parser.add_argument("-v", "--version", dest="Version", action="store_true", default=False, help="Print version info and exit")
    parser.add_argument("--procinfo-order", nargs=len(PROCINFO_ORDER_DEFAULT),
                        default=PROCINFO_ORDER_DEFAULT,
                        help="Order in which the bootloader sends information "
                        "like flash size. "
                        f"Default: `{' '.join(PROCINFO_ORDER_DEFAULT)}'. "
                        "E.g. EvB 5.1 boards seem to "
                        f"need `{' '.join(PROCINFO_ORDER_EVB)}'.")
    parser.add_argument("programfile", type=argparse.FileType("r"),
                        help="file to download (in HEX format)")

    options = parser.parse_args()
    if options.Version:
        print("""
PYGALOAD version %d.%d

This is free software licensed under the terms of the GNU General
Public License. See http://www.gnu.org/licenses/gpl.html for the license
terms.

Andrew Sterian
Padnos College of Engineering and Computing
Grand Valley State University
<steriana@gvsu.edu> -- <http://claymore.engineer.gvsu.edu/~steriana>
""" % (VERSION_MAJOR, VERSION_MINOR))
        sys.exit(0)

    ih = IntelHex(options.programfile)
    hex_data = ih.todict()
    if not hex_data:
        print('*** HEX file is empty...nothing to download')
        sys.exit(1)

    if options.Verbose >= 2:
        logging.basicConfig(level=logging.DEBUG)
        _LOGGER.debug("Logging DEBUG messages")
    elif options.Verbose:
        logging.basicConfig(level=logging.INFO)
        _LOGGER.info("Logging INFO messages")
    else:
        logging.basicConfig(level=logging.WARNING)

    if not options.Debug:
        dev = openDevice(options)
        options.dev = dev

        if options.SendReset is not None:
            _LOGGER.info("Sending reset string ...")
            s = codecs.decode(options.SendReset, 'unicode_escape').encode('ascii')
            options.dev.write(s)

        _LOGGER.info('Waiting for bootloader to connect ...')

        proc = doConnect(options)
    else:
        proc = Proc()
        proc.proc = 'ATmega32'
        proc.flash = 32768
        proc.boot = 512*2
        proc.page = 128
        proc.eeprom = 1024

    if proc:
        _LOGGER.info('Downloading FLASH ...')
    else:
        print('*** Downloading failed')
        sys.exit(1)

    if downloadFlash(options, proc, hex_data):
        print('Downloading successful')
    else:
        print('*** Downloading failed')
        sys.exit(1)

    sys.exit(0)
