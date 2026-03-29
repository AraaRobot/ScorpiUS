# Serial communication protocol

## Packet structure

HEAD 0xAA
LEN (1 byte)
DATA:
    MSG TYPE (1 byte)
    MSG CONTENT (LEN - 1 byte)
CHECKSUM (1 byte)
TAIL 0xBB

## MSG types

COMMAND 0x00
    From: Pi
    To: Arduino
    Content: 12 bytes representing servo angles

INFO 0x01
    From: Arduino
    To: Pi
    Content: 1 byte representing info code (use lookup table)

ERROR 0x02
    From: Arduino
    To: Pi
    Content: 1 byte representing error code (use lookup table)

HEARTBEAT 0x03
    From: Arduino
    To: Pi
    Content: Empty (only send msg type)

STATE 0x04
    From: Pi
    To: Arduino
    Content: 1 byte reprensenting state to change to

### MSG LOOKUP TABLE

INFO

| Code   | Meaning     |
| ------ | ------------- |
| 0x01   | Init complete |
| 0x02   | Servos Homed |

ERROR

| Code   | Meaning     |
| ------ | ------------- |
| 0x01   | Received invalid packet length |
| 0x02   | Checksum mismatch |
| 0x03   | Packet dropped |
| 0x04   | Wrong msg type |
| 0x05   | Invalid COMMAND (0x00) received |
| 0x06   | Tried to send invalid command |
| 0x07   | Invalid servo ID |
| 0x08   | Received invalid state |

STATE

| Code   | Meaning     |
| ------ | ------------- |
| 0x01   | Home |
| 0x02   | Running |
| 0x03   | Reset |