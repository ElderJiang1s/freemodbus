# RTU master request regression tests

Run from the repository root on Linux (GCC or Clang) or macOS (Xcode Command
Line Tools / Apple Clang):

```sh
./tests/host/run.sh
# Choose a compiler explicitly:
CC=clang ./tests/host/run.sh
```

The script needs only a C99 compiler and POSIX shell. It uses AddressSanitizer
and UndefinedBehaviorSanitizer by default, fails on a sanitizer diagnostic, and
removes its temporary executable on exit. `SANITIZERS=address` selects ASan
alone; `SANITIZERS=none` is available for toolchains without sanitizer support
but does not constitute memory-safety validation. GitHub Actions runs the
sanitized suite on both Ubuntu and macOS.

In restricted Linux containers where LeakSanitizer cannot inspect `/proc`, use:

```sh
ASAN_OPTIONS=detect_leaks=0 ./tests/host/run.sh
```

This disables leak detection only; ASan bounds checks and UBSan remain enabled.

## What is exercised

The harness links the **real**, separately compiled request builders,
`mbrtu_m.c`, `mbcrc.c`, and `mbutils.c`, using the production headers and port
types. Tiny RT-Thread header stubs and host OS/serial/timer implementations
remove the RTOS dependency. The wait stub calls the real RTU sender and drains
its real transmit FSM into a byte capture. No production source is copied into
the harness or compiled through a private test-only branch.

Every successful request is checked against an independently assembled PDU,
its full RTU length, big-endian address/count/register fields, payload bytes,
and low-byte-first CRC. A bitwise CRC oracle is independent of the production
lookup table; both are anchored to the `123456789 -> 0x4B37` known vector.
Unused bytes after a frame must remain unchanged.

Invalid request tests assert `MB_MRE_ILL_ARG` **before** acquiring the master
resource, changing the entire 256-byte send buffer, destination or PDU length,
posting an event, waiting, or enabling transmission. Valid requests rejected
by the resource stub retain `MB_MRE_MASTER_BUSY` with no buffer mutation.
Null payloads, `65535` counts, configured slave-address limits, and address
ranges crossing `0xFFFF` are included. Address `0xFFFF` with count 1 is legal.
Payloads for valid variable-length requests use exact-size allocations so ASan
can also detect reads beyond the caller's supplied data.

| FC16 quantity | Result | PDU bytes | RTU bytes |
| ---: | --- | ---: | ---: |
| 0 | `MB_MRE_ILL_ARG` | not constructed | not sent |
| 1 | accepted | 8 | 11 |
| 120 | accepted | 246 | 249 |
| 121 | accepted | 248 | 251 |
| 122 | accepted | 250 | 253 |
| 123 | accepted | 252 | 255 |
| 124 | `MB_MRE_ILL_ARG` | not constructed | not sent |

Additional cases cover FC23 write counts 1/119/120/121 and read counts 1/125,
invalid read counts 0/126/65535 and write counts 0/122/123/65535; FC15 byte
rounding and the 1968-coil maximum; all fixed-size master constructors; and
valid broadcast writes. Direct transport tests exercise PDU lengths
1/250/251/252/253 (including a full 256-byte ADU), reject 0/254/255/256/65535,
null/external/offset PDU pointers, and retain `MB_EIO` when RX is not idle.

## Request-path audit and scope

Limits follow the [Modbus Application Protocol V1.1b3](https://www.modbus.org/modbus-specifications),
sections 6.1-6.6, 6.11-6.12 and 6.17. Quantities are validated before any
multiplication, byte-count narrowing, resource acquisition or buffer write.
Address-span arithmetic uses `ULONG`, avoiding 16-bit wraparound.

| Master constructor | Protocol quantity | Maximum request PDU | Change |
| --- | --- | ---: | --- |
| FC01 read coils | 1-2000 | 5 | quantity and address span |
| FC02 read discrete inputs | 1-2000 | 5 | quantity and address span |
| FC03 read holding registers | 1-125 | 5 | quantity and address span |
| FC04 read input registers | 1-125 | 5 | outgoing quantity and address span only |
| FC05 write single coil | exactly one, value 0x0000/0xFF00 | 5 | existing validation retained and tested |
| FC06 write single register | exactly one | 5 | existing validation retained and tested |
| FC15 write multiple coils | 1-1968 | 252 | zero quantity, null payload and span; existing upper bound retained |
| FC16 write multiple registers | 1-123 | 252 | enforce corrected maximum, zero quantity, null payload and span |
| FC23 read/write registers | read 1-125, write 1-121 | 252 | both quantities/spans and null payload |

The RTU master buffer now holds the 256-byte **ADU**, not just the 253-byte PDU.
The send function validates PDU length and available address/CRC capacity and
requires the PDU pointer returned by `vMBMasterGetPDUSndBuf`. This is the
existing internal call path: CRC bytes were already written to the internal
buffer, so arbitrary external pointers could not produce a coherent frame.
Public function signatures, enums and structures are unchanged; the private
static send array grows by three bytes. Existing broadcast policy is retained.
The raw payload API has no allocation-length argument: callers must still
provide the declared number of elements (and zero unused FC15 padding bits).

This change does not modify slave FC04 handling, master response parsing,
Modbus TCP, or RT-Thread production ports. FC04's outgoing request guard is part
of the constructor audit, not a reimplementation of a receive-side fix.

## Reproduce against the original source

The following commands use the test harness from the fixed checkout with
unmodified production files from the original master commit. Each reproducer
is a separate process. Their setup deliberately does **not** fill a 256-byte
buffer, so a diagnostic points at a production write rather than test setup.

```sh
git worktree add --detach ../freemodbus-baseline 18b2228605c1aa3ad9145147f0eaa3270adf5012
SOURCE_DIR=../freemodbus-baseline SANITIZERS=address ./tests/host/run.sh --repro-fc16
SOURCE_DIR=../freemodbus-baseline SANITIZERS=address ./tests/host/run.sh --repro-fc23
SOURCE_DIR=../freemodbus-baseline SANITIZERS=address ./tests/host/run.sh --repro-fc15
SOURCE_DIR=../freemodbus-baseline SANITIZERS=address ./tests/host/run.sh --repro-fc16-oversize
SOURCE_DIR=../freemodbus-baseline SANITIZERS=address ./tests/host/run.sh --repro-fc23-oversize
```

Expected old-source failures:

- FC16 quantity 123, FC23 write quantity 121, and FC15 quantity 1968:
  `global-buffer-overflow` in `eMBMasterRTUSend`, original line 217, at byte 253
  immediately beyond `ucMasterRTUSndBuf[253]` while appending CRC.
- FC16 quantity 124 and FC23 write quantity 122: `global-buffer-overflow` in
  their request data loops, original lines 191 and 384 respectively.

Omit `SOURCE_DIR` to run each reproducer against the fixed sources; all must
exit successfully. The oversize reproducers then require `MB_MRE_ILL_ARG`.

## Evidence and remaining hardware work

Source arithmetic predicts the overflow: 252-byte PDU + address + CRC = 255
bytes, exceeding the old array by two bytes. On Linux x86-64 / GCC 13.3, all
five old-source reproductions above were observed under ASan, and the fixed
regression suite passed ASan+UBSan (`detect_leaks=0` due to the container's
`/proc` restriction). This establishes a host-executed memory error in the
original production code, not evidence of damage on any particular device.

The suite does not run the RT-Thread scheduler, master poll/event loop,
response callbacks, physical UART/RS-485, DMA or actual slaves. Its success
means that the expected bytes reached the host serial stub; it does not mean
a slave acknowledged them. Confirm on RT-Thread hardware:

1. FC16 123-register, FC23 read-125/write-121, and FC15 1968-coil requests;
   capture complete frames, CRC, replies, and application data.
2. UART interrupt/driver capacity, RS-485 direction turnaround and T3.5 timing.
3. Broadcast delay, timeout recovery, and back-to-back requests under scheduling
   and interrupt load.
