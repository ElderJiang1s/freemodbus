/* Host regression tests for master request construction and RTU transmission.
 * Real request builders, RTU buffer/FSM and CRC are linked without modification.
 * Only OS/serial/timer services and application callbacks are substituted.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mb.h"
#include "mb_m.h"
#include "mbframe.h"
#include "mbrtu.h"
#include "mbcrc.h"

#define RTU_CAPACITY 256U
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static UCHAR wire[RTU_CAPACITY];
static size_t wire_length;
static UCHAR destination;
static BOOL tx_enabled, resource_available = TRUE;
static unsigned take_calls, wait_calls, post_calls, serial_calls, critical_depth;
static unsigned test_cases;
static UCHAR before[RTU_CAPACITY];

void EnterCriticalSection(void) { ++critical_depth; }
void ExitCriticalSection(void) { CHECK(critical_depth > 0); --critical_depth; }
BOOL xMBMasterRunResTake(int32_t timeout)
{
    (void)timeout;
    ++take_calls;
    return resource_available;
}
void vMBMasterSetDestAddress(UCHAR address) { destination = address; }
UCHAR ucMBMasterGetDestAddress(void) { return destination; }
void vMBMasterSetErrorType(eMBMasterErrorEventType error) { (void)error; }
BOOL xMBMasterPortEventPost(eMBMasterEventType event)
{
    (void)event;
    ++post_calls;
    return TRUE;
}
BOOL xMBMasterPortSerialInit(UCHAR port, ULONG baud, UCHAR bits, eMBParity parity)
{
    (void)port; (void)baud; (void)bits; (void)parity;
    return TRUE;
}
void vMBMasterPortSerialEnable(BOOL receive, BOOL transmit)
{
    (void)receive;
    ++serial_calls;
    tx_enabled = transmit;
}
BOOL xMBMasterPortSerialPutByte(CHAR byte)
{
    CHECK(wire_length < sizeof(wire));
    wire[wire_length++] = (UCHAR)byte;
    return TRUE;
}
BOOL xMBMasterPortSerialGetByte(CHAR *byte) { *byte = 0; return TRUE; }
BOOL xMBMasterPortTimersInit(USHORT timeout) { (void)timeout; return TRUE; }
void vMBMasterPortTimersDisable(void) { }
void vMBMasterPortTimersT35Enable(void)
{
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
}
void vMBMasterPortTimersConvertDelayEnable(void)
{
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
}
void vMBMasterPortTimersRespondTimeoutEnable(void)
{
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
}

/* No receive handler should be reached by these send-side tests. */
eMBErrorCode eMBMasterRegHoldingCB(UCHAR *buf, USHORT addr, USHORT count, eMBRegisterMode mode)
{
    (void)buf; (void)addr; (void)count; (void)mode;
    CHECK(0); return MB_EIO;
}
eMBErrorCode eMBMasterRegCoilsCB(UCHAR *buf, USHORT addr, USHORT count, eMBRegisterMode mode)
{
    (void)buf; (void)addr; (void)count; (void)mode;
    CHECK(0); return MB_EIO;
}
eMBErrorCode eMBMasterRegInputCB(UCHAR *buf, USHORT addr, USHORT count)
{
    (void)buf; (void)addr; (void)count;
    CHECK(0); return MB_EIO;
}
eMBErrorCode eMBMasterRegDiscreteCB(UCHAR *buf, USHORT addr, USHORT count)
{
    (void)buf; (void)addr; (void)count;
    CHECK(0); return MB_EIO;
}

static UCHAR *pdu_buffer(void)
{
    UCHAR *pdu;
    vMBMasterGetPDUSndBuf(&pdu);
    return pdu;
}

static void drain_transmitter(void)
{
    unsigned iterations = 0;
    while (tx_enabled)
    {
        CHECK(++iterations <= RTU_CAPACITY + 1);
        (void)xMBMasterRTUTransmitFSM();
    }
    CHECK(critical_depth == 0);
}

eMBMasterReqErrCode eMBMasterWaitRequestFinish(void)
{
    ++wait_calls;
    CHECK(eMBMasterRTUSend(destination, pdu_buffer(), usMBMasterGetPDUSndLength()) == MB_ENOERR);
    drain_transmitter();
    /* Successful host transmission only; no slave reply is simulated. */
    return MB_MRE_NO_ERR;
}

/* Independent, bit-at-a-time oracle instead of the production CRC lookup table. */
static USHORT reference_crc(const UCHAR *bytes, size_t length)
{
    unsigned crc = 0xFFFF;
    size_t i;
    unsigned bit;
    for (i = 0; i < length; ++i)
    {
        crc ^= bytes[i];
        for (bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001U : 0);
    }
    return (USHORT)crc;
}

static void idle_transport(void)
{
    eMBMasterRTUStart();
    (void)xMBMasterRTUTimerExpired();
    wire_length = 0;
    take_calls = wait_calls = post_calls = serial_calls = 0;
    resource_available = TRUE;
}

static void prepare(void)
{
    idle_transport();
    memset(pdu_buffer() - 1, 0xA5, RTU_CAPACITY);
    memcpy(before, pdu_buffer() - 1, sizeof(before));
    vMBMasterSetPDUSndLength(0xBEEF);
    destination = 0xEE;
}

static void expect_untouched(unsigned expected_takes)
{
    CHECK(memcmp(before, pdu_buffer() - 1, sizeof(before)) == 0);
    CHECK(usMBMasterGetPDUSndLength() == 0xBEEF);
    CHECK(destination == 0xEE);
    CHECK(take_calls == expected_takes);
    CHECK(wait_calls == 0 && post_calls == 0 && serial_calls == 0);
    CHECK(wire_length == 0 && !tx_enabled && critical_depth == 0);
    ++test_cases;
}

#define REJECT(call) do { \
    prepare(); \
    CHECK((call) == MB_MRE_ILL_ARG); \
    expect_untouched(0); \
} while (0)
#define BUSY(call) do { \
    prepare(); resource_available = FALSE; \
    CHECK((call) == MB_MRE_MASTER_BUSY); \
    expect_untouched(1); \
} while (0)

static void put_u16(UCHAR *bytes, USHORT value)
{
    bytes[0] = (UCHAR)(value >> 8);
    bytes[1] = (UCHAR)value;
}

static void check_wire(UCHAR address, const UCHAR *expected_pdu, size_t length)
{
    USHORT crc;
    size_t i;
    CHECK(wire_length == length + 3);
    CHECK(wire[0] == address);
    CHECK(memcmp(wire + 1, expected_pdu, length) == 0);
    crc = reference_crc(wire, length + 1);
    CHECK(wire[length + 1] == (UCHAR)crc);
    CHECK(wire[length + 2] == (UCHAR)(crc >> 8));
    CHECK(reference_crc(wire, wire_length) == 0);
    CHECK(usMBCRC16(wire, (USHORT)wire_length) == 0);
    CHECK(memcmp(pdu_buffer() - 1, wire, wire_length) == 0);
    for (i = wire_length; i < RTU_CAPACITY; ++i) CHECK((pdu_buffer() - 1)[i] == 0xA5);
    CHECK(xMBMasterRequestIsBroadcast() == (address == 0));
    CHECK(!tx_enabled && critical_depth == 0);
    ++test_cases;
}

static void check_request(UCHAR address, const UCHAR *expected_pdu, size_t length)
{
    CHECK(usMBMasterGetPDUSndLength() == length);
    CHECK(take_calls == 1 && wait_calls == 1 && post_calls == 1);
    check_wire(address, expected_pdu, length);
}

static USHORT *register_data(size_t count)
{
    size_t i;
    USHORT *values = malloc(count * sizeof(*values));
    CHECK(values != NULL);
    for (i = 0; i < count; ++i)
        values[i] = (USHORT)(0x1234U + 257U * i);
    return values;
}

static void test_fc16(void)
{
    static const USHORT counts[] = {0, 1, 120, 121, 122, 123, 124, 0xFFFF};
    size_t c, i;
    USHORT one = 0x1234;
    for (c = 0; c < sizeof(counts) / sizeof(counts[0]); ++c)
    {
        USHORT count = counts[c];
        if (count == 0 || count > 123)
        {
            REJECT(eMBMasterReqWriteMultipleHoldingRegister(1, 0x1234, count, &one, 0));
            printf("FC16 count=%u: MB_MRE_ILL_ARG, no side effects\n", count);
        }
        else
        {
            UCHAR expected[252] = {0x10};
            USHORT *values = register_data(count);
            put_u16(expected + 1, 0x1234);
            put_u16(expected + 3, count);
            expected[5] = (UCHAR)(2 * count);
            for (i = 0; i < count; ++i) put_u16(expected + 6 + 2 * i, values[i]);
            prepare();
            CHECK(eMBMasterReqWriteMultipleHoldingRegister(1, 0x1234, count, values, 0) == MB_MRE_NO_ERR);
            check_request(1, expected, 6 + 2 * count);
            printf("FC16 count=%u: PDU=%u RTU=%u, bytes/CRC OK\n", count, 6 + 2 * count, 9 + 2 * count);
            free(values);
        }
    }
    REJECT(eMBMasterReqWriteMultipleHoldingRegister(1, 0, 1, NULL, 0));
    REJECT(eMBMasterReqWriteMultipleHoldingRegister(1, 0xFFFF, 2, &one, 0));
    REJECT(eMBMasterReqWriteMultipleHoldingRegister(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 1, &one, 0));
    BUSY(eMBMasterReqWriteMultipleHoldingRegister(1, 0, 1, &one, 0));
    {
        UCHAR expected[] = {0x10, 0xFF, 0xFF, 0, 1, 2, 0x12, 0x34};
        prepare();
        CHECK(eMBMasterReqWriteMultipleHoldingRegister(0, 0xFFFF, 1, &one, 0) == MB_MRE_NO_ERR);
        check_request(0, expected, sizeof(expected));
    }
}

static void test_fc23(void)
{
    static const USHORT writes[] = {1, 119, 120, 121};
    static const USHORT reads[] = {1, 125};
    size_t w, r, i;
    USHORT one = 0x1234;
    for (r = 0; r < sizeof(reads) / sizeof(reads[0]); ++r)
        for (w = 0; w < sizeof(writes) / sizeof(writes[0]); ++w)
        {
            UCHAR expected[252] = {0x17};
            USHORT count = writes[w];
            USHORT *values = register_data(count);
            put_u16(expected + 1, 0x4567);
            put_u16(expected + 3, reads[r]);
            put_u16(expected + 5, 0x1234);
            put_u16(expected + 7, count);
            expected[9] = (UCHAR)(2 * count);
            for (i = 0; i < count; ++i) put_u16(expected + 10 + 2 * i, values[i]);
            prepare();
            CHECK(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0x4567, reads[r], values, 0x1234, count, 0) == MB_MRE_NO_ERR);
            check_request(1, expected, 10 + 2 * count);
            free(values);
        }
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 0, &one, 0, 1, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 126, &one, 0, 1, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 0xFFFF, &one, 0, 1, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0, 0, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0, 122, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0, 123, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0, 0xFFFF, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, NULL, 0, 1, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0xFFFF, 2, &one, 0, 1, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0xFFFF, 2, 0));
    REJECT(eMBMasterReqReadWriteMultipleHoldingRegister(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 1, &one, 0, 1, 0));
    BUSY(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, &one, 0, 1, 0));
    {
        UCHAR expected[] = {0x17, 0xFF, 0xFF, 0, 1, 0xFF, 0xFF, 0, 1, 2, 0x12, 0x34};
        prepare();
        CHECK(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0xFFFF, 1, &one, 0xFFFF, 1, 0) == MB_MRE_NO_ERR);
        check_request(1, expected, sizeof(expected));
    }
}

static void test_fc15(void)
{
    static const USHORT counts[] = {1, 7, 8, 9, 1952, 1953, 1960, 1961, 1968};
    size_t c, i;
    UCHAR one = 1;
    for (c = 0; c < sizeof(counts) / sizeof(counts[0]); ++c)
    {
        USHORT count = counts[c];
        size_t bytes = (count + 7U) / 8U;
        UCHAR expected[252] = {0x0F};
        UCHAR *values = malloc(bytes);
        CHECK(values != NULL);
        for (i = 0; i < bytes; ++i) values[i] = (UCHAR)(0xCDU + i);
        if (count % 8) values[bytes - 1] &= (UCHAR)((1U << (count % 8)) - 1);
        put_u16(expected + 1, 0x1234);
        put_u16(expected + 3, count);
        expected[5] = (UCHAR)bytes;
        memcpy(expected + 6, values, bytes);
        prepare();
        CHECK(eMBMasterReqWriteMultipleCoils(1, 0x1234, count, values, 0) == MB_MRE_NO_ERR);
        check_request(1, expected, 6 + bytes);
        free(values);
    }
    REJECT(eMBMasterReqWriteMultipleCoils(1, 0, 0, &one, 0));
    REJECT(eMBMasterReqWriteMultipleCoils(1, 0, 1969, &one, 0));
    REJECT(eMBMasterReqWriteMultipleCoils(1, 0, 0xFFFF, &one, 0));
    REJECT(eMBMasterReqWriteMultipleCoils(1, 0, 1, NULL, 0));
    REJECT(eMBMasterReqWriteMultipleCoils(1, 0xFFFF, 2, &one, 0));
    REJECT(eMBMasterReqWriteMultipleCoils(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 1, &one, 0));
    BUSY(eMBMasterReqWriteMultipleCoils(1, 0, 1, &one, 0));
    {
        UCHAR expected[] = {0x0F, 0xFF, 0xFF, 0, 1, 1, 1};
        prepare();
        CHECK(eMBMasterReqWriteMultipleCoils(0, 0xFFFF, 1, &one, 0) == MB_MRE_NO_ERR);
        check_request(0, expected, sizeof(expected));
    }
}

typedef eMBMasterReqErrCode (*ReadRequest)(UCHAR, USHORT, USHORT, LONG);
static void test_reads(void)
{
    static const struct { ReadRequest request; UCHAR function; USHORT max; } readers[] = {
        {eMBMasterReqReadCoils, 1, 2000},
        {eMBMasterReqReadDiscreteInputs, 2, 2000},
        {eMBMasterReqReadHoldingRegister, 3, 125},
        {eMBMasterReqReadInputRegister, 4, 125}
    };
    size_t r;
    for (r = 0; r < sizeof(readers) / sizeof(readers[0]); ++r)
    {
        USHORT count;
        for (count = 1; ; count = readers[r].max)
        {
            UCHAR expected[5] = {0};
            expected[0] = readers[r].function;
            put_u16(expected + 1, (USHORT)(0x10000UL - count));
            put_u16(expected + 3, count);
            prepare();
            CHECK(readers[r].request(MB_MASTER_TOTAL_SLAVE_NUM, (USHORT)(0x10000UL - count), count, 0) == MB_MRE_NO_ERR);
            check_request(MB_MASTER_TOTAL_SLAVE_NUM, expected, sizeof(expected));
            if (count == readers[r].max) break;
        }
        REJECT(readers[r].request(1, 0, 0, 0));
        REJECT(readers[r].request(1, 0, readers[r].max + 1, 0));
        REJECT(readers[r].request(1, 0, 0xFFFF, 0));
        REJECT(readers[r].request(1, 0xFFFF, 2, 0));
        REJECT(readers[r].request(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 1, 0));
        BUSY(readers[r].request(1, 0, 1, 0));
    }
}

static void test_single_writes(void)
{
    static const UCHAR off[] = {5, 0xFF, 0xFF, 0, 0};
    static const UCHAR on[] = {5, 0xFF, 0xFF, 0xFF, 0};
    static const UCHAR reg[] = {6, 0xFF, 0xFF, 0x12, 0x34};
    prepare();
    CHECK(eMBMasterReqWriteCoil(1, 0xFFFF, 0, 0) == MB_MRE_NO_ERR);
    check_request(1, off, sizeof(off));
    prepare();
    CHECK(eMBMasterReqWriteCoil(0, 0xFFFF, 0xFF00, 0) == MB_MRE_NO_ERR);
    check_request(0, on, sizeof(on));
    prepare();
    CHECK(eMBMasterReqWriteHoldingRegister(0, 0xFFFF, 0x1234, 0) == MB_MRE_NO_ERR);
    check_request(0, reg, sizeof(reg));
    REJECT(eMBMasterReqWriteCoil(1, 0, 1, 0));
    REJECT(eMBMasterReqWriteCoil(1, 0, 0xFFFF, 0));
    REJECT(eMBMasterReqWriteCoil(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 0, 0));
    REJECT(eMBMasterReqWriteHoldingRegister(MB_MASTER_TOTAL_SLAVE_NUM + 1, 0, 0, 0));
    BUSY(eMBMasterReqWriteCoil(1, 0, 0, 0));
    BUSY(eMBMasterReqWriteHoldingRegister(1, 0, 0, 0));
}

static void test_transport(void)
{
    static const USHORT valid[] = {1, 250, 251, 252, 253};
    static const USHORT invalid[] = {0, 254, 255, 256, 0xFFFF};
    UCHAR expected[253];
    UCHAR external[256];
    size_t i;
    for (i = 0; i < sizeof(expected); ++i) expected[i] = (UCHAR)i;
    for (i = 0; i < sizeof(valid) / sizeof(valid[0]); ++i)
    {
        prepare();
        memcpy(pdu_buffer(), expected, valid[i]);
        CHECK(eMBMasterRTUSend(1, pdu_buffer(), valid[i]) == MB_ENOERR);
        drain_transmitter();
        check_wire(1, expected, valid[i]);
    }
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        prepare();
        CHECK(eMBMasterRTUSend(1, pdu_buffer(), invalid[i]) == MB_EINVAL);
        expect_untouched(0);
    }
    prepare();
    CHECK(eMBMasterRTUSend(1, NULL, 1) == MB_EINVAL);
    expect_untouched(0);
    prepare();
    CHECK(eMBMasterRTUSend(1, pdu_buffer() + 1, 1) == MB_EINVAL);
    expect_untouched(0);
    prepare();
    memset(external, 0x5A, sizeof(external));
    CHECK(eMBMasterRTUSend(1, external + 1, 1) == MB_EINVAL);
    for (i = 0; i < sizeof(external); ++i) CHECK(external[i] == 0x5A);
    expect_untouched(0);
    prepare();
    CHECK(eMBMasterRTUSend(MB_MASTER_TOTAL_SLAVE_NUM + 1, pdu_buffer(), 1) == MB_EINVAL);
    expect_untouched(0);
    prepare();
    eMBMasterRTUStart(); /* A valid request still fails when RX is not idle. */
    serial_calls = post_calls = 0;
    CHECK(eMBMasterRTUSend(1, pdu_buffer(), 1) == MB_EIO);
    expect_untouched(0);
    (void)xMBMasterRTUTimerExpired();
}

/* Run just the real request on an old source tree, without touching 256 bytes
 * in test setup. ASan must therefore report the production CRC/data write. */
static void reproduce(const char *which)
{
    USHORT *values = register_data(124);
    UCHAR coils[246] = {0};
    idle_transport();
    if (strcmp(which, "--repro-fc16") == 0)
        CHECK(eMBMasterReqWriteMultipleHoldingRegister(1, 0, 123, values, 0) == MB_MRE_NO_ERR);
    else if (strcmp(which, "--repro-fc23") == 0)
        CHECK(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 125, values, 0, 121, 0) == MB_MRE_NO_ERR);
    else if (strcmp(which, "--repro-fc15") == 0)
        CHECK(eMBMasterReqWriteMultipleCoils(1, 0, 1968, coils, 0) == MB_MRE_NO_ERR);
    else if (strcmp(which, "--repro-fc16-oversize") == 0)
        CHECK(eMBMasterReqWriteMultipleHoldingRegister(1, 0, 124, values, 0) == MB_MRE_ILL_ARG);
    else if (strcmp(which, "--repro-fc23-oversize") == 0)
        CHECK(eMBMasterReqReadWriteMultipleHoldingRegister(1, 0, 1, values, 0, 122, 0) == MB_MRE_ILL_ARG);
    else CHECK(0);
    free(values);
    puts("Reproducer passed with fixed sources.");
}

int main(int argc, char **argv)
{
    static const UCHAR check_vector[] = "123456789";
    CHECK(eMBMasterRTUInit(0, 115200, MB_PAR_NONE) == MB_ENOERR);
    CHECK(reference_crc(check_vector, 9) == 0x4B37);
    CHECK(usMBCRC16((UCHAR *)check_vector, 9) == 0x4B37);
    if (argc == 2) { reproduce(argv[1]); return 0; }
    CHECK(argc == 1);
    test_fc16();
    test_fc23();
    test_fc15();
    test_reads();
    test_single_writes();
    test_transport();
    printf("PASS: %u cases; real request builders, RTU transmit FSM and CRC.\n", test_cases);
    return 0;
}
