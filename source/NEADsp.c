// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#include <malloc.h>

#include "NEAMain.h"

/// @file NEADsp.c

#ifdef NEA_TEAK
#include "nea_dsp_tlf_bin.h"
#endif

static bool ne_dsp_available = false;
static NEA_DspTransport ne_dsp_transport = NEA_DSP_TRANSPORT_NONE;
static bool ne_dsp_allow_fifo = false;

// DSP word addresses of the descriptor and the scratch buffer (CMD_INFO). The
// FIFO transport writes and reads them directly.
static u16 ne_dsp_desc_addr;
#ifdef NEA_TEAK
static u16 ne_dsp_scratch_addr;
#endif

// Sequence number of the last command, already shifted into place
static u16 ne_dsp_cmd_seq;
static u32 ne_dsp_stale_replies;

static bool ne_dsp_job_pending;

// DTCM bounds, from the BlocksDS linker script. The stack lives there.
extern char __dtcm_start[], __dtcm_top[];

// A dead or wedged DSP must not hang the ROM. Replies normally take a few
// hundred cycles and the longest job a few milliseconds; this bound is seconds.
#define NE_DSP_TIMEOUT 0x2000000

// Buffers of the self-test and of the DMA probe in NEA_DspInit()
#define NE_DSP_PROBE_WORDS 256

//-----------------------------------------------------------------------------
// Command channel
//-----------------------------------------------------------------------------

static void ne_dsp_send(u16 cmd, u32 arg)
{
    static u16 seq = 0;

    seq = (seq % 15) + 1;
    ne_dsp_cmd_seq = seq << NEA_DSP_SEQ_SHIFT;

    // The argument goes first: the DSP reads it when it sees CMD0
    dspSendData(1, arg >> 16);
    dspSendData(2, arg & 0xFFFF);
    dspSendData(0, cmd | ne_dsp_cmd_seq);
}

// Reads one reply that is known to be ready (or about to be) and checks that it
// belongs to the last command. Returns the status, or -1 for a stale reply.
static int ne_dsp_read_reply(u16 part, u32 *value)
{
    u16 rep0 = dspReceiveData(0);
    u32 v = (u32)dspReceiveData(1) << 16;
    v |= dspReceiveData(2);

    if ((rep0 & (NEA_DSP_SEQ_MASK | NEA_DSP_REPLY_SECOND))
        != (ne_dsp_cmd_seq | part))
    {
        ne_dsp_stale_replies++;
        return -1;
    }

    if (value)
        *value = v;
    return rep0 & NEA_DSP_STATUS_MASK;
}

// Waits for reply "part" of the last command. Returns its status, or -1 on
// timeout.
static int ne_dsp_receive(u16 part, u32 *value)
{
    int timeout = NE_DSP_TIMEOUT;

    while (1)
    {
        while (!dspReceiveDataReady(0))
        {
            if (--timeout <= 0)
                return -1;
        }

        int status = ne_dsp_read_reply(part, value);
        if (status >= 0)
            return status;
    }
}

static int ne_dsp_command(u16 cmd, u32 arg, u32 *value)
{
    ne_dsp_send(cmd, arg);
    return ne_dsp_receive(0, value);
}

//-----------------------------------------------------------------------------
// Jobs
//-----------------------------------------------------------------------------

static u16 ne_dsp_xfer = NEA_DSP_XFER_DEFAULT;

void NEA_DspSetTransferConfig(u16 xfer)
{
    // Speeds other than 0 corrupt data on a DSi
    ne_dsp_xfer = xfer & ~NEA_DSP_XFER_SPEED_MASK;
}

u16 NEA_DspGetTransferConfig(void)
{
    return ne_dsp_xfer;
}

void NEA_DspDescInit(NEA_DspDesc *d, u16 kind)
{
    NEA_AssertPointer(d, "NULL descriptor");

    memset(d, 0, sizeof(*d));
    d->w[NEA_DSP_D_KIND] = kind;
    d->w[NEA_DSP_D_XFER] = ne_dsp_xfer;
}

// Starts a job regardless of the transport policy. The self-test and the
// transport probe need that.
static bool ne_dsp_job_begin(NEA_DspDesc *d)
{
    if (!ne_dsp_available || ne_dsp_job_pending)
        return false;

    u16 kind = d->w[NEA_DSP_D_KIND];
    u32 arg;

    if (ne_dsp_transport == NEA_DSP_TRANSPORT_FIFO)
    {
        // No DMA: write the descriptor straight into DSP memory, and tell the
        // DSP it is already there.
        dspFifoWriteData(d->w, ne_dsp_desc_addr, NEA_DSP_DESC_WORDS);
        arg = 0;
    }
    else
    {
        DC_FlushRange(d, sizeof(*d));
        arg = (u32)d;
    }

    ne_dsp_send(NEA_DSP_CMD_JOB | (kind & NEA_DSP_OPERAND_MASK), arg);
    ne_dsp_job_pending = true;
    return true;
}

bool NEA_DspJobBegin(NEA_DspDesc *d)
{
    NEA_AssertPointer(d, "NULL descriptor");

    // DTCM is private to the ARM9 core: the DSP's DMA would read garbage from
    // that address. The stack lives in DTCM, so this is a descriptor declared
    // as a local variable.
    NEA_Assert(!((char *)d >= __dtcm_start && (char *)d < __dtcm_top),
               "Descriptor in DTCM (a local variable?), the DSP can't read it");

    if (!NEA_DspIsUsedForWork())
        return false;

    return ne_dsp_job_begin(d);
}

bool NEA_DspJobIsPending(void)
{
    return ne_dsp_job_pending;
}

bool NEA_DspJobIsDone(void)
{
    // The reply is there: collecting it won't wait
    return ne_dsp_job_pending && dspReceiveDataReady(0);
}

bool NEA_DspJobPoll(NEA_DspJobResult *r)
{
    NEA_AssertPointer(r, "NULL result");

    if (!ne_dsp_job_pending)
        return false;

    while (dspReceiveDataReady(0))
    {
        int status = ne_dsp_read_reply(0, &r->result);
        if (status < 0)
            continue; // Stale, look again

        // The second reply follows the first immediately
        int status2 = ne_dsp_receive(NEA_DSP_REPLY_SECOND, &r->cycles);

        r->status = (status2 < 0) ? 0xFFFF : (u16)status;
        ne_dsp_job_pending = false;
        return true;
    }

    return false;
}

int NEA_DspJobWait(NEA_DspJobResult *r)
{
    NEA_AssertPointer(r, "NULL result");

    if (!ne_dsp_job_pending)
        return -1;

    int status = ne_dsp_receive(0, &r->result);
    if (status >= 0)
    {
        if (ne_dsp_receive(NEA_DSP_REPLY_SECOND, &r->cycles) < 0)
            status = -1;
    }

    ne_dsp_job_pending = false;

    r->status = (status < 0) ? 0xFFFF : (u16)status;
    return status;
}

int NEA_DspJobRun(NEA_DspDesc *d, NEA_DspJobResult *r)
{
    if (!NEA_DspJobBegin(d))
        return -1;
    return NEA_DspJobWait(r);
}

//-----------------------------------------------------------------------------
// Initialization and transport selection
//-----------------------------------------------------------------------------

// Round-trips "words" words through the DSP with DMA. Returns the number of
// words that came back wrong, or -1 on failure.
// What the last DMA self-test saw, for NEA_DspGetProbeReport()
static NEA_DspProbeReport ne_dsp_probe_report;

static int ne_dsp_selftest_dma(u16 *src, u16 *dst, int words)
{
    NEA_DspProbeReport *rep = &ne_dsp_probe_report;
    memset(rep, 0, sizeof(*rep));
    rep->first_bad = -1;

    nea_dsp_checksum sum;
    nea_dsp_checksum_init(&sum);

    for (int i = 0; i < words; i++)
    {
        src[i] = nea_dsp_bench_pattern(i, 0x1234);
        dst[i] = 0;
        nea_dsp_checksum_add16(&sum, src[i]);
    }

    DC_FlushRange(src, words * sizeof(u16));
    DC_FlushRange(dst, words * sizeof(u16));

    // Static, not a local: locals live in DTCM, which the DSP can't reach
    static NEA_DspDesc d;
    NEA_DspDescInit(&d, NEA_DSP_JOB_SELFTEST);
    NEA_DspDescSetAddr(&d, NEA_DSP_D_ADDR_A, src);
    NEA_DspDescSetAddr(&d, NEA_DSP_D_ADDR_B, dst);
    d.w[NEA_DSP_D_WORDS] = words;

    NEA_DspJobResult r;
    if (!ne_dsp_job_begin(&d))
        return -1;
    int status = NEA_DspJobWait(&r);

    DC_InvalidateRange(dst, words * sizeof(u16));

    rep->status = status;
    rep->result = r.result;
    rep->expected = nea_dsp_checksum_get(&sum);

    int bad = 0;
    for (int i = 0; i < words; i++)
    {
        u16 want = src[i] ^ NEA_DSP_SELFTEST_XOR;
        if (dst[i] != want)
        {
            if (bad == 0)
            {
                rep->first_bad = i;
                rep->got = dst[i];
                rep->want = want;
            }
            bad++;
        }
    }
    rep->bad = bad;

    if (status < 0 || status == NEA_DSP_STATUS_DMA_ERROR)
        return -1;

    // The checksum covers the ARM9 -> DSP direction on its own
    if (bad == 0 && r.result != rep->expected)
        bad = 1;

    return bad;
}

const NEA_DspProbeReport *NEA_DspGetProbeReport(void)
{
    return &ne_dsp_probe_report;
}

#ifdef NEA_TEAK

// Buffers of the DMA probe. 1 KB aligned so that they are whole cache lines
// and each one sits inside a single DMA page.
static u16 ne_dsp_probe_src[NE_DSP_PROBE_WORDS] __attribute__((aligned(1024)));
static u16 ne_dsp_probe_dst[NE_DSP_PROBE_WORDS] __attribute__((aligned(1024)));

// Writes a pattern into the scratch buffer and reads it back through the FIFO.
static bool ne_dsp_fifo_works(void)
{
    u16 *buf = ne_dsp_probe_src;

    for (int i = 0; i < NE_DSP_PROBE_WORDS; i++)
        buf[i] = nea_dsp_bench_pattern(i, 0x4321);

    dspFifoWriteData(buf, ne_dsp_scratch_addr, NE_DSP_PROBE_WORDS);

    u16 *back = ne_dsp_probe_dst;
    memset(back, 0, NE_DSP_PROBE_WORDS * sizeof(u16));
    dspFifoReadData(ne_dsp_scratch_addr, back, NE_DSP_PROBE_WORDS);

    return memcmp(buf, back, NE_DSP_PROBE_WORDS * sizeof(u16)) == 0;
}

static void ne_dsp_probe_transport(void)
{
    // Try the path hardware uses. The descriptor itself travels by DMA, so if
    // DMA does not work the job fails early with BAD_ARG; any failure counts.
    ne_dsp_transport = NEA_DSP_TRANSPORT_DMA;
    int bad = ne_dsp_selftest_dma(ne_dsp_probe_src, ne_dsp_probe_dst,
                                  NE_DSP_PROBE_WORDS);
    if (bad == 0)
        return;

    NEA_DebugPrint("DSP DMA probe failed (%d), trying the FIFO", bad);

    ne_dsp_transport = NEA_DSP_TRANSPORT_FIFO;
    if (ne_dsp_fifo_works())
        return;

    NEA_DebugPrint("DSP FIFO probe failed too");
    ne_dsp_transport = NEA_DSP_TRANSPORT_NONE;
}

#endif // NEA_TEAK

bool NEA_DspPing(void)
{
    if (!ne_dsp_available || ne_dsp_job_pending)
        return false;

    u32 value = 0;
    return ne_dsp_command(NEA_DSP_CMD_PING, 0, &value) == NEA_DSP_STATUS_OK
           && value == NEA_DSP_MAGIC;
}

bool NEA_DspInit(void)
{
#ifndef NEA_TEAK
    NEA_DebugPrint("Built without NEA_TEAK, DSP unavailable");
    return false;
#else
    if (ne_dsp_available)
        return true;

    // isDSiMode() is not enough: it is true for a DS-only ROM on a DSi, where
    // NWRAM cannot be handed to the DSP.
    if (!nwramIsAvailable())
    {
        NEA_DebugPrint("NWRAM unavailable, DSP disabled");
        return false;
    }

    DSPExecResult res = dspExecuteDefaultTLF(nea_dsp_tlf_bin);
    if (res != DSP_EXEC_OK)
    {
        NEA_DebugPrint("dspExecuteDefaultTLF failed: %d", (int)res);
        return false;
    }

    ne_dsp_available = true;
    ne_dsp_job_pending = false;

    // Loading the TLF only proves the loader accepted it
    if (!NEA_DspPing())
    {
        NEA_DebugPrint("DSP loaded but did not answer a ping");
        NEA_DspEnd();
        return false;
    }

    u32 info = 0;
    if (ne_dsp_command(NEA_DSP_CMD_INFO, 0, &info) != NEA_DSP_STATUS_OK)
    {
        NEA_DebugPrint("DSP did not report its buffers");
        NEA_DspEnd();
        return false;
    }
    ne_dsp_desc_addr = info >> 16;
    ne_dsp_scratch_addr = info & 0xFFFF;

    ne_dsp_probe_transport();
    if (ne_dsp_transport == NEA_DSP_TRANSPORT_NONE)
    {
        NEA_DspEnd();
        return false;
    }

    return true;
#endif
}

void NEA_DspEnd(void)
{
#ifdef NEA_TEAK
    if (!ne_dsp_available)
        return;

    dspPowerOff();
    ne_dsp_available = false;
    ne_dsp_job_pending = false;
    ne_dsp_transport = NEA_DSP_TRANSPORT_NONE;
#endif
}

static u32 ne_dsp_resets;

bool NEA_DspReset(void)
{
    if (!ne_dsp_available)
        return false;

    // Keep the settings the game chose; everything else starts over, including
    // the DSP's DMA and AHBM, which a stalled transfer can leave wedged.
    bool allow_fifo = ne_dsp_allow_fifo;
    u16 xfer = ne_dsp_xfer;

    NEA_DspEnd();
    ne_dsp_resets++;
    bool ok = NEA_DspInit();

    ne_dsp_allow_fifo = allow_fifo;
    ne_dsp_xfer = xfer;
    return ok;
}

u32 NEA_DspGetResetCount(void)
{
    return ne_dsp_resets;
}

bool NEA_DspIsAvailable(void)
{
    return ne_dsp_available;
}

NEA_DspTransport NEA_DspGetTransport(void)
{
    return ne_dsp_transport;
}

const char *NEA_DspGetTransportName(void)
{
    switch (ne_dsp_transport)
    {
        case NEA_DSP_TRANSPORT_DMA:
            return "DMA";
        case NEA_DSP_TRANSPORT_FIFO:
            return "FIFO";
        default:
            return "none";
    }
}

void NEA_DspAllowFifoTransport(bool allow)
{
    ne_dsp_allow_fifo = allow;
}

bool NEA_DspIsUsedForWork(void)
{
    if (!ne_dsp_available)
        return false;

    if (ne_dsp_transport == NEA_DSP_TRANSPORT_DMA)
        return true;

    return ne_dsp_transport == NEA_DSP_TRANSPORT_FIFO && ne_dsp_allow_fifo;
}

int NEA_DspSelfTest(int words)
{
    if (ne_dsp_transport != NEA_DSP_TRANSPORT_DMA)
        return -1;

    if (words < 1 || words > NEA_DSP_SCRATCH_WORDS)
        return -1;

    size_t size = words * sizeof(u16);
    u16 *src = memalign(1024, size);
    u16 *dst = memalign(1024, size);

    int bad = -1;
    if (src && dst)
        bad = ne_dsp_selftest_dma(src, dst, words);

    free(src);
    free(dst);
    return bad;
}

// Internal, for other NEA modules
// -------------------------------

// Polls before giving up on the gate. The DSP answers within a transfer
// (~45 us) plus a kernel block (~30 us); this is a few milliseconds.
#define NE_DSP_GATE_TIMEOUT 0x8000

// True once the job in flight has replied: then the DSP starts no transfer
// until the next job, gate or not.
static bool ne_dsp_job_replied(void)
{
    return dspReceiveDataReady(0);
}

// Holders of the gate. It can be taken by the main code (display lists) and by
// an interrupt (the VBlank handler's DMA work) at the same time, so it is
// counted, and the DSP only resumes when the last holder lets go.
static volatile int ne_dsp_gate_depth;

static bool ne_dsp_gate_wait_ack(void)
{
    for (int n = NE_DSP_GATE_TIMEOUT; n > 0; n--)
    {
        if ((dspGetSemaphore() & NEA_DSP_SEM_GATE_ACK) || ne_dsp_job_replied())
            return true;
    }
    return false;
}

static void ne_dsp_gate_drop(void)
{
    int ime = enterCriticalSection();
    bool last = --ne_dsp_gate_depth == 0;
    if (last)
        apbpClearSemaphore(NEA_DSP_SEM_GATE);
    leaveCriticalSection(ime);

    if (!last || !ne_dsp_job_pending)
        return;

    // Wait for the DSP to see it, so that the next acquire can't mistake this
    // acknowledgement for a new one
    for (int n = NE_DSP_GATE_TIMEOUT; n > 0; n--)
    {
        if (!(dspGetSemaphore() & NEA_DSP_SEM_GATE_ACK) || ne_dsp_job_replied())
            return;
    }
}

int __NEA_DspGateAcquire(void)
{
    if (!ne_dsp_job_pending)
        return 0;

    int ime = enterCriticalSection();
    ne_dsp_gate_depth++;
    dspSetSemaphore(NEA_DSP_SEM_GATE);
    leaveCriticalSection(ime);

    // Every holder waits for the acknowledgement, including a nested one: an
    // interrupt that takes the gate while the main code is still waiting for
    // it must not go ahead on its own.
    if (ne_dsp_gate_wait_ack())
        return 1;

    // No answer: let go, and let the caller use a path that needs no gate
    ne_dsp_gate_drop();
    return -1;
}

void __NEA_DspGateRelease(int token)
{
    if (token == 1)
        ne_dsp_gate_drop();
}

u16 __NEA_DspScratchAddr(void)
{
#ifdef NEA_TEAK
    return ne_dsp_scratch_addr;
#else
    return 0;
#endif
}

int __NEA_DspInfo(u16 operand, u32 *value)
{
    if (!ne_dsp_available || ne_dsp_job_pending)
        return -1;

    return ne_dsp_command(NEA_DSP_CMD_INFO | (operand & NEA_DSP_OPERAND_MASK),
                          0, value);
}

u32 NEA_DspGetStaleReplies(void)
{
    return ne_dsp_stale_replies;
}

u32 NEA_DspGetRepeatedCommands(void)
{
    if (!ne_dsp_available || ne_dsp_job_pending)
        return 0;

    u32 value = 0;
    ne_dsp_command(NEA_DSP_CMD_STATS, 0, &value);
    return value;
}
