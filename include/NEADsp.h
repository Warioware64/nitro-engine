// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_DSP_H__
#define NEA_DSP_H__

#include <nds.h>

#include "dsp/nea_dsp_proto.h"

/// @file NEADsp.h
/// @brief Teak DSP coprocessor runtime (DSi only, with ARM9 fallbacks).

/// @defgroup dsp DSP coprocessor (DSi)
///
/// Runs NEA's DSP program on the DSi's Teak and hands it jobs.
///
/// @section dsp_what_you_gain What it buys you
///
/// The Teak runs at about 134 MHz alongside the ARM9. Its C compiler still
/// produces slow code, so NEA's DSP work is done by hand-written Teak assembly
/// kernels, which measured 2-10x faster than the ARM9 on hardware (libteak
/// dsp-bench). Every DSP-backed feature also has an ARM9 implementation that
/// gives the same output, so nothing in a game has to branch on which ran.
///
/// @section dsp_availability When the DSP is available
///
/// On DSi hardware, running a ROM with DSi privileges. `isDSiMode()` is **not**
/// a sufficient test: it is true for a DS-only ROM booted on a DSi, where the
/// DSP cannot be given memory. NEA_DspInit() checks `nwramIsAvailable()`.
///
/// libNEA.a must also have been built with `NEA_TEAK=1` and the Teak toolchain
/// installed. Otherwise NEA_DspInit() returns false.
///
/// @section dsp_transport Transports
///
/// On hardware the DSP moves data itself with DMA (AHBM), which costs the ARM9
/// nothing. melonDS does not emulate that path, so NEA_DspInit() probes it; if
/// only the FIFO works, the DSP answers but is not used for work unless
/// NEA_DspAllowFifoTransport() says so.
///
/// @section dsp_cost Cost
///
/// Starting the DSP hands all of NWRAM (blocks B and C, 512 KB) to it for as
/// long as it runs.
///
/// @{

/// Powers on the DSP, loads NEA's DSP program and finds a working transport.
///
/// Safe to call anywhere: on a DS, a DSi without the privileges, or a library
/// built without `NEA_TEAK=1`, it returns false.
///
/// @return true if the DSP is running and answered.
bool NEA_DspInit(void);

/// Powers the DSP down and releases NWRAM.
void NEA_DspEnd(void);

/// Restarts the DSP: powers it off, reloads NEA's program and probes the
/// transport again, keeping NEA_DspAllowFifoTransport() and
/// NEA_DspSetTransferConfig() settings.
///
/// A DSP DMA transfer that stalls can leave the DSP's bus access (AHBM) busy
/// for good, so that every later transfer fails too. A restart clears it.
/// DSP-backed features do this themselves after such a failure.
///
/// @return true if the DSP is running again.
bool NEA_DspReset(void);

/// Number of times the DSP was restarted by NEA_DspReset().
u32 NEA_DspGetResetCount(void);

/// Returns true if the DSP is running.
bool NEA_DspIsAvailable(void);

/// Round-trips a command to check that the DSP is alive.
bool NEA_DspPing(void);

/// How data moves between ARM9 memory and the DSP.
typedef enum {
    NEA_DSP_TRANSPORT_NONE = 0, ///< No working path; the DSP is unusable.
    NEA_DSP_TRANSPORT_DMA,      ///< The DSP moves data itself. Hardware.
    NEA_DSP_TRANSPORT_FIFO      ///< The ARM9 pushes and pulls. Emulators.
} NEA_DspTransport;

/// Returns the transport NEA_DspInit() settled on.
NEA_DspTransport NEA_DspGetTransport(void);

/// Name of the active transport: "DMA", "FIFO" or "none".
const char *NEA_DspGetTransportName(void);

/// Lets DSP-backed features use the DSP when only the FIFO transport works.
///
/// Off by default: over the FIFO the ARM9 moves every word itself, which
/// measured as a net loss (a 64x64 texture: 71% CPU against 29% on the ARM9).
/// Turn it on to exercise the DSP code paths in an emulator.
void NEA_DspAllowFifoTransport(bool allow);

/// True if DSP-backed features will actually send work to the DSP.
bool NEA_DspIsUsedForWork(void);

/// Round-trips data through the DSP with the default DMA settings.
///
/// @param words Number of 16-bit words (1 to NEA_DSP_SCRATCH_WORDS).
/// @return Number of words that came back wrong, or -1 on failure.
int NEA_DspSelfTest(int words);

/// What the last DMA self-test saw (the one NEA_DspInit() runs to choose the
/// transport, or the last NEA_DspSelfTest()). Diagnostic.
typedef struct {
    int status;     ///< Job status, NEA_DSP_STATUS_*, or -1 for no answer
    u32 result;     ///< Checksum of what the DSP read from ARM9 memory
    u32 expected;   ///< Checksum of what the ARM9 wrote
    int bad;        ///< Words that came back wrong
    int first_bad;  ///< Index of the first wrong word, or -1
    u16 got;        ///< The first wrong word
    u16 want;       ///< What it should have been
} NEA_DspProbeReport;

/// Returns what the last DMA self-test saw.
const NEA_DspProbeReport *NEA_DspGetProbeReport(void);

/// Replies discarded because their sequence number was stale.
u32 NEA_DspGetStaleReplies(void);

/// Commands the DSP received twice and ignored. Asks the DSP.
u32 NEA_DspGetRepeatedCommands(void);

// ---------------------------------------------------------------------------
// Jobs
// ---------------------------------------------------------------------------

/// A job descriptor. The DSP fetches it from ARM9 memory with DMA.
///
/// @warning Never declare one as a local variable. Locals live in DTCM, which
/// only the ARM9 core can see, so the DSP would read garbage. Use a static,
/// a global or heap memory. The same goes for every buffer a job names.
///
/// The layout is defined by the NEA_DSP_D_* word indices in
/// dsp/nea_dsp_proto.h. The alignment keeps it inside one 1 KB DMA page and
/// on whole cache lines.
typedef struct {
    u16 w[NEA_DSP_DESC_WORDS];
} __attribute__((aligned(64))) NEA_DspDesc;

/// Result of a job.
typedef struct {
    u16 status; ///< NEA_DSP_STATUS_*, or 0xFFFF if the DSP did not answer
    u32 result; ///< Job-specific value
    u32 cycles; ///< DSP cycles the job took (see NEA_DspCyclesToUs())
} NEA_DspJobResult;

/// Sets the DSP DMA settings jobs use (NEA_DSP_XFER_* burst and chunk bits).
///
/// The default, INCR8 bursts in 512-word chunks, is the fastest measured on a
/// DSi. The speed field is always forced to 0: other speeds corrupt data.
void NEA_DspSetTransferConfig(u16 xfer);

/// Returns the DSP DMA settings jobs use.
u16 NEA_DspGetTransferConfig(void);

/// Clears a descriptor and sets its job kind and the current transfer settings.
void NEA_DspDescInit(NEA_DspDesc *d, u16 kind);

/// Stores an ARM9 address in a descriptor, at a NEA_DSP_D_ADDR_* index.
static inline void NEA_DspDescSetAddr(NEA_DspDesc *d, int index,
                                      const void *addr)
{
    u32 a = (u32)addr;
    d->w[index] = a & 0xFFFF;
    d->w[index + 1] = a >> 16;
}

/// Starts a job and returns without waiting for it.
///
/// Only one job can be outstanding. The descriptor, and every buffer it names,
/// must stay valid until the job is collected. Buffers the DSP reads must have
/// been written back from the data cache (see DC_FlushRange()); buffers it
/// writes must be invalidated before the ARM9 reads them.
///
/// @return false if the DSP is unavailable or a job is already running.
bool NEA_DspJobBegin(NEA_DspDesc *d);

/// True while a job is outstanding.
bool NEA_DspJobIsPending(void);

/// True if the outstanding job has finished, so that collecting it won't wait.
bool NEA_DspJobIsDone(void);

/// Collects the outstanding job if it has finished. Never blocks.
///
/// @return true if the job finished and `r` was filled.
bool NEA_DspJobPoll(NEA_DspJobResult *r);

/// Waits for the outstanding job and collects it.
///
/// @return The job status, or -1 if the DSP did not answer in time.
int NEA_DspJobWait(NEA_DspJobResult *r);

/// Runs a job and waits for it.
///
/// @return The job status, or -1 on failure.
int NEA_DspJobRun(NEA_DspDesc *d, NEA_DspJobResult *r);

/// Converts DSP cycles to microseconds.
static inline u32 NEA_DspCyclesToUs(u32 cycles)
{
    return (u32)(((u64)cycles * 1000000) / NEA_DSP_CLOCK);
}

/// @}

// Internal, for other NEA modules

/// DSP word address of the scratch buffer (FIFO transport).
u16 __NEA_DspScratchAddr(void);

/// Sends NEA_DSP_CMD_INFO with an operand. Returns the status or -1.
int __NEA_DspInfo(u16 operand, u32 *value);

/// Stops the DSP's DMA for a moment, see NEA_DSP_SEM_GATE. Usable from
/// interrupts, and nests.
///
/// @return 1 when no DSP transfer is in flight and none will start until
///         __NEA_DspGateRelease(1); 0 if no DSP job is pending (nothing to
///         stop); -1 if the DSP didn't answer (nothing is held).
int __NEA_DspGateAcquire(void);

/// Releases what __NEA_DspGateAcquire() returned.
void __NEA_DspGateRelease(int token);

#endif // NEA_DSP_H__
