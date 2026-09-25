// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Nitro Engine Advanced contributors
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_DSP_PROTO_H__
#define NEA_DSP_PROTO_H__

/// @file nea_dsp_proto.h
/// @brief ARM9 <-> Teak DSP protocol, compiled by both toolchains.
///
/// This header is read by arm-none-eabi-gcc and by llvm-teak, so it holds
/// nothing but integer constants and inline arithmetic on fixed-width types.
///
/// @section dsp_proto_cmd Commands
///
/// Commands travel through the APBP command registers:
///
///     CMD1:CMD2  32-bit argument (high:low), written FIRST
///     CMD0       command | sequence number | operand, written LAST
///
/// The DSP waits for CMD0, then reads CMD1 and CMD2 directly. On hardware,
/// waiting for the "new" flags of CMD1/CMD2 returned stale values, and the DSP
/// sometimes saw the same CMD0 twice (libteak dsp-bench, measured), so:
///
/// - Every command has a sequence number (1 to 15). The DSP ignores a command
///   with the same number as the previous one.
/// - Every reply carries the sequence number of its command in REP0, and the
///   ARM9 discards replies with another one.
///
/// Replies: REP0 = sequence | NEA_DSP_REPLY_SECOND? | status, REP1:REP2 =
/// 32-bit value (high:low). A job sends two replies: its result, then the DSP
/// cycles it took.
///
/// @section dsp_proto_desc Job descriptors
///
/// A job's parameters do not fit in one 32-bit argument, and uploading them
/// through the command registers costs a round trip per two words. So the
/// argument of NEA_DSP_CMD_JOB is the ARM9 address of a descriptor, and the
/// DSP fetches it with one DMA transfer.
///
/// A descriptor is an array of NEA_DSP_DESC_WORDS 16-bit words rather than a
/// struct, because the two compilers need not agree on struct layout and the
/// DSP addresses 16-bit words. 32-bit values are stored low half first, which
/// is how the ARM9 stores a u32, so an ARM9 side can write them as u32 at an
/// even word index.

#include <stdint.h>

// CMD0 layout
#define NEA_DSP_CMD_MASK        0xF000
#define NEA_DSP_SEQ_SHIFT       8
#define NEA_DSP_SEQ_MASK        (0xF << NEA_DSP_SEQ_SHIFT)
#define NEA_DSP_OPERAND_MASK    0x00FF

// Commands

/// Reply: status, number of repeated commands the DSP ignored.
#define NEA_DSP_CMD_STATS       0x1000
/// Reply: status, NEA_DSP_MAGIC.
#define NEA_DSP_CMD_PING        0x2000
/// Runs a job. Operand = job kind (NEA_DSP_JOB_*), argument = ARM9 address of
/// its descriptor, or 0 if the ARM9 already wrote it into DSP memory through
/// the FIFO. Reply 1: status, job result. Reply 2 (NEA_DSP_REPLY_SECOND):
/// status, DSP cycles the job took.
#define NEA_DSP_CMD_JOB         0x3000
/// Operand 0: reply = DSP word addresses of the descriptor (high half) and of
/// the scratch buffer (low half). Operand NEA_DSP_INFO_GRADE: reply = DSP word
/// address of the colour grading table blob. The FIFO transport writes and
/// reads them directly, because it has no DMA.
#define NEA_DSP_CMD_INFO        0x4000

#define NEA_DSP_INFO_BUFFERS    0
#define NEA_DSP_INFO_GRADE      1
/// Reply: ring of source rows (high half) and table blob (low half) of the
/// displacement job.
#define NEA_DSP_INFO_DISPLACE   2
/// Reply: the blur job's work area (see NEA_FX_BLUR_WA_*).
#define NEA_DSP_INFO_BLUR       3
/// Reply: the bloom job's work area (see NEA_FX_BLOOM_WA_*).
#define NEA_DSP_INFO_BLOOM      4

// DMA gate (APBP semaphores)
//
// On a DSi, a DMA feeding the GFX FIFO (legacy or NDMA) while the DSP moves
// data with its own DMA wedges the DSP's bus access for good. So before the
// ARM9 sends a display list by DMA during a DSP job, it raises GATE; the DSP
// finishes its transfer in flight, starts no new one and raises GATE_ACK; the
// ARM9 sends the display list, drops GATE and waits for GATE_ACK to drop.
// The DSP keeps computing meanwhile, and drops GATE_ACK at the end of a job.
#define NEA_DSP_SEM_GATE        0x0001  ///< ARM9 -> DSP: start no transfer
#define NEA_DSP_SEM_GATE_ACK    0x0001  ///< DSP -> ARM9: none in flight

/// DSP -> ARM9: the job in flight has its own copy of the tables it was given,
/// so the ARM9 may rebuild them. Cleared at the end of every job.
#define NEA_DSP_SEM_TABLES_READ 0x0002

// REP0 layout
#define NEA_DSP_REPLY_SECOND    0x1000
#define NEA_DSP_STATUS_MASK     0x00FF

#define NEA_DSP_STATUS_OK           0
#define NEA_DSP_STATUS_BAD_CMD      1
#define NEA_DSP_STATUS_BAD_ARG      2
#define NEA_DSP_STATUS_DMA_ERROR    3 ///< A transfer could not start or timed out
#define NEA_DSP_STATUS_MISMATCH     4 ///< Data did not arrive intact

/// "NEA" + protocol version, returned by NEA_DSP_CMD_PING.
#define NEA_DSP_MAGIC           0x4E454101u

/// DSP clock of the DSi, in Hz. DSP timer 0 counts DSP cycles.
#define NEA_DSP_CLOCK           134055928u

/// Size of a job descriptor, in 16-bit words. 64 bytes: two cache lines, and
/// with 64-byte alignment it can never straddle a 1 KB DMA page.
#define NEA_DSP_DESC_WORDS      32

/// Size of the DSP-side scratch buffer, in 16-bit words (8 KB). Images are
/// processed in pieces of this size.
#define NEA_DSP_SCRATCH_WORDS   4096

/// Largest DSP DMA transfer, in words. Transfers also must not cross a 1 KB
/// boundary on the ARM9 side.
#define NEA_DSP_DMA_MAX_WORDS   512

// Job kinds
// ---------

/// Transfer self-test: the DSP reads `words` words from A, writes each one
/// XOR NEA_DSP_SELFTEST_XOR to B, and returns the checksum of what it read.
#define NEA_DSP_JOB_SELFTEST    1
/// Transfer benchmark, see NEA_DSP_XB_*.
#define NEA_DSP_JOB_XFER_BENCH  2
/// Does nothing. Measures the command round trip.
#define NEA_DSP_JOB_NOP         3

/// Colour grading, see NEA_DSP_GR_* and dsp/nea_fx_grade.h.
#define NEA_DSP_JOB_GRADE       4

/// Displacement, see NEA_DSP_DP_* and dsp/nea_fx_displace.h.
#define NEA_DSP_JOB_DISPLACE    5
/// Blur, see NEA_DSP_BL_* and dsp/nea_fx_blur.h.
#define NEA_DSP_JOB_BLUR        6
/// Bloom, see NEA_DSP_BM_* and dsp/nea_fx_bloom.h.
#define NEA_DSP_JOB_BLOOM       7

#define NEA_DSP_SELFTEST_XOR    0xA55A

// NEA_DSP_D_FLAGS
#define NEA_DSP_FLAG_LOCAL          0x0001  ///< Pixels are already in scratch
#define NEA_DSP_FLAG_TABLES_LOCAL   0x0002  ///< Tables are already in place
#define NEA_DSP_FLAG_NO_OVERLAP     0x0004  ///< Transfers wait for the kernel

// Descriptor words common to every job
#define NEA_DSP_D_KIND          0   ///< Job kind, checked against the operand
#define NEA_DSP_D_FLAGS         1
#define NEA_DSP_D_ADDR_A        2   ///< 32-bit ARM9 address (words 2-3)
#define NEA_DSP_D_ADDR_B        4   ///< 32-bit ARM9 address (words 4-5)
#define NEA_DSP_D_WORDS         6   ///< Element count
#define NEA_DSP_D_XFER          7   ///< DMA configuration, NEA_DSP_XFER_*
#define NEA_DSP_D_PARAM         8   ///< First job-specific word

// DMA configuration word (NEA_DSP_D_XFER)
//
// libteak always uses speed 0, INCR bursts and 512-word chunks. Measured on a
// DSi with NEA_DSP_JOB_XFER_BENCH (examples/dsp/dsp_bench_transport), per
// 32 KB of main RAM, ARM9 -> DSP / DSP -> ARM9:
//
//   INCR   7431 / 3979 us     (4.4 / 8.2 MB/s)
//   INCR4  2179 / 2262 us
//   INCR8  1446 / 1420 us     (22.1 / 22.5 MB/s)
//
// Speeds 1-3 take exactly as long as speed 0 and deliver WRONG data in both
// directions, for every burst type. The field is kept only so the bench can
// keep checking that; nothing else may set it.
//
// Chunks: 64 words 14.8 MB/s, 128 18.2, 256 20.6, 512 22.1.
//
// VRAM in LCD mode is faster than main RAM: 26.6 MB/s read, 40.2 MB/s write.
// The ARM9's own reads of main RAM measured no slowdown while the DSP moved a
// 96 KB frame (6645 us idle, 6609 us during).
#define NEA_DSP_XFER_SPEED_SHIFT    0   ///< Must be 0 (see above)
#define NEA_DSP_XFER_SPEED_MASK     (3 << NEA_DSP_XFER_SPEED_SHIFT)
#define NEA_DSP_XFER_BURST_SHIFT    2   ///< 0 INCR, 1 INCR4, 2 INCR8
#define NEA_DSP_XFER_BURST_MASK     (3 << NEA_DSP_XFER_BURST_SHIFT)
#define NEA_DSP_XFER_BURST_INCR     (0 << NEA_DSP_XFER_BURST_SHIFT)
#define NEA_DSP_XFER_BURST_INCR4    (1 << NEA_DSP_XFER_BURST_SHIFT)
#define NEA_DSP_XFER_BURST_INCR8    (2 << NEA_DSP_XFER_BURST_SHIFT)
#define NEA_DSP_XFER_CHUNK_SHIFT    4   ///< Chunk = 64 << n words (n = 0..3)
#define NEA_DSP_XFER_CHUNK_MASK     (3 << NEA_DSP_XFER_CHUNK_SHIFT)

/// Default: speed 0, INCR8 bursts, 512-word chunks. The fastest setting that
/// delivers correct data, 5x libteak's INCR for ARM9 -> DSP.
#define NEA_DSP_XFER_DEFAULT        (NEA_DSP_XFER_BURST_INCR8 \
                                     | (3 << NEA_DSP_XFER_CHUNK_SHIFT))

static inline uint16_t nea_dsp_xfer_chunk_words(uint16_t xfer)
{
    return (uint16_t)(64u << ((xfer & NEA_DSP_XFER_CHUNK_MASK)
                              >> NEA_DSP_XFER_CHUNK_SHIFT));
}

// NEA_DSP_JOB_XFER_BENCH
//
// Moves D_WORDS words between ARM9 address A and the DSP scratch buffer (which
// wraps every NEA_DSP_SCRATCH_WORDS). Only the DMA time is counted: the job
// result is the checksum, and the second reply is the sum of the DSP cycles
// spent from starting each chunk to seeing it complete.
//
// ARM9 -> DSP: A holds nea_dsp_bench_pattern(i, seed); the result is the
//              number of words that did not match.
// DSP -> ARM9: the DSP writes nea_dsp_bench_pattern(i, seed) to A; the ARM9
//              checks it. The result is 0.
#define NEA_DSP_XB_DIR          (NEA_DSP_D_PARAM + 0) ///< 0 in, 1 out
#define NEA_DSP_XB_SEED         (NEA_DSP_D_PARAM + 1)

#define NEA_DSP_XB_DIR_IN       0   ///< ARM9 -> DSP
#define NEA_DSP_XB_DIR_OUT      1   ///< DSP -> ARM9

// NEA_DSP_JOB_GRADE
//
// Grades D_WORDS pixels from A to B (both 16-bit RGB555, B may equal A). The
// tables (NEA_FX_GRADE_WORDS words) are fetched from GR_TABLES only when
// GR_GEN differs from the generation the DSP holds. Result: DSP cycles spent in
// the kernel alone; the second reply has the whole job.
//
// With NEA_DSP_FLAG_LOCAL, the pixels are in the scratch buffer (at most
// NEA_DSP_SCRATCH_WORDS) and are graded in place. With
// NEA_DSP_FLAG_TABLES_LOCAL, the ARM9 wrote the tables itself.
#define NEA_DSP_GR_TABLES       (NEA_DSP_D_PARAM + 0)   ///< 32-bit address
#define NEA_DSP_GR_GEN          (NEA_DSP_D_PARAM + 2)   ///< Nonzero
// The image is a rectangle: rows of GR_WIDTH pixels, GR_SRC_STRIDE and
// GR_DST_STRIDE pixels apart in memory, D_WORDS pixels in total (a whole
// number of rows). A contiguous run of N pixels is one row: width = stride = N.
// Rectangles need the overlapped path (not NEA_DSP_FLAG_NO_OVERLAP) unless
// they are contiguous.
#define NEA_DSP_GR_WIDTH        (NEA_DSP_D_PARAM + 3)
#define NEA_DSP_GR_SRC_STRIDE   (NEA_DSP_D_PARAM + 4)
#define NEA_DSP_GR_DST_STRIDE   (NEA_DSP_D_PARAM + 5)

// NEA_DSP_JOB_DISPLACE
//
// Displaces a D_WORDS x DP_HEIGHT image (16-bit, width <= 256, height <= 192)
// from A to B (which must not overlap). Rows are DP_SRC_STRIDE and
// DP_DST_STRIDE pixels apart. The tables (NEA_FX_DISP_WORDS words) are fetched
// from DP_TABLES every job. Result: DSP cycles spent in the kernel.
//
// With NEA_DSP_FLAG_LOCAL (emulators): the ARM9 wrote the tables into the
// blob and the source rows into the ring slots (at NEA_FX_DISP_PAD), at most
// NEA_FX_DISP_RING_ROWS rows, and reads the result from the scratch buffer,
// rows packed D_WORDS apart.
#define NEA_DSP_DP_HEIGHT       (NEA_DSP_D_PARAM + 0)
/// Signed: a negative stride reads the source rows upwards (a mirror), from A
/// as the first row read.
#define NEA_DSP_DP_SRC_STRIDE   (NEA_DSP_D_PARAM + 1)
#define NEA_DSP_DP_DST_STRIDE   (NEA_DSP_D_PARAM + 2)
#define NEA_DSP_DP_TABLES       (NEA_DSP_D_PARAM + 3)   ///< 32-bit address
/// Source row length in pixels (<= 256), which may differ from the output
/// width D_WORDS: the XA table maps each output column to a source column.
#define NEA_DSP_DP_SRC_W        (NEA_DSP_D_PARAM + 5)

// NEA_DSP_JOB_BLUR
//
// Blurs a D_WORDS x BL_HEIGHT image (16-bit, width <= 256, height <= 192)
// from A to B; B may be A. Rows are BL_SRC_STRIDE and BL_DST_STRIDE pixels
// apart. Result: DSP cycles spent in the kernels. The job uses the colour
// grading job's LRG table as its work area, which the next grading job
// rebuilds.
//
// With NEA_DSP_FLAG_LOCAL (emulators): the image is in the scratch buffer,
// rows packed D_WORDS apart (at most NEA_DSP_SCRATCH_WORDS pixels), and the
// result goes to the work area at NEA_FX_BLUR_WA_LOCAL_OUT, packed the same.
#define NEA_DSP_BL_HEIGHT       (NEA_DSP_D_PARAM + 0)
#define NEA_DSP_BL_SRC_STRIDE   (NEA_DSP_D_PARAM + 1)
#define NEA_DSP_BL_DST_STRIDE   (NEA_DSP_D_PARAM + 2)

// NEA_DSP_JOB_BLOOM
//
// Adds bloom to a D_WORDS x BM_HEIGHT image (16-bit; width and height even,
// at most 256x192) from A to B; B may be A. Rows are BM_SRC_STRIDE and
// BM_DST_STRIDE pixels apart; BM_THRESHOLD is the bright-pass threshold
// (0-31). Result: DSP cycles spent in the kernels. Like blur, the job uses the
// colour grading LRG table as its work area.
//
// With NEA_DSP_FLAG_LOCAL (emulators): the image is in the scratch buffer,
// rows packed D_WORDS apart, at most 16 rows, and the result goes to the work
// area at NEA_FX_BLOOM_WA_LOCAL_OUT, packed the same.
#define NEA_DSP_BM_HEIGHT       (NEA_DSP_D_PARAM + 0)
#define NEA_DSP_BM_SRC_STRIDE   (NEA_DSP_D_PARAM + 1)
#define NEA_DSP_BM_DST_STRIDE   (NEA_DSP_D_PARAM + 2)
#define NEA_DSP_BM_THRESHOLD    (NEA_DSP_D_PARAM + 3)

/// Test pattern shared by both sides. Additions and XOR only, so the Teak
/// generates it without a multiply, and it changes every bit position across
/// neighbouring words so a stuck or shifted line shows up.
static inline uint16_t nea_dsp_bench_pattern(uint16_t i, uint16_t seed)
{
    uint16_t v = (uint16_t)(i + seed);
    return (uint16_t)(v ^ (uint16_t)(v << 7) ^ (uint16_t)(i >> 3));
}

// Checksum (Fletcher-style) computed the same way on both CPUs from values,
// not memory layout.
typedef struct {
    uint32_t a, b;
} nea_dsp_checksum;

static inline void nea_dsp_checksum_init(nea_dsp_checksum *c)
{
    c->a = 1;
    c->b = 0;
}

static inline void nea_dsp_checksum_add16(nea_dsp_checksum *c, uint16_t v)
{
    c->a += v;
    c->b += c->a;
}

static inline uint32_t nea_dsp_checksum_get(const nea_dsp_checksum *c)
{
    return (c->b << 16) ^ c->a;
}

#endif // NEA_DSP_PROTO_H__
