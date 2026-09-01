// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_PATTERN_H__
#define NEA_PATTERN_H__

/// @file   NEAPattern.h
/// @brief  Stylus pattern recognition: turn a stroke into a meaning.
///
/// The stylus can be read anywhere in the engine, but reading it only gives
/// coordinates. This turns a drawn shape into a code: a gesture, a digit, a
/// letter. It is the facility retail DS games used for handwriting entry and
/// for gesture commands, rebuilt for NEA.
///
/// Patterns live in a `.neaptn` bank authored with `tools/pattern_editor`,
/// or trained on the DS itself and saved to a filesystem.

#include <nds.h>

#include "NEAFAT.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @defgroup pattern_system Pattern recognition system
///
/// Recognises a stylus gesture against a bank of prototypes.
///
/// There are three objects. A **strokes** buffer collects what the stylus
/// draws. A **bank** holds the prototypes to match against, loaded from a
/// `.neaptn` file or built at run time. A **recognizer** holds the settings
/// and the scratch memory the matching needs. Only the recognizer costs any
/// appreciable RAM, and only when it is set to the Fine algorithm.
///
/// ```c
/// NEA_PatternSystemReset(0);
///
/// NEA_PatternBank *bank = NEA_PatternBankLoad(patterns_bin);
/// NEA_PatternStrokes *ink = NEA_PatternStrokesCreate(512);
/// NEA_PatternRecognizer *rec = NEA_PatternRecognizerCreate(40, 8);
///
/// while (1)
/// {
///     NEA_WaitForVBL(0);
///     scanKeys();
///
///     // Collects points while the stylus is down and closes the stroke when
///     // it lifts. Returns 1 on the frame the stylus came up.
///     NEA_PatternStrokesFeedTouch(ink);
///
///     if (keysDown() & KEY_A)
///     {
///         NEA_PatternResult results[5];
///         int n = NEA_PatternRecognize(rec, bank, ink, NEA_PATTERN_KIND_ALL,
///                                      results, 5);
///         if (n > 0)
///             printf("%s (%d)\n",
///                    NEA_PatternBankGetCodeName(bank, results[0].code),
///                    results[0].score);
///         NEA_PatternStrokesClear(ink);
///     }
/// }
/// ```
///
/// A gesture matches at any size and anywhere on the screen: the whole shape
/// is fitted to the bank's coordinate space before anything is compared. What
/// is *not* forgiven is the number of strokes, which must match exactly, or
/// the direction each one is drawn in.
///
/// @{

/// Default maximum number of banks, used by NEA_PatternSystemReset().
#define NEA_DEFAULT_PATTERN_BANKS 4

/// Default maximum number of recognizers, used by NEA_PatternSystemReset().
#define NEA_DEFAULT_PATTERN_RECOGNIZERS 4

/// Match every entry regardless of its kind bitmask.
#define NEA_PATTERN_KIND_ALL 0xFFFFFFFF

/// A perfect score. Scores are f32, so this is 1.0.
#define NEA_PATTERN_SCORE_MAX 4096

/// The x coordinate of a pen up marker. No real point may use it.
#define NEA_PATTERN_PEN_UP_X (-1)

/// The y coordinate of a pen up marker.
#define NEA_PATTERN_PEN_UP_Y (-1)

/// One sampled point. Screen space, with y running down.
typedef struct {
    int16_t x; ///< X coordinate, or NEA_PATTERN_PEN_UP_X for a marker.
    int16_t y; ///< Y coordinate, or NEA_PATTERN_PEN_UP_Y for a marker.
} NEA_PatternPoint;

/// Matching algorithm, from cheapest to most tolerant.
typedef enum {
    /// Compares stroke *direction* only. It needs no scratch memory and is
    /// the fastest, but it knows nothing about where the strokes sit relative
    /// to one another, so it cannot tell a `T` from a `+`. Good for single
    /// stroke gestures such as flicks, arcs and loops.
    NEA_PATTERN_LIGHT = 0,

    /// Weights direction agreement by how close the corresponding points are,
    /// so relative stroke position counts. Still needs no scratch memory.
    /// This is the default and the right choice for most uses.
    NEA_PATTERN_STANDARD = 1,

    /// Elastic matching: pairs the points with dynamic programming instead of
    /// assuming both were drawn at proportional speeds. Tolerates a hand that
    /// lingers or rushes part way through, at the cost of scratch memory that
    /// grows with the square of the point count and noticeably more time.
    NEA_PATTERN_FINE = 2
} NEA_PatternAlgorithm;

/// How a gesture is reduced to the handful of points that get compared.
typedef enum {
    /// Keep every point, removing only exact repeats. Slowest to match and
    /// the most sensitive to a shaky hand.
    NEA_PATTERN_RESAMPLE_NONE = 0,

    /// Keep a point every `threshold` units of travel. Cheapest, but a slow
    /// hand oversamples and a fast one cuts corners.
    NEA_PATTERN_RESAMPLE_DISTANCE = 1,

    /// Keep a point wherever the direction turns by `threshold` 16 bit angle
    /// units or more, where 65536 is a full turn.
    NEA_PATTERN_RESAMPLE_ANGLE = 2,

    /// Keep the points that carry the shape, by recursively splitting at
    /// whichever point lies furthest off the chord until none is further than
    /// `threshold`. The default, and the best of the four.
    NEA_PATTERN_RESAMPLE_RECURSIVE = 3
} NEA_PatternResampleMethod;

/// One ranked match.
typedef struct {
    int entry;      ///< Index of the matching entry in the bank.
    int code;       ///< Code of that entry, its meaning.
    int32_t score;  ///< f32 in [0, NEA_PATTERN_SCORE_MAX]; higher is better.
} NEA_PatternResult;

/// A buffer of stylus points, opaque. Create it with
/// NEA_PatternStrokesCreate().
typedef struct NEA_PatternStrokes NEA_PatternStrokes;

/// A bank of prototypes, opaque. Create it with NEA_PatternBankLoad(),
/// NEA_PatternBankLoadFAT() or NEA_PatternBankCreate().
typedef struct NEA_PatternBank NEA_PatternBank;

/// Matching settings and scratch, opaque. Create it with
/// NEA_PatternRecognizerCreate().
typedef struct NEA_PatternRecognizer NEA_PatternRecognizer;

// Strokes
// =======

/// Creates a buffer that can hold up to max_points sampled points.
///
/// The pen up markers that separate strokes are counted against the budget,
/// so allow one per stroke you expect. 512 is generous for a handwritten
/// character sampled at 60 Hz.
///
/// @param max_points Capacity in points. Must be at least 2.
/// @return Pointer to the new buffer, or NULL on error.
NEA_PatternStrokes *NEA_PatternStrokesCreate(int max_points);

/// Frees a strokes buffer.
///
/// @param strokes Buffer to free. NULL is ignored.
void NEA_PatternStrokesDelete(NEA_PatternStrokes *strokes);

/// Discards everything in the buffer and lifts the pen.
///
/// @param strokes Buffer to clear.
void NEA_PatternStrokesClear(NEA_PatternStrokes *strokes);

/// Samples the stylus and appends what it finds.
///
/// Call this once per frame, after scanKeys(). It appends a point while the
/// stylus is down and closes the stroke with a pen up marker on the frame it
/// lifts, so a whole multi-stroke gesture accumulates with no bookkeeping in
/// the caller.
///
/// @param strokes Buffer to append to.
/// @return 1 on the frame the stylus lifted, 0 otherwise, negative on error.
int NEA_PatternStrokesFeedTouch(NEA_PatternStrokes *strokes);

/// Appends one point.
///
/// Use this instead of NEA_PatternStrokesFeedTouch() when the points come
/// from somewhere other than the stylus, or when the stylus is sampled more
/// than once per frame.
///
/// @param strokes Buffer to append to.
/// @param x X coordinate. Must not be NEA_PATTERN_PEN_UP_X.
/// @param y Y coordinate.
/// @return 0 on success, negative if the buffer is full or the point is the
///         pen up marker.
int NEA_PatternStrokesAppendPoint(NEA_PatternStrokes *strokes, int x, int y);

/// Ends the current stroke.
///
/// Does nothing if the current stroke is empty, so it is safe to call more
/// than once.
///
/// @param strokes Buffer to append to.
/// @return 0 on success, negative if the buffer is full.
int NEA_PatternStrokesAppendPenUp(NEA_PatternStrokes *strokes);

/// Checks whether anything has been drawn.
///
/// @param strokes Buffer to check.
/// @return True if the buffer holds no points.
bool NEA_PatternStrokesIsEmpty(const NEA_PatternStrokes *strokes);

/// Checks whether the buffer has run out of room.
///
/// @param strokes Buffer to check.
/// @return True if no further point fits.
bool NEA_PatternStrokesIsFull(const NEA_PatternStrokes *strokes);

/// Counts the strokes drawn so far, the one in progress included.
///
/// @param strokes Buffer to check.
/// @return Number of strokes, or negative on error.
int NEA_PatternStrokesGetCount(const NEA_PatternStrokes *strokes);

/// Gets the raw points, for drawing the ink back to the user.
///
/// The array is marker terminated: a point whose x is NEA_PATTERN_PEN_UP_X
/// ends a stroke and must not be drawn. The pointer stays valid until the
/// buffer is modified or freed.
///
/// @param strokes Buffer to read.
/// @param points Set to the point array. May be NULL.
/// @return Number of points, or negative on error.
int NEA_PatternStrokesGetPoints(const NEA_PatternStrokes *strokes,
                                const NEA_PatternPoint **points);

// Banks
// =====

/// Loads a bank from a `.neaptn` in memory.
///
/// The data is copied, so the pointer does not need to stay valid.
///
/// @param pointer Pointer to the file contents.
/// @return Pointer to the new bank, or NULL on error.
NEA_PatternBank *NEA_PatternBankLoad(const void *pointer);

/// Loads a bank from a filesystem.
///
/// Any path the C library can open works, so a bank can come from NitroFS,
/// from FAT, or from a mounted NPAC archive.
///
/// @param path Path to the `.neaptn` file.
/// @return Pointer to the new bank, or NULL on error.
NEA_PatternBank *NEA_PatternBankLoadFAT(const char *path);

/// Creates an empty bank to be filled in at run time.
///
/// This is how an application lets the player define their own gestures. Add
/// to it with NEA_PatternBankAdd() and keep it with NEA_PatternBankSaveFAT().
///
/// @param max_entries Most prototypes the bank will hold.
/// @param max_points Most points, pen up markers included, across all of them.
/// @param normalize_size Coordinate space every prototype is fitted to. 64 is
///                       what the tools default to; larger costs more time in
///                       matching and buys little.
/// @return Pointer to the new bank, or NULL on error.
NEA_PatternBank *NEA_PatternBankCreate(int max_entries, int max_points,
                                       int normalize_size);

/// Frees a bank.
///
/// @param bank Bank to free. NULL is ignored.
void NEA_PatternBankDelete(NEA_PatternBank *bank);

/// Adds a gesture to a bank as a new prototype.
///
/// The strokes are fitted to the bank's coordinate space first, so they can
/// be handed straight over from NEA_PatternStrokesFeedTouch(). Only a bank
/// from NEA_PatternBankCreate() can be added to.
///
/// @param bank Bank to add to.
/// @param strokes Gesture to store.
/// @param code Meaning to report when this prototype matches.
/// @param kind Bitmask this prototype belongs to. Use 1 if unsure.
/// @return Index of the new entry, or negative on error.
int NEA_PatternBankAdd(NEA_PatternBank *bank, const NEA_PatternStrokes *strokes,
                       int code, uint32_t kind);

/// Removes an entry, renumbering everything after it.
///
/// @param bank Bank to modify.
/// @param entry Index of the entry to remove.
/// @return 0 on success, negative on error.
int NEA_PatternBankRemove(NEA_PatternBank *bank, int entry);

/// Includes or excludes an entry from matching.
///
/// Cheaper than removing it, and reversible. Use this to narrow recognition
/// to what the current screen expects.
///
/// @param bank Bank to modify.
/// @param entry Index of the entry.
/// @param enabled True to include it.
/// @return 0 on success, negative on error.
int NEA_PatternBankSetEnabled(NEA_PatternBank *bank, int entry, bool enabled);

/// Writes a bank out as a `.neaptn`.
///
/// @param bank Bank to save.
/// @param path Path to write to. It must be writable, so NitroFS will not do.
/// @return 0 on success, negative on error.
int NEA_PatternBankSaveFAT(const NEA_PatternBank *bank, const char *path);

/// Counts the entries in a bank.
///
/// @param bank Bank to query.
/// @return Number of entries, or negative on error.
int NEA_PatternBankGetEntryCount(const NEA_PatternBank *bank);

/// Gets the code an entry reports when it matches.
///
/// @param bank Bank to query.
/// @param entry Index of the entry.
/// @return The code, or negative on error.
int NEA_PatternBankGetEntryCode(const NEA_PatternBank *bank, int entry);

/// Gets the name a code was authored under.
///
/// Names are optional, so this returns NULL for a bank that carries none.
///
/// @param bank Bank to query.
/// @param code Code to look up.
/// @return The name, or NULL if there is none.
const char *NEA_PatternBankGetCodeName(const NEA_PatternBank *bank, int code);

/// Gets an entry's stored points, for drawing the prototype to the user.
///
/// The array is marker terminated and lives inside the bank, so it stays
/// valid until the bank is modified or freed. Coordinates are in the bank's
/// own space, which NEA_PatternBankGetNormalizeSize() reports.
///
/// @param bank Bank to query.
/// @param entry Index of the entry.
/// @param points Set to the point array. May be NULL.
/// @return Number of points, or negative on error.
int NEA_PatternBankGetEntryPoints(const NEA_PatternBank *bank, int entry,
                                  const NEA_PatternPoint **points);

/// Gets the coordinate space a bank's prototypes are stored in.
///
/// @param bank Bank to query.
/// @return The size, or negative on error.
int NEA_PatternBankGetNormalizeSize(const NEA_PatternBank *bank);

// Recognition
// ===========

/// Creates a recognizer.
///
/// The caps are what a gesture is reduced to before matching, not what the
/// stylus may draw. 40 points and 8 strokes comfortably hold a handwritten
/// character; a gesture that overruns them is truncated rather than refused.
///
/// @param max_points Most points a gesture is reduced to.
/// @param max_strokes Most strokes a gesture may have.
/// @return Pointer to the new recognizer, or NULL on error.
NEA_PatternRecognizer *NEA_PatternRecognizerCreate(int max_points,
                                                   int max_strokes);

/// Frees a recognizer.
///
/// @param rec Recognizer to free. NULL is ignored.
void NEA_PatternRecognizerDelete(NEA_PatternRecognizer *rec);

/// Selects the matching algorithm. The default is NEA_PATTERN_STANDARD.
///
/// Selecting NEA_PATTERN_FINE is what allocates its scratch memory, so a
/// program that never asks for it never pays for it. That allocation can
/// fail, which is why this reports a result.
///
/// @param rec Recognizer to configure.
/// @param algo Algorithm to use.
/// @return 0 on success, negative on error.
int NEA_PatternRecognizerSetAlgorithm(NEA_PatternRecognizer *rec,
                                      NEA_PatternAlgorithm algo);

/// Selects how a gesture is reduced before matching.
///
/// The default is NEA_PATTERN_RESAMPLE_RECURSIVE with a threshold of 2. The
/// threshold is in the bank's coordinate space, so 2 in a 64 unit space means
/// "keep any detail further than 2/64 of the gesture off the chord". Raising
/// it is the first thing to try when recognition is slow; lowering it is the
/// first thing to try when two similar shapes are being confused.
///
/// @param rec Recognizer to configure.
/// @param method Method to use.
/// @param threshold Threshold, in coordinate units for DISTANCE and
///                  RECURSIVE and in 16 bit angle units for ANGLE.
/// @return 0 on success, negative on error.
int NEA_PatternRecognizerSetResample(NEA_PatternRecognizer *rec,
                                     NEA_PatternResampleMethod method,
                                     int threshold);

/// Tunes the length filter NEA_PATTERN_FINE uses to skip hopeless pairs.
///
/// A stroke pair is rejected without running the expensive matching when both
/// strokes are longer than the threshold and one is more than ratio times the
/// other. Has no effect on the other algorithms.
///
/// @param rec Recognizer to configure.
/// @param threshold Length below which the filter never rejects, as f32.
///                  Pass 0 for the default, the bank's normalize size.
/// @param ratio Length ratio at which a pair is rejected. Pass 0 for 3.
/// @return 0 on success, negative on error.
int NEA_PatternRecognizerSetLengthFilter(NEA_PatternRecognizer *rec,
                                         int32_t threshold, int ratio);

/// Recognises a gesture.
///
/// Results come back best first. An entry only competes if its kind is in
/// kind_mask and it has the same number of strokes as the gesture.
///
/// @param rec Recognizer to use.
/// @param bank Bank to match against.
/// @param strokes Gesture to recognise.
/// @param kind_mask Kinds to consider. NEA_PATTERN_KIND_ALL for every one.
/// @param results Array to fill, best first.
/// @param max_results Capacity of that array.
/// @return Number of results written. 0 means nothing matched, which includes
///         a gesture too degenerate to score. Negative on error.
int NEA_PatternRecognize(NEA_PatternRecognizer *rec, NEA_PatternBank *bank,
                         const NEA_PatternStrokes *strokes, uint32_t kind_mask,
                         NEA_PatternResult *results, int max_results);

/// Gets the points the last recognition actually compared.
///
/// This is the gesture after it has been fitted to the bank's space and
/// reduced, which is what the matching saw rather than what the stylus drew.
/// Drawing it over the raw ink is the quickest way to see that a resample
/// threshold is throwing away the shape.
///
/// The array is marker terminated and lives inside the recognizer.
///
/// @param rec Recognizer to query.
/// @param points Set to the point array. May be NULL.
/// @return Number of points, or negative on error.
int NEA_PatternRecognizerGetInputPoints(const NEA_PatternRecognizer *rec,
                                        const NEA_PatternPoint **points);

// System
// ======

/// Resets the pattern system and sets how many objects it may hold.
///
/// Call this before anything else in this module. Unlike the core systems it
/// is not reset by NEA_Init3D(), so a program that never recognises a gesture
/// never links any of this in.
///
/// @param max_banks Number of banks. Below 1 gives NEA_DEFAULT_PATTERN_BANKS.
/// @return 0 on success, negative on error.
int NEA_PatternSystemReset(int max_banks);

/// Frees every bank and recognizer, and the system itself.
void NEA_PatternSystemEnd(void);

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_PATTERN_H__
