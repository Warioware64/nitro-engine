// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_CELL_H__
#define NEA_CELL_H__

/// @file   NEACell.h
/// @brief  Cell animation: one authored format, three renderers.
///
/// A cell is a list of parts, each a rectangle of an atlas placed at an
/// offset -- the shape retail DS games used in NCER. A sequence steps through
/// cells with per-frame durations and an optional scale/rotate/translate, the
/// way NANR did. On top of that this adds per-part keyframe tracks, a parent
/// hierarchy, and per-part colour and alpha.
///
/// The same `.neacell` drives three backends: hardware OBJ sprites, textured
/// quads in screen space, and camera-facing billboards in a 3D scene. They
/// share one evaluator, so what you author is what all three play.

#include <nds.h>

#include "NEA2D.h"
#include "NEACamera.h"
#include "NEAHw2D.h"
#include "NEAPalette.h"
#include "NEATexture.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @defgroup cell_system Cell animation system
///
/// Keyframe-driven 2D cell animation, loaded from compact binary `.neacell`
/// files authored with `tools/cell_editor`. Animation data is shared; each
/// playing instance is a small struct with its own clock.
///
/// @{

// =========================================================================
// Constants
// =========================================================================

#define NEA_DEFAULT_CELL_ANIMS 16 ///< Default max cell animation instances.

/// Binary file magic: "NCEL" in little-endian.
#define NEA_CELL_MAGIC 0x4C45434E

/// Version written by the current tools.
#define NEA_CELL_VERSION 1

/// Number of hardware OBJ size classes (see NEA_OBJSize in NEAHw2D.h).
#define NEA_CELL_OBJ_SIZES 12

/// A part that no hardware OBJ size class can hold.
#define NEA_CELL_OBJ_SIZE_NONE 0xFF

/// A part with no graphics in the companion `.ncgfx` blob.
#define NEA_CELL_NO_GFX 0xFFFFFFFF

/// One full turn in the format's angle unit. Retail NANR's unit, kept so an
/// imported animation lands exactly where the original did.
#define NEA_CELL_ROT_FULL 65536

// =========================================================================
// Enums
// =========================================================================

/// What a sequence animates.
typedef enum {
    NEA_CELL_KIND_CELL  = 0, ///< Steps through cells. The retail shape.
    NEA_CELL_KIND_RIG   = 1, ///< One cell, moved by per-part keyframe tracks.
    NEA_CELL_KIND_MULTI = 2  ///< Steps through multi-cells.
} NEA_CellKind;

/// Playback mode. The values and the behaviour are retail NANR's, including
/// the fact that the two "backward" modes are really ping-pong: they play
/// forward first, then reverse.
typedef enum {
    NEA_CELL_MODE_FORWARD       = 1, ///< Play once, hold the last frame.
    NEA_CELL_MODE_FORWARD_LOOP  = 2, ///< Loop back to the first frame.
    NEA_CELL_MODE_PINGPONG      = 3, ///< Forward, reverse, stop at frame 0.
    NEA_CELL_MODE_PINGPONG_LOOP = 4  ///< Forward and reverse, forever.
} NEA_CellMode;

/// How values between two keys are resolved.
typedef enum {
    NEA_CELL_INTERP_STEP   = 0, ///< Hold. What every imported NANR uses.
    NEA_CELL_INTERP_LINEAR = 1  ///< Interpolate.
} NEA_CellInterp;

/// How a track stores its values. The same three modes as NEAAnimMat, chosen
/// for the same reason: a track that never changes should not cost an array,
/// and one that changes every tick should not cost a binary search.
typedef enum {
    NEA_CELL_STORE_KEYS  = 0, ///< Keyframes, binary searched.
    NEA_CELL_STORE_CONST = 1, ///< One value, held in the track header.
    NEA_CELL_STORE_BAKED = 2  ///< One int16_t per tick, indexed directly.
} NEA_CellStorage;

/// What a track drives on its part.
typedef enum {
    NEA_CELL_CH_TX       = 0, ///< Translate X, f32 pixels.
    NEA_CELL_CH_TY       = 1, ///< Translate Y, f32 pixels.
    NEA_CELL_CH_ROT      = 2, ///< Rotation, 65536 = 360 degrees.
    NEA_CELL_CH_SX       = 3, ///< Scale X, f32.
    NEA_CELL_CH_SY       = 4, ///< Scale Y, f32.
    NEA_CELL_CH_ALPHA    = 5, ///< Alpha 0-31. No hardware OBJ equivalent.
    NEA_CELL_CH_COLOR    = 6, ///< RGB15 tint. No hardware OBJ equivalent.
    NEA_CELL_CH_VISIBLE  = 7, ///< 0 or 1, always stepped.
    NEA_CELL_CH_PARTSWAP = 8, ///< Draw another part's pixels, always stepped.
    NEA_CELL_CH_COUNT    = 9
} NEA_CellChannel;

/// Where a part cannot be drawn.
typedef enum {
    NEA_CELL_PART_HFLIP       = 1 << 0, ///< Mirror horizontally.
    NEA_CELL_PART_VFLIP       = 1 << 1, ///< Mirror vertically.
    NEA_CELL_PART_DOUBLE_SIZE = 1 << 2, ///< Double the OBJ area when affine.
    NEA_CELL_PART_HIDDEN      = 1 << 3, ///< Never drawn.
    NEA_CELL_PART_NO_OAM      = 1 << 4, ///< Hardware OBJ cannot hold it.
    NEA_CELL_PART_NO_3D       = 1 << 5  ///< Not drawn by the 3D backends.
} NEA_CellPartFlags;

/// What to do about parts the hardware OBJ backend cannot draw.
typedef enum {
    /// Skip them and draw the rest. A shoulder pad vanishes; the character
    /// does not. This is the default because a partly-drawn cell is almost
    /// always more useful than nothing.
    NEA_CELL_OAM_SKIP = 0,
    /// Refuse to bind at all, so the mistake is loud.
    NEA_CELL_OAM_FAIL = 1
} NEA_CellOAMPolicy;

/// The point of a cell that its position refers to, and that
/// NEA_CellAnimSetTransformI() rotates and scales about.
typedef enum {
    NEA_CELL_ANCHOR_CENTER  = 0, ///< Middle of the cell's bounding box.
    NEA_CELL_ANCHOR_BOTTOM  = 1, ///< Bottom centre, so feet meet the ground.
    NEA_CELL_ANCHOR_TOPLEFT = 2  ///< The cell origin itself.
} NEA_CellAnchor;

// =========================================================================
// On-disk records
//
// These map the file byte for byte. tools/cell_editor/cell_format.py writes
// exactly this layout and documents every field -- keep the two in lockstep.
// =========================================================================

/// One source image, resolved to a material by name.
typedef struct {
    char name[32];       ///< NEA_MaterialFindByName() key.
    uint16_t width;      ///< Atlas width in pixels.
    uint16_t height;     ///< Atlas height in pixels.
    uint8_t tex_format;  ///< NEA_TextureFormat.
    uint8_t obj_color;   ///< NEA_OBJColorMode. Hardware OBJ backend only.
    uint8_t flags;       ///< bit0: palette index 0 is transparent.
    uint8_t pad[9];
} NEA_CellAtlas;

/// One cell: a run of parts, plus the bounds the backends cull against.
typedef struct {
    uint16_t first_part; ///< Index into the part pool.
    uint16_t num_parts;
    int16_t min_x;       ///< Bounding box in cell space, Y down.
    int16_t min_y;
    int16_t max_x;
    int16_t max_y;
    uint16_t radius;     ///< Bounding circle, for cheap culling.
    uint16_t flags;
    uint32_t reserved;
} NEA_CellCell;

/// One part: a rectangle of an atlas, placed and optionally parented.
///
/// Parts are drawn in array order, so the last part of a cell is on top. Note
/// this is the reverse of retail NCER, where OBJ 0 is topmost; the importer
/// flips the order so that "later is nearer" holds for all three backends.
typedef struct {
    uint16_t src_x;      ///< Texel origin in the atlas.
    uint16_t src_y;
    uint16_t src_w;      ///< Texel size.
    uint16_t src_h;
    int16_t off_x;       ///< Top-left in cell space.
    int16_t off_y;
    int16_t pivot_x;     ///< Rotation and scale centre, cell space.
    int16_t pivot_y;
    int16_t parent;      ///< Parent part index, or -1 for a root.
    uint16_t color;      ///< RGB15 tint, 0x7FFF = untinted. 3D backends only.
    uint8_t atlas;       ///< Index into the atlas table.
    uint8_t pal_slot;    ///< 16-colour OBJ palette slot. Hardware only.
    uint8_t obj_size;    ///< NEA_OBJSize, or NEA_CELL_OBJ_SIZE_NONE.
    uint8_t obj_color;   ///< NEA_OBJColorMode. Hardware only.
    uint8_t priority;    ///< 0-3. OBJ priority, or a Z bias in 3D.
    uint8_t alpha;       ///< 0-31. 3D backends only.
    uint8_t poly_id_off; ///< Added to the instance polygon ID. 3D only.
    uint8_t flags;       ///< NEA_CellPartFlags.
    uint32_t gfx_offset; ///< Byte offset into the `.ncgfx` blob, or
                         ///< NEA_CELL_NO_GFX. Hardware only.
    uint32_t gfx_size;   ///< Bytes to transfer. Hardware only.
    uint32_t reserved;
} NEA_CellPart;

/// One sequence.
typedef struct {
    char name[16];        ///< Looked up by NEA_CellFindSequence().
    uint8_t kind;         ///< NEA_CellKind.
    uint8_t mode;         ///< NEA_CellMode.
    uint8_t interp;       ///< NEA_CellInterp, for the per-frame SRT.
    uint8_t flags;
    uint16_t first_frame; ///< Index into the frame pool (CELL and MULTI).
    uint16_t num_frames;
    uint16_t first_track; ///< Index into the track pool (RIG).
    uint16_t num_tracks;
    uint16_t total_ticks; ///< Cached length, in 1/60 s ticks.
    uint16_t cell;        ///< The rig cell (RIG).
} NEA_CellSequence;

/// One frame of a CELL or MULTI sequence.
typedef struct {
    uint16_t target;   ///< Cell index, or multi-cell index for MULTI.
    uint16_t duration; ///< 1/60 s ticks.
    int32_t sx;        ///< f32 scale.
    int32_t sy;
    uint16_t rot;      ///< 65536 = 360 degrees.
    uint16_t pad;
    int16_t px;        ///< Pixel translation.
    int16_t py;
} NEA_CellFrame;

/// One keyframe track on one part.
typedef struct {
    uint8_t channel;  ///< NEA_CellChannel.
    uint8_t interp;   ///< NEA_CellInterp.
    uint8_t storage;  ///< NEA_CellStorage.
    uint8_t flags;
    uint16_t part;    ///< Part index within the rig cell.
    uint16_t count;   ///< KEYS: keyframes. BAKED: ticks. CONST: 0.
    uint32_t data;    ///< KEYS/BAKED: byte offset into the key pool.
                      ///< CONST: the value itself.
} NEA_CellTrack;

/// One keyframe.
typedef struct {
    uint16_t tick;
    uint16_t pad;
    int32_t value;
} NEA_CellKeyframe;

/// A set of nodes that play together.
typedef struct {
    char name[16];
    uint16_t first_node; ///< Index into the node pool.
    uint16_t num_nodes;
} NEA_CellMultiCell;

/// One node of a multi-cell: a sequence with its own clock and its own offset.
///
/// Nodes are drawn in array order, so the last node is in front -- the same
/// rule parts follow.
typedef struct {
    uint16_t seq;     ///< The sequence this node plays.
    int16_t x;        ///< Node offset, added to the node's own translation.
    int16_t y;
    uint8_t priority;
    uint8_t flags;    ///< bit0: start paused.
} NEA_CellNode;

/// What the hardware OBJ backend must allocate up front.
///
/// Computed by the exporter, because the runtime binds its OBJ pool once and
/// then never allocates again -- discovering the worst case at load time
/// would mean walking every cell of every sequence on the ARM9.
typedef struct {
    uint16_t max_objs[NEA_CELL_OBJ_SIZES]; ///< Peak OBJs per size class.
    uint32_t max_transfer;                 ///< Largest single tile copy.
    uint16_t max_affine;                   ///< Peak transformed parts.
    uint16_t pad;
} NEA_CellBudget;

// =========================================================================
// Runtime types
// =========================================================================

/// A loaded `.neacell`. Shared: any number of instances can play it at once.
typedef struct {
    uint16_t flags;         ///< Header flags (see the format docs).
    uint16_t num_atlases;
    uint16_t num_cells;
    uint16_t num_parts;
    uint16_t num_sequences;
    uint16_t num_frames;
    uint16_t num_tracks;
    uint16_t num_multicells;
    uint16_t num_nodes;
    uint16_t max_parts_any_cell; ///< How big a pose buffer has to be.

    const NEA_CellAtlas *atlases;   ///< All of these point into the loaded
    const NEA_CellCell *cells;      ///< blob; nothing here is copied.
    const NEA_CellPart *parts;
    const NEA_CellSequence *sequences;
    const NEA_CellFrame *frames;
    const NEA_CellTrack *tracks;
    const uint8_t *keys;            ///< The pool tracks index into.
    const NEA_CellMultiCell *multicells;
    const NEA_CellNode *nodes;
    const NEA_CellBudget *budget;   ///< NULL if the file carries no BUDG.

    NEA_Material **atlas_mat;       ///< Resolved at bind, one per atlas.
    uint16_t *seq_prefix;           ///< Frame start ticks, one run per
                                    ///< sequence, so the frame lookup is a
                                    ///< binary search and not a walk.
    uint16_t *seq_prefix_base;      ///< Where each sequence's run starts.

    void *_base_data;
    bool _loaded_from_fat;
} NEA_CellData;

/// One part, resolved for this tick.
///
/// `m` and `tx`/`ty` map part-local pixels to cell space: a corner at (u, v)
/// inside the part lands at m * (u, v) + (tx, ty). Every backend consumes
/// this and nothing else, which is what keeps the three of them agreeing.
typedef struct {
    const NEA_CellPart *part; ///< Where the transform came from.
    const NEA_CellPart *src;  ///< Whose pixels to draw. Differs from `part`
                              ///< only when a PARTSWAP track is in play.
    int32_t m[4];             ///< 2x2 matrix, f32, row-major.
    int32_t tx;               ///< Translation, f32 pixels.
    int32_t ty;
    uint16_t color;           ///< RGB15.
    uint8_t alpha;            ///< 0-31.
    uint8_t priority;         ///< 0-3.
    uint8_t visible;
    uint8_t pad[3];
} NEA_CellPartXform;

/// One playing animation.
typedef struct NEA_CellAnim_t NEA_CellAnim;

struct NEA_CellAnim_t {
    const NEA_CellData *data;
    uint16_t sequence;
    int32_t currtick;   ///< f32 ticks. Frames have durations, so this is not
                        ///< a frame counter.
    int32_t speed;      ///< f32; 1 << 12 advances one tick per call.
    bool playing;
    bool finished;
    bool active;
    bool is_child;      ///< Seated as a multi-cell node. Its parent drives its
                        ///< clock, so NEA_CellAnimUpdateAll() steps over it.

    // Instance transform, applied on top of whatever the sequence resolves to.
    int32_t rot;        ///< 0-511, matching NEA_SpriteSetRot().
    int32_t scale_x;    ///< f32.
    int32_t scale_y;
    uint8_t base_alpha;
    uint8_t base_poly_id;
    uint16_t base_color;

    // Billboard placement.
    int32_t units_per_pixel; ///< f32 world units per source pixel.
    uint8_t anchor;          ///< NEA_CellAnchor.

    // Multi-cell: one child per node, each with its own clock. Children own
    // their own pose buffers, so nothing is copied up into the parent.
    NEA_CellAnim **children;
    uint16_t num_children;
    uint16_t multicell;      ///< Which multi-cell the children were seated
                             ///< from, so a MULTI sequence only re-seats them
                             ///< when it actually changes.
    int16_t node_x;          ///< This instance's offset as a multi-cell node.
    int16_t node_y;
    uint8_t node_priority;
    int16_t multi_px;        ///< The MULTI frame's own translation, applied to
    int16_t multi_py;        ///< every child at draw time.

    // Hardware OBJ binding. Opaque: NEACellOAM.c owns it, and keeping it a
    // void * is what stops a ROM that never calls NEA_CellBindOAM() from
    // linking NEAHw2D at all.
    void *oam;

    NEA_CellPartXform *pose;
    uint16_t pose_count;
    uint16_t pose_capacity;
    uint16_t pose_cell;      ///< Which cell the current pose came from, so a
                             ///< backend can find its anchor without
                             ///< searching the bank for it.
    void *xform_scratch;     ///< One accumulated transform per part, so a
                             ///< child can read its parent's without a
                             ///< second pass. Owned; sized with the pose.
};

// =========================================================================
// System
// =========================================================================

/// Reset the cell system and set how many instances can exist.
///
/// Call once before creating any animation. Unlike the core systems this is
/// opt-in: NEA_Init3D() does not call it, so a ROM that never animates a cell
/// pays nothing.
///
/// @param max_anims Number of instances. Below 1 means NEA_DEFAULT_CELL_ANIMS.
/// @return 0 on success.
int NEA_CellSystemReset(int max_anims);

/// Shut the cell system down and free everything it owns.
void NEA_CellSystemEnd(void);

// =========================================================================
// Loading
// =========================================================================

/// Parse a `.neacell` already in RAM.
///
/// The returned data points into @p pointer, which must stay alive and must
/// not move. Nothing is copied.
///
/// @param pointer The file contents.
/// @return The loaded bank, or NULL if it is not a valid `.neacell`.
NEA_CellData *NEA_CellDataLoad(const void *pointer);

/// Load a `.neacell` from the filesystem.
NEA_CellData *NEA_CellDataLoadFAT(const char *path);

/// Free a bank. Any instance still pointing at it is stopped first.
void NEA_CellDataFree(NEA_CellData *data);

/// Resolve every atlas to a material by name, with NEA_MaterialFindByName().
///
/// @return 0 if every atlas found its material, otherwise the number missing.
int NEA_CellDataBindMaterials(NEA_CellData *data);

/// Point one atlas at a material directly, for banks whose artwork is not
/// registered under the name the file expects.
int NEA_CellDataSetMaterial(NEA_CellData *data, int atlas, NEA_Material *mat);

// =========================================================================
// Query
// =========================================================================

/// Find a sequence by name. Returns -1 if there is none.
int NEA_CellFindSequence(const NEA_CellData *data, const char *name);

/// Find a multi-cell by name. Returns -1 if there is none.
int NEA_CellFindMultiCell(const NEA_CellData *data, const char *name);

/// Length of a sequence in 1/60 s ticks.
int NEA_CellSequenceTicks(const NEA_CellData *data, int seq);

/// A cell's bounding box, in cell space. Any output pointer may be NULL.
int NEA_CellGetBounds(const NEA_CellData *data, int cell,
                      int *min_x, int *min_y, int *max_x, int *max_y);

// =========================================================================
// Instances
// =========================================================================

/// Create an animation instance.
NEA_CellAnim *NEA_CellAnimCreate(void);

/// Delete an instance and everything it owns, including its OBJ pool.
void NEA_CellAnimDelete(NEA_CellAnim *anim);

/// Delete every instance.
void NEA_CellAnimDeleteAll(void);

/// Point an instance at a bank. Stops whatever it was playing.
int NEA_CellAnimSetData(NEA_CellAnim *anim, const NEA_CellData *data);

/// Start a sequence.
///
/// @param anim  The instance.
/// @param seq   Sequence index.
/// @param speed f32 ticks per update; (1 << 12) is real time. 0 keeps the
///              instance's current speed.
/// @return 0 on success.
int NEA_CellAnimPlay(NEA_CellAnim *anim, int seq, int32_t speed);

/// Start a sequence looked up by name.
int NEA_CellAnimPlayNamed(NEA_CellAnim *anim, const char *name, int32_t speed);

/// Seat one child instance per node of a multi-cell and start them all.
///
/// Each node keeps its own clock, so a looping cape and a one-shot swing can
/// share one entity without being resampled onto a common frame counter.
int NEA_CellAnimPlayMulti(NEA_CellAnim *anim, int multicell);

/// The child instance playing a multi-cell node, or NULL.
NEA_CellAnim *NEA_CellAnimGetNode(NEA_CellAnim *anim, int node);

/// Stop playback and rewind to the start.
void NEA_CellAnimStop(NEA_CellAnim *anim);

/// Freeze or resume without rewinding.
void NEA_CellAnimPause(NEA_CellAnim *anim, bool paused);

/// Set playback speed, f32 ticks per update.
void NEA_CellAnimSetSpeed(NEA_CellAnim *anim, int32_t speed);

/// Jump to a tick, given in f32.
void NEA_CellAnimSetTick(NEA_CellAnim *anim, int32_t tick);

/// The current play head, in f32 ticks.
int32_t NEA_CellAnimGetTick(const NEA_CellAnim *anim);

/// The frame index the play head currently resolves to, or -1.
int NEA_CellAnimGetFrame(const NEA_CellAnim *anim);

/// True once a non-looping sequence has run out.
bool NEA_CellAnimIsFinished(const NEA_CellAnim *anim);

/// Alpha, polygon ID and tint applied on top of every part. 3D backends only.
void NEA_CellAnimSetParams(NEA_CellAnim *anim, uint8_t alpha,
                           uint8_t poly_id, uint16_t color);

/// An extra rotation and scale around the whole cell.
///
/// Both happen about the cell's anchor -- its bounding-box centre by default,
/// so a cell spins in place the way NEA_SpriteSetRot() does. Choose a
/// different pivot with NEA_CellAnimSetAnchor(): BOTTOM turns a character
/// about its feet, and TOPLEFT turns it about the cell origin.
///
/// @param anim The instance.
/// @param rot  Angle, 0-511, the same unit NEA_SpriteSetRot() uses.
/// @param sx   f32 scale.
/// @param sy   f32 scale.
void NEA_CellAnimSetTransformI(NEA_CellAnim *anim, int rot,
                               int32_t sx, int32_t sy);

/// Advance every instance one step and re-evaluate it.
///
/// Called for you by NEA_WaitForVBL(NEA_UPDATE_CELL).
void NEA_CellAnimUpdateAll(void);

/// Advance one instance and re-evaluate it.
void NEA_CellAnimUpdate(NEA_CellAnim *anim);

/// Resolve the current tick into the instance's pose buffer.
///
/// Called by NEA_CellAnimUpdate(). Call it yourself only after changing
/// something mid-frame that the next draw has to see.
///
/// @return Number of posed parts.
int NEA_CellAnimEvaluate(NEA_CellAnim *anim);

/// The resolved pose. Valid until the next update.
const NEA_CellPartXform *NEA_CellAnimGetPose(const NEA_CellAnim *anim,
                                             int *count);

// =========================================================================
// Backend: textured quads in screen space
// =========================================================================

/// Draw an instance as textured quads, in screen pixels.
///
/// Call NEA_2DViewInit() first, exactly as for NEA_SpriteDraw().
///
/// @param anim The instance.
/// @param x    Where the cell origin lands, in screen pixels.
/// @param y    Where the cell origin lands, in screen pixels.
void NEA_CellAnimDraw2D(NEA_CellAnim *anim, int x, int y);

// =========================================================================
// Backend: camera-facing billboards in world space
// =========================================================================

/// Set the camera billboards face. Required before drawing one.
void NEA_CellAnimSetCamera(NEA_Camera *camera);

/// How much world space one source pixel covers.
///
/// The default is f32 1/32, so a 32-pixel-tall character stands one world
/// unit high.
void NEA_CellAnimSetUnitsPerPixelI(NEA_CellAnim *anim, int32_t upp);

/// Where the cell's origin sits relative to the position it is drawn at.
///
/// This is also the pivot NEA_CellAnimSetTransformI() turns about, on every
/// backend -- one point per instance rather than one per purpose.
void NEA_CellAnimSetAnchor(NEA_CellAnim *anim, NEA_CellAnchor anchor);

/// Draw an instance as a camera-facing billboard at a world position.
void NEA_CellAnimDrawBillboardI(NEA_CellAnim *anim, int32_t x, int32_t y,
                                int32_t z);

/// Draw an instance as a camera-facing billboard at a world position.
#define NEA_CellAnimDrawBillboard(anim, x, y, z) \
    NEA_CellAnimDrawBillboardI(anim, floattof32(x), floattof32(y), \
                               floattof32(z))

// =========================================================================
// Backend: hardware OBJ sprites
// =========================================================================

/// Bind an instance to the hardware OBJ engine.
///
/// Allocates the OBJ pool the file's budget calls for, once, and keeps it for
/// the life of the binding. That is not an optimisation: NEAHw2D hands out OAM
/// indices monotonically and never recycles them, so creating and deleting
/// sprites per frame runs the engine out of entries.
///
/// Graphics are streamed. Only the tiles the current frame shows live in OBJ
/// VRAM; @p ncgfx stays in main RAM and the runtime copies each part's tiles
/// in as the cell changes, which is how retail fitted cell banks far larger
/// than the OBJ bank. @p ncgfx must outlive the binding.
///
/// @param anim   The instance.
/// @param engine NEA_ENGINE_MAIN or NEA_ENGINE_SUB.
/// @param ncgfx  The companion `.ncgfx` tile blob, resident in main RAM.
/// @param size   Its size in bytes.
/// A multi-cell's nodes are bound too, in either order: bind first and the
/// nodes are bound as they are seated, seat first and they are bound here.
/// Each node gets a pool sized for the whole composition, because the file's
/// budget is per bank rather than per sequence -- so a four-node character
/// reserves more sprites than it uses. The parent itself reserves none, since
/// its nodes hold the poses.
///
/// @param policy What to do about parts the hardware cannot draw.
/// @return 0 on success, -1 if the pool could not be allocated.
int NEA_CellAnimBindOAM(NEA_CellAnim *anim, NEA_Hw2DEngine engine,
                        const void *ncgfx, size_t size,
                        NEA_CellOAMPolicy policy);

/// Load the companion `.ncpal` into an OBJ palette slot.
int NEA_CellAnimLoadOAMPalette(NEA_CellAnim *anim, const void *ncpal,
                               int num_colors, int slot);

/// Release the OBJ pool and the palette slots the binding took.
void NEA_CellAnimUnbindOAM(NEA_CellAnim *anim);

/// Push the current pose into the bound OBJ sprites.
///
/// The OAM flush itself stays where it was: NEA_WaitForVBL(NEA_UPDATE_HW2D)
/// calls NEA_Hw2DOBJUpdateAll(), and this never writes OAM directly.
///
/// @param anim The instance.
/// @param x    Where the cell origin lands, in screen pixels.
/// @param y    Where the cell origin lands, in screen pixels.
void NEA_CellAnimApplyOAM(NEA_CellAnim *anim, int x, int y);

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_CELL_H__
