// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Warioware64
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_RB_IPC_H__
#define NEA_RB_IPC_H__

/// @file   NEA_RB_IPC.h
/// @brief  Shared ARM7/ARM9 IPC protocol for the rigid body physics engine.
///
/// This header is included by both the ARM7 physics simulation and the ARM9
/// API layer. It defines FIFO command IDs, limits, and encoding conventions.

#include <nds/fifocommon.h>
#include <nds/ndstypes.h>

// =========================================================================
// Limits
// =========================================================================

#define NEA_RB_MAX_BODIES   16   ///< Max simultaneous rigid bodies.
#define NEA_RB_MAX_STATICS  128  ///< Max static AAR colliders.
#define NEA_RB_MAX_CONTACTS 64   ///< Max contact points per simulation step.

// =========================================================================
// FIFO channels
// =========================================================================

#define NEA_RB_FIFO_CMD   FIFO_USER_08  ///< ARM9 -> ARM7 commands.
#define NEA_RB_FIFO_STATE FIFO_USER_07  ///< ARM7 -> ARM9 state updates.

// =========================================================================
// Command encoding
// =========================================================================
//
// Each command is a 32-bit word sent via fifoSendValue32:
//   bits [5:0]  = command ID (NEA_RB_Command)
//   bits [31:6] = parameter (usually body/static ID)
//
// Multi-word commands send the command word first, followed by data words
// on the same FIFO channel.

#define NEA_RB_CMD_BITS  6
#define NEA_RB_CMD_MASK  ((1u << NEA_RB_CMD_BITS) - 1u)

#define NEA_RB_ENCODE_CMD(cmd, id) \
    ((u32)(cmd) | ((u32)(id) << NEA_RB_CMD_BITS))

#define NEA_RB_DECODE_CMD(val)  ((val) & NEA_RB_CMD_MASK)
#define NEA_RB_DECODE_ID(val)   ((val) >> NEA_RB_CMD_BITS)

/// Command IDs for NEA_RB_FIFO_CMD (ARM9 -> ARM7).
typedef enum {
    NEA_RB_CMD_START         = 1,  ///< Enable simulation. No extra words.
    NEA_RB_CMD_PAUSE         = 2,  ///< Pause simulation. No extra words.
    NEA_RB_CMD_RESET         = 3,  ///< Reset all bodies & statics. No extra words.
    NEA_RB_CMD_ADD_BODY      = 4,  ///< Create OBB. +7 words: hx,hy,hz, mass, px,py,pz.
    NEA_RB_CMD_KILL_BODY     = 5,  ///< Destroy body. ID in header. No extra words.
    NEA_RB_CMD_APPLY_FORCE   = 6,  ///< Apply force at point. +6 words: fx,fy,fz, px,py,pz.
    NEA_RB_CMD_APPLY_IMPULSE = 7,  ///< Apply impulse at point. +6 words: jx,jy,jz, px,py,pz.
    NEA_RB_CMD_SET_VELOCITY  = 8,  ///< Set linear velocity. +3 words: vx,vy,vz.
    NEA_RB_CMD_SET_GRAVITY   = 9,  ///< Set global gravity. +1 word: g (f32, Y-axis).
    NEA_RB_CMD_ADD_STATIC    = 10, ///< Add static AAR. +9 words: px,py,pz, sx,sy,sz, nx,ny,nz.
    NEA_RB_CMD_REMOVE_STATIC = 11, ///< Remove static AAR. ID in header. No extra words.
    NEA_RB_CMD_SET_RESTITUTION = 12, ///< Set restitution. +1 word: e (f32).
    NEA_RB_CMD_SET_FRICTION  = 13, ///< Set friction. +1 word: mu (f32).
    NEA_RB_CMD_SET_POSITION  = 14, ///< Teleport body. +3 words: px,py,pz.
    NEA_RB_CMD_INJECT_CONTACT = 15, ///< Inject external contact. +7 words: nx,ny,nz, px,py,pz, depth.
} NEA_RB_Command;

// =========================================================================
// State packet encoding (ARM7 -> ARM9 via NEA_RB_FIFO_STATE)
// =========================================================================
//
// Per active (non-sleeping) body, ARM7 sends 7 words:
//
//   Word 0: body_id (bits 7:0) | flags (bits 15:8)
//           flags bit 0 = sleeping
//   Word 1: position.x (f32)
//   Word 2: position.y (f32)
//   Word 3: position.z (f32)
//   Word 4: transform[0] (f32)  -- rotation matrix row-major
//   Word 5: transform[1] (f32)
//   Word 6: transform[2] (f32)
//   Word 7: transform[3] (f32)
//   Word 8: transform[4] (f32)
//   Word 9: transform[5] (f32)
//   Word 10: transform[6] (f32)
//   Word 11: transform[7] (f32)
//   Word 12: transform[8] (f32)
//
// A sentinel word 0xFFFFFFFF marks end of frame.

#define NEA_RB_STATE_WORDS    13  ///< Words per body state packet.
#define NEA_RB_STATE_END      0xFFFFFFFFu ///< End-of-frame sentinel.

#define NEA_RB_STATE_FLAG_SLEEP  (1u << 8)

#define NEA_RB_STATE_ENCODE_HDR(id, flags) \
    ((u32)(id) | (u32)(flags))

#define NEA_RB_STATE_DECODE_ID(hdr)    ((hdr) & 0xFFu)
#define NEA_RB_STATE_DECODE_FLAGS(hdr) ((hdr) & 0xFF00u)

#endif // NEA_RB_IPC_H__
