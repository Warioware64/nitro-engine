// SPDX-License-Identifier: MIT
//
// Copyright (c) 2026 Erin Catto        (original Box3D)
// Copyright (c) 2026 Warioware64       (Nitro Engine Advanced fixed-point port)
//
// This file is part of Nitro Engine Advanced

#pragma once

/// @file   b3profile.h
/// @brief  Where the step went, in timer ticks.
///
/// @section why Why this exists when physics_world.h says it does not
///
/// physics_world.h's @section absent lists `b3Profile` among the things the
/// port dropped, on the grounds that upstream's is "millisecond timers on a
/// machine without a millisecond clock". That reasoning was right about
/// upstream's profile and wrong about the need. What it ruled out was the
/// *unit*, not the measurement: a 16.6 ms frame quantized to milliseconds has
/// sixteen distinguishable values, so a millisecond profile of a DS step says
/// almost nothing.
///
/// This one counts **timer ticks** instead. Timers 0 and 1 cascade into one
/// 32-bit counter at BUS_CLOCK -- 33.513982 MHz, about 30 ns a tick -- which
/// resolves a 16.6 ms frame into 558,000 steps and does not wrap for 128
/// seconds. That is enough to separate the broad phase from the narrow phase
/// from the solver, which is the question this file exists to answer.
///
/// @section nesting Why every region is a mark, not a bracket
///
/// libnds gives one cascaded pair, so there is exactly one hardware counter
/// and regions **cannot nest**. cpuStartTiming(0)/cpuEndTiming() is a bracket
/// and would be wrong here twice over: it can only measure one region at a
/// time, and re-arming the timer inside a region resets the count the outer
/// region was accumulating.
///
/// So this takes absolute snapshots and subtracts. b3ProfileStart arms the
/// counter once at the top of the step; every b3ProfileMark reads
/// cpuGetTiming(), charges the elapsed ticks to one field, and remembers the
/// reading as the start of the next region. Phases therefore tile the step
/// exactly, and their sum is the total by construction rather than by luck --
/// which is the property that makes a missing probe visible as a gap.
///
/// @section cost What a probe costs
///
/// cpuGetTiming() is two 16-bit I/O reads at 0x04000100; the mark adds a
/// subtract, a store and a register copy. Order tens of cycles, against a DSi
/// frame of roughly 2.2 million. Level 1 places ten of them per step, which is
/// well inside the noise of the thing being measured.
///
/// Level 2 (B3_NEA_PROFILE_SUBSTEP) is a different matter and carries its own
/// warning -- see B3_NEA_PROFILE_SUBSTEP in nea_config.h.
///
/// @section off What it costs switched off
///
/// Nothing. Every macro below expands to `(void)0` when B3_NEA_PROFILE is 0,
/// and b3ProfileTimer becomes an empty-bodied struct the compiler drops.
///
/// The b3Profile *field* on b3World stays either way, and
/// b3World_GetProfile() is always linkable -- it simply reports zeros in a
/// non-profiling build. A hundred-odd bytes on a struct already several
/// kilobytes wide is not worth an #if in every caller, and a game that reads
/// the profile unconditionally should keep compiling when the switch is off.

#include "box3d/base.h"
#include "box3d/id.h"

#include <stdint.h>

/**
 * @defgroup profile Profile
 * Per-step cost attribution. See @ref b3profile.h.
 * @{
 */

/// One step, attributed. Every `*Ticks` field is in BUS_CLOCK ticks
/// (33.513982 MHz); divide by 33.514 for microseconds, and do it *outside* any
/// region being timed, because the divide is a `__aeabi_uidiv` call in Thumb.
///
/// The level-1 fields tile the step: broad phase, narrow phase, solve and
/// sensors sum to `totalTicks` apart from the bookkeeping either side of them,
/// and the solve breakdown sums to `solveTicks`. A discrepancy means a probe
/// is missing, not that time vanished.
typedef struct b3Profile
{
	// -------------------------------------------------------------------
	// Level 1 -- step phases
	// -------------------------------------------------------------------

	/// The whole of b3World_Step, from the first probe to the last.
	uint32_t totalTicks;

	/// b3UpdateBroadPhasePairs: move-buffer replay, tree queries, new pairs.
	uint32_t broadPhaseTicks;

	/// b3Collide: the whole narrow phase, mesh and convex together. The split
	/// between them is level 3 below.
	uint32_t narrowPhaseTicks;

	/// b3Solve, end to end. The eight fields after it break it down.
	uint32_t solveTicks;

	/// b3OverlapSensors: sensor overlap sets rebuilt and transitions published.
	uint32_t sensorTicks;

	// -------------------------------------------------------------------
	// Level 1 -- inside b3Solve
	// -------------------------------------------------------------------

	/// b3PrepareJointsTask + b3PrepareContacts. Once per step, not per
	/// sub-step -- see the comment above the sub-step loop in b3Solve for why
	/// that is load bearing rather than an optimization.
	uint32_t prepareTicks;

	/// The sub-step loop in full, summed over subStepCount iterations. The
	/// stage breakdown within it is level 2.
	uint32_t subStepTicks;

	/// b3ApplyRestitution + b3StoreImpulses.
	uint32_t restitutionTicks;

	/// Draining the hit-event and joint-event bit sets into their arrays.
	uint32_t eventTicks;

	/// b3FinalizeBodiesTask: transforms rebuilt, move events written, sleep
	/// timers advanced.
	uint32_t finalizeTicks;

	/// b3SolveContinuous over the bullets. Pairs with `toiEventCount` and the
	/// three TOI iteration counters below -- a large number here with a zero
	/// event count means the sweeps are running and finding nothing.
	uint32_t continuousTicks;

	/// Applying enlarged AABBs to the broad phase.
	uint32_t enlargeTicks;

	/// b3SplitIsland + the b3TrySleepIsland sweep.
	uint32_t sleepTicks;

	// -------------------------------------------------------------------
	// Level 2 -- sub-step stages (B3_NEA_PROFILE_SUBSTEP)
	// -------------------------------------------------------------------
	//
	// Accumulated across every sub-step of the step, so each is the total
	// spent in that stage per step rather than per sub-step. Zero unless
	// B3_NEA_PROFILE_SUBSTEP is on.
	//
	// Read these as *ratios between stages*. Their absolute values are
	// inflated by the probes themselves and by what the probes do to b3Solve's
	// instruction footprint -- see B3_NEA_PROFILE_SUBSTEP in nea_config.h.

	uint32_t integrateVelocitiesTicks;
	uint32_t warmStartJointsTicks;
	uint32_t warmStartContactsTicks;
	uint32_t solveJointsTicks;
	uint32_t solveContactsTicks;
	uint32_t integratePositionsTicks;
	uint32_t relaxJointsTicks;
	uint32_t relaxContactsTicks;

	// -------------------------------------------------------------------
	// Level 3 -- narrow phase split (B3_NEA_PROFILE_NARROW)
	// -------------------------------------------------------------------

	/// Time inside b3ComputeMeshManifolds, summed over every mesh contact this
	/// step -- the triangle queries, the per-triangle collides and the normal
	/// clustering. It is 8,496 bytes of code called once per mesh contact, so
	/// this is the field that says whether B3_ITCM_MESH would earn its bytes.
	uint32_t meshManifoldTicks;

	/// The rest of the narrow phase: b3Collide's contact gathering, each
	/// contact's prologue in b3UpdateContact, and every convex, capsule and
	/// sphere collide.
	///
	/// Named for what it is rather than "convex", because it is not only
	/// convex work. The probe pair brackets b3ComputeMeshManifolds and nothing
	/// else, so everything outside that one call lands here -- including a mesh
	/// contact's own prologue. Splitting further would need a probe per contact
	/// per branch, which starts to cost a measurable fraction of the cheap
	/// contacts it would be measuring.
	///
	/// The useful property is that the two tile the narrow phase exactly:
	/// meshManifoldTicks + narrowPhaseOtherTicks == narrowPhaseTicks, so the
	/// split is a real partition rather than two samples that might miss
	/// something between them.
	uint32_t narrowPhaseOtherTicks;

	/// Triangles the mesh BVH handed to the narrow phase this step, before the
	/// per-contact B3_NEA_MAX_MESH_CONTACT_TRIANGLES cap. Not a time -- it is
	/// the work unit `meshContactTicks` is spent on, and the two together give
	/// a cost per triangle that is comparable across scenes.
	uint32_t meshTriangleCount;

	// -------------------------------------------------------------------
	// Counters (free -- the world already keeps these)
	// -------------------------------------------------------------------
	//
	// Every field below was already maintained and reset by b3Collide or
	// b3Solve and simply had no way out of the library. They cost nothing to
	// publish and they answer the questions a timer cannot: a phase that is
	// expensive because it is slow looks identical to one that is expensive
	// because it ran a hundred times.

	/// Bodies in the awake solver set. The multiplier on every per-body pass,
	/// and the number that says whether sleeping is doing anything.
	int awakeBodyCount;

	/// Contacts, manifolds and joints the solver actually built constraints
	/// for this step. `manifoldCount` is the one that drives solver cost --
	/// one mesh contact can carry up to B3_NEA_MAX_MESH_MANIFOLDS of them.
	int contactCount;
	int manifoldCount;
	int jointCount;

	/// Contacts answered from the recycling fast path instead of re-running
	/// the narrow phase. High is good; it is narrow-phase work not done.
	int recycledContactCount;

	/// Dynamic bodies parked this step for leaving maximumWorldExtent.
	int parkedBodyCount;

	/// The continuous pass: bodies pulled back to a time of impact, and the
	/// worst iteration counts b3TimeOfImpact reported doing it.
	int toiEventCount;
	int toiDistanceIterations;
	int toiPushBackIterations;
	int toiRootIterations;

	/// Normal clusters dropped for exceeding B3_NEA_MAX_MESH_MANIFOLDS. A
	/// non-zero reading here means contacts are being silently discarded and
	/// no timing number below is describing the scene you think it is.
	int meshManifoldDropCount;

	/// The sub-step count the step ran at, echoed back. Every level-2 field is
	/// a sum over this many iterations, and comparing two profiles is
	/// meaningless without it.
	int subStepCount;
} b3Profile;

/// The most recent step, attributed.
///
/// Valid after b3World_Step returns and overwritten by the next one, so read
/// it once per frame in the same place the event arrays are read.
///
/// The timing fields are all zero when the library was built without
/// B3_NEA_PROFILE; the counters are filled either way.
B3_API b3Profile b3World_GetProfile( b3WorldId worldId );

/// Whether **the library** was built with the timing probes compiled in.
///
/// @section why Why this is a function and not the B3_NEA_PROFILE macro
///
/// Because a game cannot see that macro's real value. Makefile.blocksds passes
/// `-DB3_NEA_PROFILE=1` to `$(OBJS_BOX3D)` only -- the library's own
/// translation units -- while a game compiling against the *installed* headers
/// reads nea_config.h's `#ifndef` default of 0. The macro is therefore 1 inside
/// libNEA_box3d.a and 0 in every caller, whatever the library was built with.
///
/// That mismatch is not academic. b3World_Step arms timers 0 and 1 when the
/// probes are compiled in, so a caller that decides from the macro will keep
/// its own cpuStartTiming/cpuEndTiming bracket around the step and read a
/// counter the library re-armed underneath it. The number it gets back is
/// wrong, and wrong in a way that looks plausible.
///
/// So the question "are the probes there" has to be answered by the library,
/// at runtime, in a function the library compiled. Branch on this:
///
///     if ( b3IsProfilingEnabled() == false )
///         cpuStartTiming( 0 );
///     b3World_Step( worldId, subStepCount );
///     uint32_t ticks = b3IsProfilingEnabled()
///                        ? b3World_GetProfile( worldId ).totalTicks
///                        : cpuEndTiming();
///
/// It is a constant return, so the branch costs a call and a compare once per
/// frame -- and it cannot go stale the way a rebuilt library and a stale object
/// file can.
B3_API bool b3IsProfilingEnabled( void );

/**@}*/

//! @cond

// -------------------------------------------------------------------------
// The probe itself
// -------------------------------------------------------------------------
//
// Internal to the library. A game reads b3Profile; it does not place probes.

#if defined( B3_NEA_PROFILE ) && B3_NEA_PROFILE && defined( __NDS__ )

// cpuStartTiming/cpuGetTiming/cpuEndTiming live in <nds/timers.h>. base.h does
// not pull in <nds.h> -- b3fixed.h is what does that for the rest of the port
// -- and this header must stand on its own, because b3World_GetProfile is
// public API a game may include without touching the fixed-point layer.
#include <nds.h>

/// Carries the previous snapshot between marks. One per step, on the stack.
typedef struct b3ProfileTimer
{
	uint32_t last;
} b3ProfileTimer;

/// Arms the cascaded pair and zeroes the reference point.
///
/// This *takes over* timers 0 and 1 for the duration of the step, which is the
/// same pair libnds's cpuStartTiming uses. A caller that brackets b3World_Step
/// with its own cpuStartTiming/cpuEndTiming must drop that bracket in a
/// profiling build and read b3Profile::totalTicks instead -- the two cannot
/// coexist, because arming the timer here resets the caller's count.
B3_INLINE void b3ProfileStart( b3ProfileTimer* timer )
{
	cpuStartTiming( 0 );
	timer->last = 0;
}

/// Charges everything since the previous mark to *dst, and starts a new region.
B3_INLINE void b3ProfileMark( b3ProfileTimer* timer, uint32_t* dst )
{
	uint32_t now = cpuGetTiming();
	*dst = now - timer->last;
	timer->last = now;
}

/// As b3ProfileMark, but adds rather than assigns. For a stage visited once per
/// sub-step, where the field wanted is the total over the step.
B3_INLINE void b3ProfileAccum( b3ProfileTimer* timer, uint32_t* dst )
{
	uint32_t now = cpuGetTiming();
	*dst += now - timer->last;
	timer->last = now;
}

/// Elapsed since b3ProfileStart, without ending a region. For the total.
B3_INLINE uint32_t b3ProfileElapsed( const b3ProfileTimer* timer )
{
	(void)timer;
	return cpuGetTiming();
}

/// Closes the step and releases the timer pair back to the caller.
B3_INLINE void b3ProfileEnd( b3ProfileTimer* timer, uint32_t* total )
{
	*total = cpuEndTiming();
	timer->last = 0;
}

#define B3_PROFILE_START( timer ) b3ProfileStart( timer )
#define B3_PROFILE_MARK( timer, dst ) b3ProfileMark( timer, dst )
#define B3_PROFILE_ACCUM( timer, dst ) b3ProfileAccum( timer, dst )
#define B3_PROFILE_END( timer, total ) b3ProfileEnd( timer, total )

#else

// Off, or on the host, where there is no cascaded timer pair to read. An empty
// struct is a GCC extension the rest of the port already relies on; declaring
// one member keeps it strictly conforming and costs a byte of stack that the
// unused-variable elimination removes anyway.
typedef struct b3ProfileTimer
{
	uint32_t last;
} b3ProfileTimer;

#define B3_PROFILE_START( timer ) ( (void)0 )
#define B3_PROFILE_MARK( timer, dst ) ( (void)0 )
#define B3_PROFILE_ACCUM( timer, dst ) ( (void)0 )
#define B3_PROFILE_END( timer, total ) ( (void)0 )

#endif

// Level 2 and level 3 ride on the same probe, gated separately so that the
// cheap level-1 breakdown can be left on while the expensive ones are not.

#if defined( B3_NEA_PROFILE_SUBSTEP ) && B3_NEA_PROFILE_SUBSTEP
#define B3_PROFILE_SUBSTEP( timer, dst ) B3_PROFILE_ACCUM( timer, dst )
#else
#define B3_PROFILE_SUBSTEP( timer, dst ) ( (void)0 )
#endif

#if defined( B3_NEA_PROFILE_NARROW ) && B3_NEA_PROFILE_NARROW
#define B3_PROFILE_NARROW( timer, dst ) B3_PROFILE_ACCUM( timer, dst )
#else
#define B3_PROFILE_NARROW( timer, dst ) ( (void)0 )
#endif

//! @endcond
