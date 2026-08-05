// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026

// bake_ref -- the C baker behind a .colmesh -> .b3mesh command line.
//
// This is not a tool. The tool is `obj2dl.py --collision-b3`, which bakes a
// .b3mesh in Python. This exists so that the Python can be *checked*: it is
// mesh_bake.c -- the baker every run_pair mesh case and every test_world mode
// already exercises -- reachable from a shell, so test_bake_diff.py can bake a
// corpus both ways and compare the bytes.
//
// A second implementation of a binary contract is only safe if something
// compares the two. This is that something, and it lives in tests/ rather than
// tools/ because checking the Python is the whole of its job.
//
// It was tools/mesh2b3/mesh2b3.c. What survived the move is the .colmesh reader
// and the call into pdBakeMesh, which are the two things the Python has to
// agree with; the reporting, the C-array emitter and the validation all moved
// into b3mesh.py, where an author now meets them.
//
// Compiled in the port's **device mode**: bare int32_t, no shadow values, the
// same configuration Makefile.blocksds compiles the library with. Not a
// preference -- b3f is a struct carrying a double under B3_FIXED_DEBUG, so
// b3Vec3 and every offset computed from sizeof() would differ, and the blob
// compared here has to have the layout the ARM9 reads.

#include "mesh_bake.h"

#include "box3d/collision.h"
#include "box3d/math_fixed.h"
#include "box3d/types.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// =========================================================================
// The .colmesh format
// =========================================================================
//
// Written by generate_colmesh() in tools/obj2dl/obj2dl.py. Little-endian
// throughout, and every coordinate is a libnds f32 -- Q19.12, which is exactly
// b3f, so the numbers cross into the port without requantization.

#define COLM_MAGIC 0x4D4C4F43u // "COLM"
#define COLM_VERSION 1u

#define COLM_HEADER_BYTES 16 // magic, version, triangleCount, pad
#define COLM_BOUNDS_BYTES 24 // 6 x f32
#define COLM_TRIANGLE_BYTES 48 // 9 x f32 vertex + 3 x f32 face normal

static uint32_t readU32( const uint8_t* p )
{
	return (uint32_t)p[0] | ( (uint32_t)p[1] << 8 ) | ( (uint32_t)p[2] << 16 ) | ( (uint32_t)p[3] << 24 );
}

/// One f32 coordinate as a double. The cast through int32_t is the point: the
/// file stores the two's-complement bit pattern as an unsigned word.
static double readF32( const uint8_t* p )
{
	return (double)(int32_t)readU32( p ) / 4096.0;
}

// =========================================================================
// Files
// =========================================================================

static uint8_t* readFile( const char* path, long* sizeOut )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		fprintf( stderr, "bake_ref: %s: %s\n", path, strerror( errno ) );
		return NULL;
	}

	if ( fseek( f, 0, SEEK_END ) != 0 )
	{
		fprintf( stderr, "bake_ref: %s: not a seekable file\n", path );
		fclose( f );
		return NULL;
	}

	long size = ftell( f );
	rewind( f );

	if ( size <= 0 )
	{
		fprintf( stderr, "bake_ref: %s: empty\n", path );
		fclose( f );
		return NULL;
	}

	uint8_t* data = malloc( (size_t)size );
	if ( data == NULL )
	{
		fprintf( stderr, "bake_ref: out of memory reading %s\n", path );
		fclose( f );
		return NULL;
	}

	if ( fread( data, 1, (size_t)size, f ) != (size_t)size )
	{
		fprintf( stderr, "bake_ref: %s: short read\n", path );
		free( data );
		fclose( f );
		return NULL;
	}

	fclose( f );
	*sizeOut = size;
	return data;
}

static bool writeFile( const char* path, const void* data, int size )
{
	FILE* f = fopen( path, "wb" );
	if ( f == NULL )
	{
		fprintf( stderr, "bake_ref: %s: %s\n", path, strerror( errno ) );
		return false;
	}

	bool ok = fwrite( data, 1, (size_t)size, f ) == (size_t)size;
	if ( ok == false )
	{
		fprintf( stderr, "bake_ref: %s: write failed\n", path );
	}

	if ( fclose( f ) != 0 )
	{
		fprintf( stderr, "bake_ref: %s: %s\n", path, strerror( errno ) );
		ok = false;
	}

	return ok;
}

// =========================================================================
// .colmesh -> pdMesh
// =========================================================================

/// Fill `desc` from a .colmesh image, one vertex per triangle corner.
///
/// A .colmesh is a non-indexed triangle soup -- 9 f32 vertex coordinates plus a
/// face normal per triangle, no shared vertex array
/// (tools/obj2dl/obj2dl.py:generate_colmesh). The baker keys shared edges off
/// vertex *indices*, so without indices every edge would be a free rim and the
/// ghost filter would have nothing to work with.
///
/// Welding is left to the baker: it merges vertices that land on the same Q12
/// lattice point, which is precisely the right rule here. obj2dl writes the
/// same source OBJ vertex into every triangle that uses it, bit for bit, so
/// coincident corners quantize to the same lattice point and merge; anything
/// that does not merge really is a distinct vertex.
///
/// b3mesh.py's bake_colmesh() is the line-for-line counterpart of this
/// function, and the diff test depends on the two staying that way.
static bool loadColMesh( const uint8_t* data, long size, double scale, pdMesh* desc )
{
	if ( size < COLM_HEADER_BYTES + COLM_BOUNDS_BYTES )
	{
		fprintf( stderr, "bake_ref: input is too short to be a .colmesh\n" );
		return false;
	}

	uint32_t magic = readU32( data + 0 );
	uint32_t version = readU32( data + 4 );
	uint32_t triangleCount = readU32( data + 8 );

	if ( magic != COLM_MAGIC )
	{
		fprintf( stderr, "bake_ref: not a .colmesh (magic %08x, expected %08x)\n", magic, COLM_MAGIC );
		return false;
	}

	if ( version != COLM_VERSION )
	{
		fprintf( stderr, "bake_ref: .colmesh version %u, this reads %u\n", version, COLM_VERSION );
		return false;
	}

	long expected = COLM_HEADER_BYTES + COLM_BOUNDS_BYTES + (long)triangleCount * COLM_TRIANGLE_BYTES;
	if ( size != expected )
	{
		fprintf( stderr, "bake_ref: .colmesh claims %u triangles (%ld bytes) but the file is %ld\n", triangleCount,
				 expected, size );
		return false;
	}

	if ( triangleCount < 1 )
	{
		fprintf( stderr, "bake_ref: .colmesh has no triangles\n" );
		return false;
	}

	if ( (int)triangleCount > PD_MAX_MESH_TRIANGLES || (int)( 3 * triangleCount ) > PD_MAX_MESH_VERTICES )
	{
		fprintf( stderr, "bake_ref: %u triangles exceeds this build's cap of %d\n", triangleCount,
				 PD_MAX_MESH_TRIANGLES );
		return false;
	}

	desc->refId = 0;
	desc->triangleCount = (int)triangleCount;
	desc->vertexCount = 3 * (int)triangleCount;

	const uint8_t* tri = data + COLM_HEADER_BYTES + COLM_BOUNDS_BYTES;

	for ( int i = 0; i < (int)triangleCount; ++i )
	{
		for ( int corner = 0; corner < 3; ++corner )
		{
			// The face normal at the end of each triangle record is skipped:
			// the port derives normals from the stored vertices, and a normal
			// read from the file could disagree with the quantized triangle it
			// labels. That is the baker's decision 2, applied here.
			const uint8_t* v = tri + i * COLM_TRIANGLE_BYTES + corner * 12;
			int vertex = 3 * i + corner;

			desc->vertices[vertex].x = readF32( v + 0 ) / scale;
			desc->vertices[vertex].y = readF32( v + 4 ) / scale;
			desc->vertices[vertex].z = readF32( v + 8 ) / scale;

			desc->indices[vertex] = vertex;
		}
	}

	// Recomputed rather than read from the header, so that the scale is applied
	// once and in one place. Nothing in the bake reads these; they are here
	// because a pdMesh is supposed to describe its own extent.
	desc->lower = desc->vertices[0];
	desc->upper = desc->vertices[0];

	for ( int i = 1; i < desc->vertexCount; ++i )
	{
		pdVec3 p = desc->vertices[i];
		desc->lower.x = p.x < desc->lower.x ? p.x : desc->lower.x;
		desc->lower.y = p.y < desc->lower.y ? p.y : desc->lower.y;
		desc->lower.z = p.z < desc->lower.z ? p.z : desc->lower.z;
		desc->upper.x = p.x > desc->upper.x ? p.x : desc->upper.x;
		desc->upper.y = p.y > desc->upper.y ? p.y : desc->upper.y;
		desc->upper.z = p.z > desc->upper.z ? p.z : desc->upper.z;
	}

	return true;
}

// =========================================================================
// main
// =========================================================================

int main( int argc, char** argv )
{
	if ( argc < 3 || argc > 4 )
	{
		fprintf( stderr,
				 "bake_ref <input.colmesh> <output.b3mesh> [scale]\n"
				 "\n"
				 "The reference bake, for tests/box3d_host/test_bake_diff.py to compare\n"
				 "b3mesh.py against. Authors want obj2dl.py --collision-b3 instead.\n" );
		return 1;
	}

	const char* inputPath = argv[1];
	const char* outputPath = argv[2];
	double scale = 1.0;

	if ( argc == 4 )
	{
		char* end = NULL;
		scale = strtod( argv[3], &end );
		if ( end == argv[3] || *end != '\0' || !( scale > 0.0 ) )
		{
			fprintf( stderr, "bake_ref: scale wants a positive number, got '%s'\n", argv[3] );
			return 1;
		}
	}

	long inputSize = 0;
	uint8_t* input = readFile( inputPath, &inputSize );
	if ( input == NULL )
	{
		return 1;
	}

	// On the heap: a pdMesh at this build's caps is megabytes, which is not a
	// stack frame.
	pdMesh* desc = malloc( sizeof( pdMesh ) );
	if ( desc == NULL )
	{
		fprintf( stderr, "bake_ref: out of memory\n" );
		free( input );
		return 1;
	}

	if ( loadColMesh( input, inputSize, scale, desc ) == false )
	{
		free( desc );
		free( input );
		return 1;
	}

	free( input );

	int bound = pdBakeMeshSize( desc );
	void* blob = malloc( (size_t)bound );
	if ( blob == NULL )
	{
		fprintf( stderr, "bake_ref: out of memory for a %d byte blob\n", bound );
		free( desc );
		return 1;
	}

	int size = pdBakeMesh( desc, blob, bound );
	if ( size == 0 )
	{
		// The baker returns one number for every refusal. b3mesh.py says which
		// one it hit; here it is enough to say that it refused, because the
		// diff test only asks whether the two agree.
		fprintf( stderr, "bake_ref: pdBakeMesh refused %d triangles\n", desc->triangleCount );
		free( blob );
		free( desc );
		return 2;
	}

	bool ok = writeFile( outputPath, blob, size );

	free( blob );
	free( desc );
	return ok ? 0 : 1;
}
