// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#include <stdlib.h>

#include "NEAMain.h"
#include "NEAAlloc.h"

//-----------------------------------------------------------------------------
// Where chunk nodes come from
//-----------------------------------------------------------------------------

// Every split of a chunk needs a node and every merge gives one back, so an
// allocator in use goes through nodes all the time. By default they are
// malloc()ed one at a time, as they always were. NEA_AllocReserveChunks() sets
// aside blocks of them once instead; from then on a node comes off this free
// list and goes back on it, and the heap is only touched if the reserve runs
// out.

#define NE_POOL_BLOCKS_MAX  4

typedef struct {
    NEAChunk *start;
    int count;
} ne_pool_block_t;

static ne_pool_block_t ne_pool_blocks[NE_POOL_BLOCKS_MAX];
static int ne_pool_block_count = 0;
static NEAChunk *ne_pool_free = NULL;   // Linked through 'next'
static NEAAllocPoolStats ne_pool_stats;

static bool ne_pool_owns(const NEAChunk *chunk)
{
    for (int i = 0; i < ne_pool_block_count; i++)
    {
        const ne_pool_block_t *b = &ne_pool_blocks[i];
        if ((chunk >= b->start) && (chunk < b->start + b->count))
            return true;
    }

    return false;
}

static NEAChunk *ne_chunk_new(void)
{
    NEAChunk *chunk = ne_pool_free;

    if (chunk != NULL)
    {
        ne_pool_free = chunk->next;
        ne_pool_stats.in_use++;
        if (ne_pool_stats.in_use > ne_pool_stats.peak)
            ne_pool_stats.peak = ne_pool_stats.in_use;
        return chunk;
    }

    chunk = malloc(sizeof(NEAChunk));
    if ((chunk != NULL) && (ne_pool_stats.capacity > 0))
        ne_pool_stats.fallbacks++;

    return chunk;
}

static void ne_chunk_delete(NEAChunk *chunk)
{
    if (!ne_pool_owns(chunk))
    {
        free(chunk);
        return;
    }

    chunk->next = ne_pool_free;
    ne_pool_free = chunk;
    ne_pool_stats.in_use--;
}

int NEA_AllocReserveChunks(int count)
{
    if (count <= 0)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    if (ne_pool_block_count == NE_POOL_BLOCKS_MAX)
    {
        NEA_DebugPrint("Too many chunk reserves");
        return -2;
    }

    NEAChunk *block = malloc(sizeof(NEAChunk) * (size_t)count);
    if (block == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -3;
    }

    ne_pool_blocks[ne_pool_block_count].start = block;
    ne_pool_blocks[ne_pool_block_count].count = count;
    ne_pool_block_count++;

    for (int i = count - 1; i >= 0; i--)
    {
        block[i].next = ne_pool_free;
        ne_pool_free = &block[i];
    }

    ne_pool_stats.capacity += count;

    return 0;
}

void NEA_AllocGetPoolStats(NEAAllocPoolStats *stats)
{
    NEA_AssertPointer(stats, "NULL pointer");

    if (stats != NULL)
        *stats = ne_pool_stats;
}

//-----------------------------------------------------------------------------
// The allocator
//-----------------------------------------------------------------------------

int NEA_AllocInit(NEAChunk **first_chunk, void *start, void *end)
{
    if (first_chunk == NULL)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    if (end <= start)
    {
        NEA_DebugPrint("Invalid range");
        return -2;
    }

    *first_chunk = ne_chunk_new();
    if (*first_chunk == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return -3;
    }

    (*first_chunk)->previous = NULL;
    (*first_chunk)->state = NEA_STATE_FREE;
    (*first_chunk)->start = start;
    (*first_chunk)->end = end;
    (*first_chunk)->next = NULL;

    return 0;
}

int NEA_AllocEnd(NEAChunk **first_chunk)
{
    if (first_chunk == NULL)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    NEAChunk *this = *first_chunk;

    while (this != NULL)
    {
        NEAChunk *next = this->next;
        ne_chunk_delete(this);
        this = next;
    }

    *first_chunk = NULL;
    return 0;
}

// The function resizes this chunk to the provided size, and creates a new chunk
// with the remaining size.
//
// +-----------------+------+
// |      THIS       | NEXT |  Before
// +-----------------+------+
//
// +------+----------+------+
// | THIS |   NEW    | NEXT |  After
// +------+----------+------+
//
// It returns a pointer to the new chunk.
static NEAChunk *ne_split_chunk(NEAChunk *this, size_t this_size)
{
    NEA_AssertPointer(this, "NULL pointer");

    // Get next chunk and create a new one.

    NEAChunk *next = this->next;

    NEAChunk *new = ne_chunk_new();
    if (new == NULL)
    {
        NEA_DebugPrint("Not enough memory");
        return NULL;
    }

    // Update pointers in the linked list
    // ----------------------------------

    new->previous = this;
    this->next = new;

    if (next == NULL)
    {
        new->next = NULL;
    }
    else
    {
        new->next = next;
        next->previous = new;

        // It shouldn't be free because deallocating a chunk should merge it
        // with any free chunk next to it.
        NEA_Assert(next->state != NEA_STATE_FREE, "Possible list corruption");
    }

    // Update pointers to start and end of this chunk and the new chunk
    // ----------------------------------------------------------------

    new->end = this->end;
    this->end = (void *)((uintptr_t)this->start + this_size);
    new->start = this->end;

    return new;
}

// This returns a pointer to the chunk that contains the provided address.
// The start address of the chunk is considered to be part of that chunk, but
// the end address isn't considered part of that chunk.
static NEAChunk *ne_search_address(NEAChunk *first_chunk, void *address)
{
    NEA_AssertPointer(first_chunk, "NULL pointer");

    NEAChunk *this = first_chunk;

    uintptr_t addr = (uintptr_t)address;

    for ( ; this != NULL; this = this->next)
    {
        uintptr_t start = (uintptr_t)this->start;
        uintptr_t end = (uintptr_t)this->end;

        // We've gone too far. This pointer was likely before the allocated
        // memory pool.
        if (addr < start)
            return NULL;

        if (addr < end)
            return this;
    }

    return NULL;
}

void *NEA_AllocFindInRange(NEAChunk *first_chunk, void *start, void *end, size_t size)
{
    if ((first_chunk == NULL) || (start == NULL) || (end == NULL) || (size == 0))
    {
        NEA_DebugPrint("Invalid arguments");
        return NULL;
    }

    // Get the chunk that contains the first address. If the start address of
    // the first chunk is after the provided start, get the first chunk.
    NEAChunk *this;
    if (start < first_chunk->start)
        this = first_chunk;
    else
        this = ne_search_address(first_chunk, start);

    uintptr_t range_end = (uintptr_t)end;

    for ( ; this != NULL; this = this->next)
    {
        // Is this free?
        if (this->state != NEA_STATE_FREE)
            continue;

        // "start" is inside "this", but "this->start" may be before "start". In
        // that case, we need to calculate the size actually inside the range
        // provided by the user.
        uintptr_t real_start;
        if (this->start < start)
            real_start = (uintptr_t)start;
        else
            real_start = (uintptr_t)this->start;

        uintptr_t this_end = (uintptr_t)this->end;
        size_t this_size = this_end - real_start;

        // Check if the requested chunk would fit here
        if (this_size < size)
            continue;

        // Check if the expected end of the allocated chunk is within the limits
        // provided by the user. If so, we've gone over the limit, there is no
        // need to keep searching.
        uintptr_t expected_end = real_start + size;
        if (expected_end > range_end)
            return NULL;

        return (void *)real_start;
    }

    return NULL;
}

// This function searches the list and returns a chunk that contains the
// specified range of memory (address, address + size) if it is free.
static NEAChunk *ne_search_free_range_chunk(NEAChunk *first_chunk,
                                           void *address, size_t size)
{
    NEA_AssertPointer(first_chunk, "NULL pointer");

    // If that range of memory is free, it should be in one single chunk. Look
    // for the chunk that contains the base address, and check if the end
    // address is also part of that chunk.

    NEAChunk *chunk = ne_search_address(first_chunk, address);
    if (chunk == NULL)
        return NULL;

    uintptr_t end_address = (uintptr_t)address + size;
    uintptr_t chunk_end = (uintptr_t)chunk->end;

    if (end_address > chunk_end)
        return NULL;

    if (chunk->state != NEA_STATE_FREE)
        return NULL;

    return chunk;
}

int NEA_AllocAddress(NEAChunk *first_chunk, void *address, size_t size)
{
    if ((first_chunk == NULL) || (address == NULL) || (size == 0))
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    // Force sizes multiple of NEA_ALLOC_MIN_SIZE
    const size_t mask = NEA_ALLOC_MIN_SIZE - 1;
    if ((size & mask) != 0)
        size += NEA_ALLOC_MIN_SIZE - (size & mask);

    // Get a free chunk that contains this range of memory. This function
    // returns NULL if there is no chunk that contains this range entirely.
    // It also returns NULL if it isn't free.
    NEAChunk *this = ne_search_free_range_chunk(first_chunk, address, size);
    if (this == NULL)
        return -2;

    uintptr_t alloc_start = (uintptr_t)address;
    uintptr_t alloc_end = alloc_start + size;

    uintptr_t this_start = (uintptr_t)this->start;
    uintptr_t this_end = (uintptr_t)this->end;

    // If this point is reached we have a free chunk such that:
    //
    //     this_start <= alloc_start  &&  alloc_end <= this_end
    //
    // - If the start address isn't the same, we need to create a new chunk.
    //
    // - If the end address isn't the same, we need to create another chunk.
    //
    // Finally, return the chunk that stays in the middle.

    bool this_is_modified = false;

    if (this_start < alloc_start)
    {
        // Split this chunk into two, ignore the first one and get the second
        // one (which contains the start address)
        this = ne_split_chunk(this, alloc_start - this_start);
        if (this == NULL)
            return -3;

        this_is_modified = true;

        // Only the start has changed
        this_start = (uintptr_t)this->start;

        NEA_Assert(this_end == (uintptr_t)this->end, "Unexpected error");
    }

    this->state = NEA_STATE_USED;

    if (alloc_end < this_end)
    {
        // Split this chunk into two as well. The first one is the final desired
        // chunk, the second one is more free space.
        NEAChunk *next = ne_split_chunk(this, size);
        if (next == NULL)
        {
            if (this_is_modified)
                NEA_Free(first_chunk, this);

            return -4;
        }

        next->state = NEA_STATE_FREE;

        // Only the end has changed
        this_end = (uintptr_t)this->end;

        NEA_Assert(this_start == (uintptr_t)this->start, "Unexpected error");
    }

    NEA_Assert(size == (this_end - this_start), "Unexpected error");
    NEA_Assert(this->start == address, "Unexpected error");

    return 0;
}

void *NEA_Alloc(NEAChunk *first_chunk, size_t size)
{
    if ((first_chunk == NULL) || (size == 0))
    {
        NEA_DebugPrint("Invalid arguments");
        return NULL;
    }

    // Force sizes multiple of NEA_ALLOC_MIN_SIZE
    const size_t mask = NEA_ALLOC_MIN_SIZE - 1;
    if ((size & mask) != 0)
        size += NEA_ALLOC_MIN_SIZE - (size & mask);

    NEAChunk *this = first_chunk;

    for ( ; this != NULL; this = this->next)
    {
        // Skip non-free chunks
        if (this->state != NEA_STATE_FREE)
            continue;

        size_t this_size = (size_t)this->end - (size_t)this->start;

        // If this chunk doesn't have enough space, simply skip it.
        if (this_size < size)
            continue;

        // If we have exactly the space requested, we're done.
        if (this_size == size)
        {
            this->state = NEA_STATE_USED;
            return this->start;
        }

        // If we have more space than requested, split this chunk:
        //
        // |      THIS       | NEXT |
        // +-----------------+------+  Before
        // |    NOT USED     | USED |
        //
        // | THIS |   NEW    | NEXT |
        // +------+----------+------+  After
        // | USED | NOT USED | USED |

        NEAChunk *new = ne_split_chunk(this, size);
        if (new == NULL)
            return NULL;

        // Flag this chunk as used and the new one as free
        this->state = NEA_STATE_USED;
        new->state = NEA_STATE_FREE;

        return this->start;
    }

    // No more chunks... Not enough free space.
    return NULL;
}

void *NEA_AllocFromEnd(NEAChunk *first_chunk, size_t size)
{
    if ((first_chunk == NULL) || (size == 0))
    {
        NEA_DebugPrint("Invalid arguments");
        return NULL;
    }

    // Force sizes multiple of NEA_ALLOC_MIN_SIZE
    const size_t mask = NEA_ALLOC_MIN_SIZE - 1;
    if ((size & mask) != 0)
        size += NEA_ALLOC_MIN_SIZE - (size & mask);

    // Find last chunk
    NEAChunk *this = first_chunk;
    while (this->next != NULL)
        this = this->next;

    // Traverse list from end to beginning
    for ( ; this != NULL; this = this->previous)
    {
        // Skip non-free chunks
        if (this->state != NEA_STATE_FREE)
            continue;

        size_t this_size = (size_t)this->end - (size_t)this->start;

        // If this chunk doesn't have enough space, simply skip it.
        if (this_size < size)
            continue;

        // If we have exactly the space requested, we're done.
        if (this_size == size)
        {
            this->state = NEA_STATE_USED;
            return this->start;
        }

        // If we have more space than requested, split this chunk:
        //
        // |        THIS      | NEXT |
        // +------------------+------+  Before
        // |      NOT USED    | USED |
        //
        // |    THIS   | NEW  | NEXT |
        // +-----------+------+------+  After
        // |  NOT USED | USED | USED |

        // The size of this chunk has to be the current one minus the requested
        // size for the new chunk.
        NEAChunk *new = ne_split_chunk(this, this_size - size);
        if (new == NULL)
            return NULL;

        // Flag the new chunk as used
        new->state = NEA_STATE_USED;

        return new->start;
    }

    // No more chunks... Not enough free space.
    return NULL;
}

int NEA_Free(NEAChunk *first_chunk, void *pointer)
{
    if (first_chunk == NULL)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    NEAChunk *this = first_chunk;

    // Look for the chunk that corresponds to the given pointer
    while (1)
    {
        // Check if we have reached the end without finding the chunk
        if (this == NULL)
            return -2;

        // If this is the chunk we're looking for, exit loop
        if (this->start == pointer)
            break;

        this = this->next;
    }

    // If the specified chunk is free or locked, it can't be freed.
    if (this->state != NEA_STATE_USED)
        return -3;

    // Chunk found. Free it.
    this->state = NEA_STATE_FREE;

    // Now, check if we can join this free chunk with the previous or the
    // next one
    NEAChunk *previous = this->previous;
    NEAChunk *next = this->next;

    // Check the previous one
    if (previous && previous->state == NEA_STATE_FREE)
    {
        // We can join them
        //
        // | PREVIOUS |   THIS   | NEXT |
        // +----------+----------+------+
        // | NOT USED | NOT USED | ???? |
        //
        // |       PREVIOUS      | NEXT |
        // +---------------------+------+
        // |       NOT USED      | ???? |

        if (next)
        {
            // First, join the previous and the next
            next->previous = previous;
            previous->next = next;
        }
        else
        {
            previous->next = NULL;
        }

        // Expand the previous one
        previous->end = this->end;

        // Delete current chunk
        ne_chunk_delete(this);

        // Change the active chunk to try to join it with the next one.
        this = previous;
    }

    // Check the next one
    if (next && next->state == NEA_STATE_FREE)
    {
        // We can join them
        //
        // |   THIS   |   NEXT   | NEXT NEXT |
        // +----------+----------+-----------+  Before
        // | NOT USED | NOT USED |   USED    |
        //
        // |         THIS        | NEXT NEXT |
        // +---------------------+-----------+  After
        // |       NOT USED      |   USED    |

        NEAChunk *next_next = next->next;

        if (next_next)
        {
            // Next Next should be used or locked. If not,
            // something bad is happening here.
            NEA_Assert(next_next->state != NEA_STATE_FREE,
                      "Possible list corruption. (2)");

            // First, join this chunk and the next next chunk
            next_next->previous = this;
            this->next = next_next;
        }
        else
        {
            this->next = NULL;
        }

        // Expand this node one
        this->end = next->end;

        // Delete next chunk
        ne_chunk_delete(next);
    }

    return 0;
}

int NEA_Lock(NEAChunk *first_chunk, void *pointer)
{
    if (first_chunk == NULL)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    NEAChunk *this = first_chunk;

    while (1)
    {
        if (this->start == pointer)
        {
            // Check if we are trying to lock a chunk that isn't in use
            if (this->state != NEA_STATE_USED)
                return -2;

            this->state = NEA_STATE_LOCKED;
            return 0;
        }

        // Couldn't find a chunk at the specified address
        if (this->next == NULL)
            return -3;

        this = this->next;
    }
}

int NEA_Unlock(NEAChunk *first_chunk, void *pointer)
{
    if (first_chunk == NULL)
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    NEAChunk *this = first_chunk;

    while (1)
    {
        if (this->start == pointer)
        {
            // Check if we are trying to unlock a chunk that isn't locked
            if (this->state != NEA_STATE_LOCKED)
                return -2;

            this->state = NEA_STATE_USED;
            return 0;
        }

        // Couldn't find a chunk at the specified address
        if (this->next == NULL)
            return -3;

        this = this->next;
    }
}

int NEA_MemGetInformation(NEAChunk *first_chunk, NEAMemInfo *info)
{
    if ((first_chunk == NULL) || (info == NULL))
    {
        NEA_DebugPrint("Invalid arguments");
        return -1;
    }

    info->free = 0;
    info->used = 0;
    info->total = 0;
    info->locked = 0;

    NEAChunk *this = first_chunk;

    for ( ; this != NULL; this = this->next)
    {
        size_t size = (uintptr_t)this->end - (uintptr_t)this->start;

        switch (this->state)
        {
            case NEA_STATE_FREE:
                info->free += size;
                info->total += size;
                break;
            case NEA_STATE_USED:
                info->used += size;
                info->total += size;
                break;
            case NEA_STATE_LOCKED:
                // Locked memory isn't added to the total
                info->locked += size;
                break;
            default:
                return -2;
                break;
        }
    }

    info->free_percent = (info->free * 100) / info->total;
    return 0;
}
