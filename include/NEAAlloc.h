// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_ALLOC_H__
#define NEA_ALLOC_H__

/// @file   NEAAlloc.h
/// @brief  VRAM memory allocator
///
/// Internal use, used to handle VRAM when other function requests memory.

typedef enum {
    NEA_STATE_FREE,
    NEA_STATE_USED,
    NEA_STATE_LOCKED
} ne_chunk_state;

typedef struct NEAChunk {
    struct NEAChunk *previous; // Pointer to previous chunk. NULL if this is the first one
    struct NEAChunk *next;     // Pointer to next chunk. NULL if this is the last one
    ne_chunk_state state;     // Used, free or locked
    void *start, *end;        // Pointers to the start and end of this memory chunk
} NEAChunk;

typedef struct {
    // Values in bytes. Total memory does not include locked memory
    size_t free, used, total, locked;
    unsigned int free_percent; // Locked memory doesn't count
} NEAMemInfo;

#define NEA_ALLOC_MIN_SIZE (16)

// They return 0 on success. On error, they return a negative number.
int NEA_AllocInit(NEAChunk **first_element, void *start, void *end);
int NEA_AllocEnd(NEAChunk **first_element);

// This function takes a memory range defined by ["start", "end"] and tries to
// look a chunk of free memory that is at least as big as "size". It doesn't
// allocate it, that needs to be done with NEA_AllocAddress(). On error, this
// function returns NULL.
void *NEA_AllocFindInRange(NEAChunk *first_chunk, void *start, void *end, size_t size);

// It returns 0 on success. On error, it returns a negative number.
int NEA_AllocAddress(NEAChunk *first_chunk, void *address, size_t size);

// Allocates data at the first available space starting from the start of the
// memory pool. Returns NULL on error, or a valid pointer on success.
void *NEA_Alloc(NEAChunk *first_element, size_t size);

// Allocates data at the first available space starting from the end of the
// memory pool. Returns NULL on error, or a valid pointer on success.
void *NEA_AllocFromEnd(NEAChunk *first_element, size_t size);

// Returns 0 on success. On error, it returns a negative number.
int NEA_Free(NEAChunk *first_element, void *pointer);

// Only an allocated chunk of memory can be locked. After it is locked, it stops
// counting towards the total memory reported by NEA_MemGetInformation(). When it
// is unlocked, it is considered to still be allocated, and it needs to be freed
// manually.
//
// They return 0 on success. On error, they returns a negative number.
int NEA_Lock(NEAChunk *first_element, void *pointer);
int NEA_Unlock(NEAChunk *first_element, void *pointer);

// Returns 0 on success. On error, it returns a negative number.
int NEA_MemGetInformation(NEAChunk *first_element, NEAMemInfo *info);

#endif // NEA_ALLOC_H__
