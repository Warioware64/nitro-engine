// SPDX-License-Identifier: Zlib OR MIT
//
// Copyright (c) 2024-2026 Antonio Niño Díaz

#include <stddef.h>
#include <stdint.h>

#include <dsf.h>

size_t DSF_UTF8_CodepointRead(const char *str, uint32_t *codepoint)
{
    // https://en.wikipedia.org/wiki/UTF-8#Encoding

    size_t size;
    uint32_t rune;

    uint32_t b1 = str[0];

    if ((b1 & 0x80) == 0)
    {
        size = 1;
        *codepoint = b1 & 0x7F;
        return 1;
    }
    else if ((b1 & 0xE0) == 0xC0)
    {
        size = 2;
        rune = b1 & 0x1F;
    }
    else if ((b1 & 0xF0) == 0xE0)
    {
        size = 3;
        rune = b1 & 0x0F;
    }
    else if ((b1 & 0xF8) == 0xF0)
    {
        size = 4;
        rune = b1 & 0x07;
    }
    else
    {
        goto error;
    }

    for (size_t i = 1; i < size; i++)
    {
        uint32_t b = str[i];
        if ((b & 0xC0) != 0x80)
            goto error;

        rune <<= 6;
        rune |= b & 0x3F;
    }

    *codepoint = rune;
    return size;

error:

    // Incorrect encoding. Advance characters until we find one character
    // that isn't a continuation character.
    size = 1;
    str++;

    while (1)
    {
        uint8_t c = *str++;
        size++;
        if ((c & 0xC0) != 0x80)
            break;
    }

    *codepoint = REPLACEMENT_CHARACTER;
    return size;
}

size_t DSF_UTF8_StringLength(const char *str)
{
    if (str == NULL)
        return 0;

    const char *readptr = str;

    size_t count = 0;

    while (*readptr != '\0')
    {
        uint32_t codepoint;
        size_t size = DSF_UTF8_CodepointRead(readptr, &codepoint);
        readptr += size;

        count++;
    }

    return count;
}

bool DSF_UTF8_StringStartsWith(const char *str, const uint32_t codepoints[])
{
    if ((str == NULL) || (codepoints == NULL))
        return false;

    uint32_t codepoint;
    DSF_UTF8_CodepointRead(str, &codepoint);

    // Check if this codepoint is a separator.
    for (int i = 0; ; i++)
    {
        uint32_t entry = codepoints[i];
        if (entry == 0)
            break;

        if (codepoint == entry)
            return true;
    }

    return false;
}

size_t DSF_UTF8_WordLength(const char *str, const uint32_t separators[])
{
    if (str == NULL)
        return 0;

    const uint32_t default_separators[] = {
        ' ', '\n', '\t', 0
    };

    // If the user hasn't specified separators, use the default set
    if (separators == NULL)
        separators = default_separators;

    const char *readptr = str;

    size_t count = 0;

    while (*readptr != '\0')
    {
        uint32_t codepoint;
        size_t size = DSF_UTF8_CodepointRead(readptr, &codepoint);
        readptr += size;

        // Check if this codepoint is a separator.
        for (int i = 0; ; i++)
        {
            uint32_t separator = separators[i];
            if (separator == 0)
                break;

            if (codepoint == separator)
                return count;
        }

        count++;
    }

    return count;
}
