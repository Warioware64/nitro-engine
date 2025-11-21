// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine

#include <nds.h>

// Internal math functions

static inline
void ne_div_start(int32_t num, int32_t den)
{
    div32_asynch(num, den);
}

static inline
int32_t ne_div_result(void)
{
    return div32_result();
}
