// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2023 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_MAIN_H__
#define NEA_MAIN_H__

/// @file   NEAMain.h
/// @brief  Main header of Nitro Engine Advanced.

/// @defgroup global_defines Global definitions
///
/// Definitions related to debug features and version number of the library.
///
/// @{

#include <nds.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NEA_BLOCKSDS
# ifndef ARM_CODE
#  define ARM_CODE __attribute__((target("arm")))
# endif
#endif

#include "NEA2D.h"
#include "NEAAnimation.h"
#include "NEACamera.h"
#include "NEACollision.h"
#include "NEADisplayList.h"
#include "NEAFAT.h"
#include "NEAFormats.h"
#include "NEAGeneral.h"
#include "NEAGUI.h"
#include "NEAModel.h"
#include "NEAPalette.h"
#include "NEAPhysics.h"
#include "NEAPolygon.h"
#include "NEARichText.h"
#include "NEAText.h"
#include "NEATexture.h"
#include "NEABoneCollision.h"
#include "NEASound.h"

/// Major version of Nitro Engine Advanced
#define NITRO_ENGINE_ADVANCED_MAJOR (0)
/// Minor version of Nitro Engine Advanced
#define NITRO_ENGINE_ADVANCED_MINOR (15)
/// Patch version of Nitro Engine Advanced
#define NITRO_ENGINE_ADVANCED_PATCH (5)

/// Full version of Nitro Engine Advanced
#define NITRO_ENGINE_ADVANCED_VERSION ((NITRO_ENGINE_ADVANCED_MAJOR << 16) |  \
                              (NITRO_ENGINE_ADVANCED_MINOR << 8) |   \
                              (NITRO_ENGINE_ADVANCED_PATCH))

/// String with the version of Nitro Engine Advanced
#define NITRO_ENGINE_ADVANCED_VERSION_STRING "0.15.4"

/// @}

#ifdef __cplusplus
}
#endif

#endif // NEA_MAIN_H__
