// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Fog density curves (NEA_FogEnableCurve).
//
// The older NEA_FogEnable() takes a hardware `shift` and a `mass`, which have to
// be found by trial and error. NEA_FogEnableCurve() instead takes the two
// distances you actually care about -- where fog starts and where it becomes
// opaque -- plus the shape of the ramp in between, and derives the hardware
// shift itself.
//
// Three spheres are placed at increasing distance so the shape of each ramp is
// easy to read: with EXP the middle sphere is already almost gone, with SQUARED
// it is still mostly visible.
//
// Cost: nothing per frame. The 32-entry density table is built and uploaded
// once per NEA_FogEnableCurve() call, and no VRAM is used.

#include <NEAMain.h>

#include "texture.h"
#include "sphere_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model, *Model2, *Model3;
} SceneData;

static const char *CURVE_NAMES[] = {
    "LINEAR    ", "SQUARED   ", "EXP       ", "SMOOTHSTEP"
};

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // Fog is per-polygon: NEA_FOG_ENABLE has to be in the polygon format or the
    // fog unit skips the pixels.
    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, NEA_FOG_ENABLE);

    NEA_ModelDraw(Scene->Model);
    NEA_ModelDraw(Scene->Model2);
    NEA_ModelDraw(Scene->Model3);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();
    // Use banks A and B for textures. libnds uses bank C for the text console.
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Model2 = NEA_ModelCreate(NEA_Static);
    Scene.Model3 = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    NEA_CameraSet(Scene.Camera,
                 -1, 2, -1,
                  1, 1, 1,
                  0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Model, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model2, sphere_bin);
    NEA_ModelLoadStaticMesh(Scene.Model3, sphere_bin);

    NEA_MaterialTexLoad(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                       textureBitmap);

    NEA_ModelSetMaterial(Scene.Model, Material);
    NEA_ModelSetMaterial(Scene.Model2, Material);
    NEA_ModelSetMaterial(Scene.Model3, Material);

    NEA_LightSet(0, NEA_White, 0, -1, -1);

    NEA_ModelSetCoord(Scene.Model, 1, 0, 1);
    NEA_ModelSetCoord(Scene.Model2, 3, 1, 3);
    NEA_ModelSetCoord(Scene.Model3, 7, 2, 7);

    // Fog starts 2 units from the camera and is opaque at 12. The fog unit
    // compares raw depth-buffer values, not world units, and what those mean
    // depends on the depth buffer mode -- NEA_FogDepthFromDistance() does that
    // conversion for the mode that is actually active.
    int curve = NEA_FOG_LINEAR;
    int near_units = 2;
    int far_units = 12;
    u32 color = NEA_Black;
    bool dirty = true;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();

        if (keys & KEY_A)
        {
            curve = (curve + 1) & 3;
            dirty = true;
        }
        if (keys & (KEY_UP | KEY_DOWN))
        {
            far_units += (keys & KEY_UP) ? 1 : -1;
            if (far_units <= near_units)
                far_units = near_units + 1;
            if (far_units > 60)
                far_units = 60;
            dirty = true;
        }
        if (keys & (KEY_RIGHT | KEY_LEFT))
        {
            near_units += (keys & KEY_RIGHT) ? 1 : -1;
            if (near_units < 0)
                near_units = 0;
            if (near_units >= far_units)
                near_units = far_units - 1;
            dirty = true;
        }
        if (keys & KEY_START)
        {
            color = NEA_Black;
            dirty = true;
        }
        if (keys & KEY_SELECT)
        {
            color = NEA_White;
            dirty = true;
        }

        // Only re-upload the table when something actually changed. It is cheap
        // enough to call every frame, but there is no reason to.
        if (dirty)
        {
            NEA_FogEnableCurve(curve, color, 31,
                              NEA_FogDepthFromDistance(inttof32(near_units)),
                              NEA_FogDepthFromDistance(inttof32(far_units)));
            dirty = false;
        }

        printf("\x1b[0;0H"
               "Fog density curves\n"
               "==================\n\n"
               "A         Curve: %s\n"
               "Right/Left Near: %2d units \n"
               "Up/Down     Far: %2d units \n"
               "Select/Start  Color\n\n"
               "Spheres sit at ~1.7, ~5 and\n"
               "~11 units from the camera.\n\n"
               "depth 0x%04X .. 0x%04X\n",
               CURVE_NAMES[curve], near_units, far_units,
               (unsigned)NEA_FogDepthFromDistance(inttof32(near_units)),
               (unsigned)NEA_FogDepthFromDistance(inttof32(far_units)));

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
