// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Nitro Engine Advanced contributors, 2026
//
// This file is part of Nitro Engine Advanced

// Sphere-map environment mapping (NEA_TEXGEN_NORMAL).
//
// The DS can generate texture coordinates from the surface normal instead of
// reading them from the mesh. Point that at a circular "matcap" image and you
// get a reflection: the chrome look, at the cost of no extra polygons and no
// per-vertex work, because the coordinate generation happens in the same
// hardware that already transforms the normal for lighting.
//
// Three things have to line up, and this example lets you break each one so the
// failure is visible rather than described:
//
// 1. The material uses NEA_TEXGEN_NORMAL.
// 2. A texture matrix built from the current transform (NEA_ModelSetEnvMap()
//    makes NEA_ModelDraw() load one at the right moment). Without it the
//    reflection is frozen to the mesh.
// 3. Every texture coordinate in the mesh sits at the centre of the texture.
//    In this mode the hardware adds the mesh's coordinate to the generated one,
//    so it acts as the origin of the sphere map. Press Y to swap in a mesh
//    exported with its normal UVs and watch the reflection smear.
//
// Export meshes for this with `obj2dl.py --envmap-uv`. As a side effect the
// display list gets smaller: every texture coordinate is identical, so the
// packer emits one TEXCOORD command instead of one per vertex.
//
// Cost: one wait for the geometry engine per environment-mapped model per
// frame, to read back the vector matrix. Nothing per vertex, and no extra VRAM
// beyond the sphere map itself.

#include <NEAMain.h>

#include "matcap.h"
#include "teapot_env_bin.h"
#include "teapot_uv_bin.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Env, *Uv;
    NEA_Material *Matcap;
    bool centered_uvs;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    NEA_PolyFormat(31, 0, NEA_LIGHT_ALL, NEA_CULL_BACK, 0);

    // The two models differ only in their baked texture coordinates.
    NEA_ModelDraw(Scene->centered_uvs ? Scene->Env : Scene->Uv);
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

    Scene.Env = NEA_ModelCreate(NEA_Static);
    Scene.Uv = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    Scene.Matcap = NEA_MaterialCreate();
    Scene.centered_uvs = true;

    NEA_CameraSet(Scene.Camera,
                 0, 1, -3.5,
                 0, 0, 0,
                 0, 1, 0);

    NEA_ModelLoadStaticMesh(Scene.Env, teapot_env_bin);
    NEA_ModelLoadStaticMesh(Scene.Uv, teapot_uv_bin);

    // NEA_TEXGEN_NORMAL is what turns the texture into a sphere map. It can also
    // be switched at runtime with NEA_MaterialSetTexGen().
    NEA_MaterialTexLoad(Scene.Matcap, NEA_A1RGB5, 64, 64, NEA_TEXGEN_NORMAL,
                       matcapBitmap);

    NEA_ModelSetMaterial(Scene.Env, Scene.Matcap);
    NEA_ModelSetMaterial(Scene.Uv, Scene.Matcap);

    // The sphere map already contains all the shading, so the material should
    // not add any of its own: emission carries the texture through untouched.
    NEA_MaterialSetProperties(Scene.Matcap,
                             RGB15(0, 0, 0),    // diffuse
                             RGB15(0, 0, 0),    // ambient
                             RGB15(0, 0, 0),    // specular
                             RGB15(31, 31, 31), // emission
                             false, false);

    NEA_ModelSetEnvMap(Scene.Env, true);
    NEA_ModelSetEnvMap(Scene.Uv, true);

    bool envmap = true;
    bool spin = true;
    int rx = 0, ry = 0;
    int cam_angle = 0;

    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        uint32_t keys = keysDown();
        uint32_t held = keysHeld();

        if (keys & KEY_A)
        {
            // Without the matrix the generated coordinates stop following the
            // transform, so the reflection freezes onto the surface.
            envmap = !envmap;
            NEA_ModelSetEnvMap(Scene.Env, envmap);
            NEA_ModelSetEnvMap(Scene.Uv, envmap);
        }
        if (keys & KEY_Y)
            Scene.centered_uvs = !Scene.centered_uvs;
        if (keys & KEY_X)
        {
            // Switching the whole material back to ordinary UV mapping: the
            // teapot becomes a teapot with a picture of a chrome ball on it.
            static bool texgen_normal = true;
            texgen_normal = !texgen_normal;
            NEA_MaterialSetTexGen(Scene.Matcap,
                                 texgen_normal ? NEA_TEXGEN_NORMAL
                                               : NEA_TEXGEN_TEXCOORD);
        }
        if (keys & KEY_START)
            spin = !spin;

        if (spin)
            ry = (ry + 2) & 511;
        if (held & KEY_LEFT)
            ry = (ry - 4) & 511;
        if (held & KEY_RIGHT)
            ry = (ry + 4) & 511;
        if (held & KEY_UP)
            rx = (rx - 4) & 511;
        if (held & KEY_DOWN)
            rx = (rx + 4) & 511;

        // Orbiting the camera as well, because a matcap that only reacts to the
        // model is easy to mistake for a correct one.
        if (held & KEY_L)
            cam_angle = (cam_angle - 3) & 511;
        if (held & KEY_R)
            cam_angle = (cam_angle + 3) & 511;

        NEA_CameraSetI(Scene.Camera,
                      3 * sinLerp(cam_angle << 6), inttof32(1),
                      -3 * cosLerp(cam_angle << 6),
                      0, 0, 0,
                      0, inttof32(1), 0);

        NEA_ModelSetRot(Scene.Env, rx, ry, 0);
        NEA_ModelSetRot(Scene.Uv, rx, ry, 0);

        printf("\x1b[0;0H"
               "Environment mapping\n"
               "===================\n\n"
               "A   Texture matrix: %s\n"
               "Y   Mesh UVs:       %s\n"
               "X   Texgen mode:    toggle\n"
               "Start  Spin: %s\n"
               "Pad rotates, L/R orbits\n\n"
               "Turn the matrix off and the\n"
               "reflection freezes onto the\n"
               "surface. Switch the UVs and\n"
               "it smears: in this mode the\n"
               "mesh coordinate is the      \n"
               "origin of the sphere map.   \n",
               envmap ? "on " : "off",
               Scene.centered_uvs ? "centered" : "original",
               spin ? "on " : "off");

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
