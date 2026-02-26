// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced

// Example: Triangle mesh collision (ColMesh) with --collision generated data.
// A sphere and animated robot fall onto a teapot used as a colmesh obstacle.
// The robot uses per-bone collision (NEA_BoneCollisionData) for accurate
// collision with the sphere against its animated skeleton.

#include <NEAMain.h>

#include "teapot_bin.h"
#include "teapot_col_bin.h"
#include "sphere_bin.h"
#include "robot_dsm_bin.h"
#include "robot_walk_dsa_bin.h"
#include "robot_boncol_bin.h"
#include "texture.h"
#include "texture1.h"

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model[3];    // teapot, sphere, robot
    NEA_Physics *Physics[3];
    NEA_BoneCollisionData *BonCol;
    int hit_bone;           // last bone hit by sphere (-1 = none)
} SceneData;

static void ResetScene(SceneData *Scene)
{
    NEA_ModelSetCoord(Scene->Model[1], -1, 5, 0);
    NEA_PhysicsSetSpeed(Scene->Physics[1], 0, 0, 0);

    NEA_ModelSetCoord(Scene->Model[2], 2, 5, 0);
    NEA_PhysicsSetSpeed(Scene->Physics[2], 0, 0, 0);
}

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);

    // Teapot (green)
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[0]);

    // Sphere (blue)
    NEA_PolyFormat(31, 0, NEA_LIGHT_1, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[1]);

    // Robot (textured)
    NEA_PolyFormat(31, 0, NEA_LIGHT_0, NEA_CULL_BACK, 0);
    NEA_ModelDraw(Scene->Model[2]);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    // Reserve VRAM A+B for textures (C used by console)
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    Scene.Camera = NEA_CameraCreate();
    NEA_CameraSet(Scene.Camera,
                  -6, 5, -6,
                  0, 2, 0,
                  0, 1, 0);

    // --- Teapot: static colmesh obstacle ---

    Scene.Model[0] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[0], teapot_bin);
    NEA_ModelSetCoord(Scene.Model[0], 0, 0, 0);

    NEA_ColMesh *teapot_mesh = NEA_ColMeshLoad(teapot_col_bin);
    NEA_ColShape teapot_shape;
    NEA_ColShapeInitMesh(&teapot_shape, teapot_mesh);

    Scene.Physics[0] = NEA_PhysicsCreateEx(NEA_COL_TRIMESH);
    NEA_PhysicsSetModel(Scene.Physics[0], Scene.Model[0]);
    NEA_PhysicsSetColShape(Scene.Physics[0], &teapot_shape);
    NEA_PhysicsEnable(Scene.Physics[0], false); // static obstacle

    // --- Sphere: bouncing off the teapot ---

    Scene.Model[1] = NEA_ModelCreate(NEA_Static);
    NEA_ModelLoadStaticMesh(Scene.Model[1], sphere_bin);

    Scene.Physics[1] = NEA_PhysicsCreateEx(NEA_COL_SPHERE);
    NEA_PhysicsSetModel(Scene.Physics[1], Scene.Model[1]);
    NEA_PhysicsSetRadius(Scene.Physics[1], 0.5);
    NEA_PhysicsEnable(Scene.Physics[1], true);
    NEA_PhysicsSetGravity(Scene.Physics[1], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[1], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[1], 75);

    // --- Robot: animated model with capsule collider ---

    Scene.Model[2] = NEA_ModelCreate(NEA_Animated);
    NEA_ModelLoadMultiMesh(Scene.Model[2], robot_dsm_bin);

    NEA_Animation *anim = NEA_AnimationCreate();
    NEA_AnimationLoad(anim, robot_walk_dsa_bin);
    NEA_ModelSetAnimation(Scene.Model[2], anim);
    NEA_ModelAnimStart(Scene.Model[2], NEA_ANIM_LOOP, floattof32(0.1));

    // Material 0: "teapot" (A1RGB5, no palette)
    NEA_Material *Mat0 = NEA_MaterialCreate();
    NEA_MaterialSetName(Mat0, "teapot");
    NEA_MaterialTexLoad(Mat0, NEA_A1RGB5, 256, 256,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                            | NEA_TEXTURE_WRAP_T,
                        textureBitmap);

    // Material 1: "a3pal32" (paletted)
    NEA_Material *Mat1 = NEA_MaterialCreate();
    NEA_MaterialSetName(Mat1, "a3pal32");
    NEA_MaterialTexLoad(Mat1, NEA_A3PAL32, 256, 256,
                        NEA_TEXGEN_TEXCOORD | NEA_TEXTURE_WRAP_S
                            | NEA_TEXTURE_WRAP_T,
                        texture1Bitmap);

    NEA_Palette *PalMat1 = NEA_PaletteCreate();
    NEA_PaletteLoadSize(PalMat1, texture1Pal, texture1PalLen, NEA_A3PAL32);
    NEA_MaterialSetPalette(Mat1, PalMat1);

    NEA_ModelAutoBindMaterials(Scene.Model[2]);

    // Load per-bone collision data for the robot
    Scene.BonCol = NEA_BoneCollisionLoad(robot_boncol_bin);
    Scene.hit_bone = -1;

    // Robot still needs a capsule for physics (gravity, teapot collision)
    Scene.Physics[2] = NEA_PhysicsCreateEx(NEA_COL_CAPSULE);
    NEA_PhysicsSetModel(Scene.Physics[2], Scene.Model[2]);
    {
        NEA_ColShape capsule;
        NEA_ColShapeInitCapsule(&capsule, 0.5, 1.5);
        NEA_PhysicsSetColShape(Scene.Physics[2], &capsule);
    }
    NEA_PhysicsEnable(Scene.Physics[2], true);
    NEA_PhysicsSetGravity(Scene.Physics[2], 0.001);
    NEA_PhysicsOnCollision(Scene.Physics[2], NEA_ColBounce);
    NEA_PhysicsSetBounceEnergy(Scene.Physics[2], 50);

    ResetScene(&Scene);

    printf("ColMesh Demo\n\n");
    printf("Teapot: triangle mesh\n");
    printf("Blue:   sphere collider\n");
    printf("Robot:  bone collision\n\n");
    printf("START:Reset\n");
    printf("D-pad:Rotate L/R:Move X/Y:Up/Dn\n");

    NEA_LightSet(0, NEA_Green, -1, -1, 0);
    NEA_LightSet(1, NEA_Blue, -1, -1, 0);
    NEA_ClearColorSet(NEA_Red, 31, 63);

    while (1)
    {
        NEA_WaitForVBL(NEA_UPDATE_PHYSICS | NEA_UPDATE_ANIMATIONS);

        scanKeys();
        uint32_t keys = keysHeld();

        if (keysDown() & KEY_START)
            ResetScene(&Scene);

        if (keys & KEY_UP)    NEA_CameraRotateFree(Scene.Camera, 2, 0, 0);
        if (keys & KEY_DOWN)  NEA_CameraRotateFree(Scene.Camera, -2, 0, 0);
        if (keys & KEY_LEFT)  NEA_CameraRotateFree(Scene.Camera, 0, 2, 0);
        if (keys & KEY_RIGHT) NEA_CameraRotateFree(Scene.Camera, 0, -2, 0);
        if (keys & KEY_L)     NEA_CameraMoveFree(Scene.Camera, 0.1, 0, 0);
        if (keys & KEY_R)     NEA_CameraMoveFree(Scene.Camera, -0.1, 0, 0);
        if (keys & KEY_X)     NEA_CameraMoveFree(Scene.Camera, 0, 0, 0.1);
        if (keys & KEY_Y)     NEA_CameraMoveFree(Scene.Camera, 0, 0, -0.1);

        // Test sphere against robot's animated bones
        NEA_ColShape sphere_shape;
        NEA_ColShapeInitSphere(&sphere_shape, 0.5);
        NEA_Vec3 sphere_pos = NEA_Vec3Make(Scene.Model[1]->x,
                                           Scene.Model[1]->y,
                                           Scene.Model[1]->z);

        int hit_bone = -1;
        NEA_ColResult bone_result = NEA_BoneCollisionTest(
            Scene.Model[2], Scene.BonCol,
            &sphere_shape, sphere_pos, &hit_bone);

        Scene.hit_bone = hit_bone;
        
        if (hit_bone >= 0 && bone_result.depth > 0)
        {
            // Push sphere out along collision normal
            NEA_Vec3 push = NEA_Vec3Scale(bone_result.normal,
                                          bone_result.depth);
            Scene.Model[1]->x -= push.x;
            Scene.Model[1]->y -= push.y;
            Scene.Model[1]->z -= push.z;
        }

        printf("\x1b[10;0HBone hit: %3d  ", Scene.hit_bone);

        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
