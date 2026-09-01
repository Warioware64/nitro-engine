// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Loads a model and its texture out of an NPAC archive.
//
// The point of the example is what *isn't* here: once the archive is mounted,
// the loading calls are the ordinary NEA_*LoadFAT() ones, given a path that
// happens to start with "assets:/". Nothing between here and the archive knows
// the difference, and the members that the packer compressed are inflated on
// the way through.

#include <NEAMain.h>

#include <filesystem.h>

typedef struct {
    NEA_Camera *Camera;
    NEA_Model *Model;
} SceneData;

void Draw3DScene(void *arg)
{
    SceneData *Scene = arg;

    NEA_CameraUse(Scene->Camera);
    NEA_ModelDraw(Scene->Model);
}

static void WaitForStart(void)
{
    while (1)
    {
        NEA_WaitForVBL(0);

        scanKeys();
        if (keysHeld() & KEY_START)
            break;
    }
}

// Prints how a member ended up being stored. This reads the archive's index in
// RAM, so it costs no I/O and does not decompress anything.
static void PrintMember(const char *path, const char *label)
{
    NEA_NpacFileInfo info;

    if (NEA_NpacGetFileInfo(path, &info) != 0)
    {
        printf("%-12s <missing>\n", label);
        return;
    }

    const char *method;

    switch (info.method)
    {
        case NEA_NPAC_LZ77:
            method = "lz77";
            break;
        case NEA_NPAC_HUFF4:
        case NEA_NPAC_HUFF8:
            method = "huff";
            break;
        case NEA_NPAC_RLE:
            method = "rle";
            break;
        default:
            method = "-";
            break;
    }

    printf("%-12s %-4s %6lu %5lu\n", label, method,
           (unsigned long)info.size, (unsigned long)info.stored_size);
}

int main(int argc, char *argv[])
{
    SceneData Scene = { 0 };

    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    // Init Nitro Engine Advanced in normal 3D mode
    NEA_Init3D();

    // libnds uses VRAM_C for the text console, reserve A and B only
    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    if (!nitroFSInit(NULL))
    {
        printf("nitroFSInit failed.\nPress START to exit");
        WaitForStart();
        return 0;
    }

    // The archive is a plain file, so anything that can already be read works
    // as a place to keep it: NitroFS here, but "fat:/" or "sd:/" just as well.
    if (NEA_NpacMount("assets", "nitro:/assets.npac") != 0)
    {
        printf("NEA_NpacMount failed.\nPress START to exit");
        WaitForStart();
        return 0;
    }

    printf("assets.npac\n");
    printf("file         meth   size store\n");
    PrintMember("assets:/models/robot.bin", "robot.bin");
    PrintMember("assets:/textures/texture.img.bin", "texture.bin");
    PrintMember("assets:/readme.txt", "readme.txt");
    printf("\n");

    // Allocate space for the objects we'll use
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    // Set coordinates for the camera
    NEA_CameraSet(Scene.Camera,
                 -8, 3, 0,  // Position
                  0, 3, 0,  // Look at
                  0, 1, 0); // Up direction

    // Exactly the calls the loose-file version of this example makes. The only
    // thing that changed is the drive the paths name.
    NEA_ModelLoadStaticMeshFAT(Scene.Model, "assets:/models/robot.bin");
    NEA_MaterialTexLoadFAT(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                          "assets:/textures/texture.img.bin");

    // fopen() reaches the archive too, and a compressed member reads back as
    // the original file.
    FILE *f = fopen("assets:/readme.txt", "rb");
    if (f != NULL)
    {
        char line[64];
        size_t read = fread(line, 1, sizeof(line) - 1, f);
        line[read] = '\0';
        fclose(f);
        printf("%s", line);
    }

    // Assign texture to model...
    NEA_ModelSetMaterial(Scene.Model, Material);

    // We set up a light and its color
    NEA_LightSet(0, NEA_White, -0.5, -0.5, -0.5);

    while (1)
    {
        // Wait for next frame
        NEA_WaitForVBL(0);

        // Get keys information
        scanKeys();
        uint32_t keys = keysHeld();

        printf("\x1b[22;0HPad: Rotate.");

        // Rotate model using the pad
        if (keys & KEY_UP)
            NEA_ModelRotate(Scene.Model, 0, 0, -2);
        if (keys & KEY_DOWN)
            NEA_ModelRotate(Scene.Model, 0, 0, 2);
        if (keys & KEY_RIGHT)
            NEA_ModelRotate(Scene.Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NEA_ModelRotate(Scene.Model, 0, -2, 0);

        // Draw scene
        NEA_ProcessArg(Draw3DScene, &Scene);
    }

    return 0;
}
