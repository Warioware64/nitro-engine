// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// Two NPAC archives mounted at once, under names this file chose.
//
// Where npac_archive shows that the loaders do not need to change, this one
// shows the drive side of it: a directory tree walked with opendir()/readdir(),
// a working directory set with chdir() so relative paths resolve inside an
// archive, and an unmount.

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

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

// Walks a directory tree the way it would be walked on any other drive. The
// archive answers all of this out of its in-RAM index, without touching the
// file it was mounted from.
static void ListTree(const char *path, int depth)
{
    DIR *dir = opendir(path);
    if (dir == NULL)
        return;

    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL)
    {
        if ((strcmp(entry->d_name, ".") == 0) ||
            (strcmp(entry->d_name, "..") == 0))
            continue;

        // Built by hand rather than with snprintf() so the bound is
        // explicit: d_name is 256 bytes and would not always fit.
        char child[192];
        size_t path_len = strlen(path);
        size_t name_len = strlen(entry->d_name);

        // "docs:/" already ends in a slash, deeper paths do not.
        bool slash = (path_len > 0) && (path[path_len - 1] != '/');

        if (path_len + (slash ? 1 : 0) + name_len + 1 > sizeof(child))
            continue;

        memcpy(child, path, path_len);
        if (slash)
            child[path_len++] = '/';
        memcpy(child + path_len, entry->d_name, name_len + 1);

        if (entry->d_type == DT_DIR)
        {
            printf("%*s%s/\n", depth + 1, "", entry->d_name);
            ListTree(child, depth + 1);
            continue;
        }

        // stat() reports the decompressed size, which is the size a caller
        // would have to allocate for, not the size stored in the archive.
        struct stat st;
        if (stat(child, &st) != 0)
            st.st_size = 0;

        printf("%*s%-16s %5ld\n", depth + 1, "", entry->d_name,
               (long)st.st_size);
    }

    closedir(dir);
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

    // Both archives are mounted at the same time, under whatever names suit
    // the game. One libnds device serves all of them.
    if ((NEA_NpacMount("level", "nitro:/level.npac") != 0) ||
        (NEA_NpacMount("docs", "nitro:/docs.npac") != 0))
    {
        printf("NEA_NpacMount failed.\nPress START to exit");
        WaitForStart();
        return 0;
    }

    printf("mounted level: and docs:\n\n");

    printf("docs:/\n");
    ListTree("docs:/", 0);
    printf("\n");

    // chdir() into an archive, so relative paths resolve inside it.
    if (chdir("docs:/notes/deeper") == 0)
    {
        char cwd[128];
        if (getcwd(cwd, sizeof(cwd)) != NULL)
            printf("cwd %s\n", cwd);

        FILE *f = fopen("deep.txt", "rb");
        if (f != NULL)
        {
            char line[64];
            size_t read = fread(line, 1, sizeof(line) - 1, f);
            line[read] = '\0';
            fclose(f);
            printf("%s", line);
        }
    }

    // Leave the working directory somewhere that outlives the unmount below.
    chdir("nitro:/");

    NEA_NpacUnmount("docs");
    printf("unmounted docs: -> %s\n\n",
           NEA_NpacIsMounted("docs") ? "still there" : "gone");

    // Allocate space for the objects we'll use
    Scene.Model = NEA_ModelCreate(NEA_Static);
    Scene.Camera = NEA_CameraCreate();
    NEA_Material *Material = NEA_MaterialCreate();

    // Set coordinates for the camera
    NEA_CameraSet(Scene.Camera,
                 -8, 3, 0,  // Position
                  0, 3, 0,  // Look at
                  0, 1, 0); // Up direction

    // The other archive is untouched by that unmount and still loads.
    NEA_ModelLoadStaticMeshFAT(Scene.Model, "level:/models/robot.bin");
    NEA_MaterialTexLoadFAT(Material, NEA_A1RGB5, 256, 256, NEA_TEXGEN_TEXCOORD,
                          "level:/textures/texture.img.bin");

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
