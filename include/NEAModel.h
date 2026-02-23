// SPDX-License-Identifier: MIT
//
// Copyright (c) 2008-2022 Antonio Niño Díaz
//
// This file is part of Nitro Engine Advanced

#ifndef NEA_MODEL_H__
#define NEA_MODEL_H__

/// @file   NEAModel.h
/// @brief  Functions to draw and handle models.

/// @defgroup model_system Model handling system
///
/// Functions to create and manipulate animated or static models.
///
/// @{

#define NEA_NO_TEXTURE       -1 ///< Value that represents not having a texture
#define NEA_NO_MESH          -1 ///< Value that represents not having a mesh

#define NEA_DEFAULT_MODELS   512 ///< Default max number of models

/// Possible animation types.
typedef enum {
    NEA_ANIM_LOOP,    ///< When the end is reached it jumps to the start.
    NEA_ANIM_ONESHOT, ///< When the end is reached it stops.
} NEA_AnimationType;

/// Holds information of the animation of a model.
typedef struct {
    NEA_Animation *animation; ///< Pointer to animation file
    NEA_AnimationType type;   ///< Animation type.
    int32_t speed;           ///< Animation speed (f32).
    int32_t currframe;       ///< Current frame. It can be between frames (f32).
    int32_t numframes;       ///< Number of frames in the animation (int).
} NEA_AnimInfo;

/// Possible model types.
typedef enum {
    NEA_Static,  ///< Not animated.
    NEA_Animated ///< Animated.
} NEA_ModelType;

/// Holds information of a model.
typedef struct {
    NEA_ModelType modeltype;   ///< Model type (static or animated)
    int meshindex;            ///< Index of mesh (static or DSM)
    NEA_AnimInfo *animinfo[2]; ///< Animation information (two can be blended)
    int32_t anim_blend;       ///< Animation blend factor
    NEA_Material *texture;     ///< Material used by this model
    int x;                    ///< X position of the model (f32)
    int y;                    ///< Y position of the model (f32)
    int z;                    ///< Z position of the model (f32)
    int rx;                   ///< Rotation of the model by X axis
    int ry;                   ///< Rotation of the model by Y axis
    int rz;                   ///< Rotation of the model by Z axis
    int sx;                   ///< X scale of the model (f32)
    int sy;                   ///< Y scale of the model (f32)
    int sz;                   ///< Z scale of the model (f32)
    m4x3 *mat;                ///< Transformation matrix assigned by the user.
} NEA_Model;

/// Creates a new model object.
///
/// @param type Model type (static or animated).
/// @return Pointer to the newly created camera.
NEA_Model *NEA_ModelCreate(NEA_ModelType type);

/// Deletes a model.
///
/// @param model Pointer to the model.
void NEA_ModelDelete(NEA_Model *model);

/// Marks the mesh of a model to be freed when the model is deleted.
///
/// When a model is loaded with NEA_ModelLoadStaticMeshFAT() or
/// NEA_ModelLoadDSMFAT(), the mesh is read from storage into RAM to a buffer
/// allocated witn malloc(). This buffer is freed with free() when
/// NEA_ModelDelete() is called.
///
/// If the mesh is loaded using NEA_ModelLoadStaticMesh() or NEA_ModelLoadDSM(),
/// Nitro Engine Advanced can't tell if the user has loaded it from storage, or if it has
/// been included in the NDS file as data. If the user has loaded the mesh into
/// a buffer allocated with malloc(), the user is responsible for calling free()
/// after NEA_ModelDelete().
///
/// NEA_ModelFreeMeshWhenDeleted() gives you the option of skipping that last
/// free(). After you call this function, NEA_ModelDelete() will call free() for
/// you automatically so that you don't have to keep track of the pointer in the
/// logic of your program.
///
/// @param model Pointer to the model.
void NEA_ModelFreeMeshWhenDeleted(NEA_Model *model);

/// Assign a display list in RAM to a static model.
///
/// @param model Pointer to the model.
/// @param pointer Pointer to the display list.
/// @return It returns 1 on success, 0 on error.
int NEA_ModelLoadStaticMesh(NEA_Model *model, const void *pointer);

/// Loads a display list from a filesystem and assigns it to a static model.
///
/// @param model Pointer to the model.
/// @param path Path to the display list.
/// @return It returns 1 on success, 0 on error.
int NEA_ModelLoadStaticMeshFAT(NEA_Model *model, const char *path);

/// Assign a material to a model.
///
/// @param model Pointer to the model.
/// @param material Pointer to the material.
void NEA_ModelSetMaterial(NEA_Model *model, NEA_Material *material);

/// Assign an animation to a model.
///
/// @param model Pointer to the model.
/// @param anim Pointer to the animation.
void NEA_ModelSetAnimation(NEA_Model *model, NEA_Animation *anim);

/// Assign a secondary animation to a model.
///
/// The secondary animation is an animation that is averaged with the main
/// animation. This is useful to do transitions between animations. This takes a
/// bit more of CPU to display, though.
///
/// Whenever you want to go back to having one animation, you have to use
/// NEA_ModelAnimSecondaryClear(). This function lets you stop blending
/// animations, and it gives you the option to preserve the main or the
/// secondary animation.
///
/// Function NEA_ModelAnimSecondarySetFactor() lets you specify a value to
/// specify the blending factor, where 0.0 means "display the main animation
/// only" and 1.0 means "display the secondary animation only". The initial
/// value after calling NEA_ModelAnimSecondaryStart() is 0.0.
///
/// @param model Pointer to the model.
/// @param anim Pointer to the animation.
void NEA_ModelSetAnimationSecondary(NEA_Model *model, NEA_Animation *anim);

/// Draw a model.
///
/// @param model Pointer to the model.
void NEA_ModelDraw(const NEA_Model *model);

/// Clone model.
///
/// This clones the mesh, including the animation, the material it uses. It
/// doesn't actually duplicate the memory used in RAM to store the mesh, and it
/// keeps track of how many models use the same mesh. NEA_ModelDelete() only
/// frees the mesh when the user deletes the last model that uses it.
///
/// @param dest Pointer to the destination model.
/// @param source Pointer to the source model.
void NEA_ModelClone(NEA_Model *dest, NEA_Model *source);

/// Set position of a model.
///
/// @param model Pointer to the model.
/// @param x (x, y, z) Coordinates (f32).
/// @param y (x, y, z) Coordinates (f32).
/// @param z (x, y, z) Coordinates (f32).
void NEA_ModelSetCoordI(NEA_Model *model, int x, int y, int z);

/// Set position of a model.
///
/// @param m Pointer to the model.
/// @param x (x, y, z) Coordinates (float).
/// @param y (x, y, z) Coordinates (float).
/// @param z (x, y, z) Coordinates (float).
#define NEA_ModelSetCoord(m, x, y, z) \
    NEA_ModelSetCoordI(m, floattof32(x), floattof32(y), floattof32(z))

/// Set scale of a model.
///
/// @param model Pointer to the model.
/// @param x (x, y, z) Scale (f32).
/// @param y (x, y, z) Scale (f32).
/// @param z (x, y, z) Scale (f32).
void NEA_ModelScaleI(NEA_Model *model, int x, int y, int z);

/// Set scale of a model.
///
/// @param m Pointer to the model.
/// @param x (x, y, z) Scale (float).
/// @param y (x, y, z) Scale (float).
/// @param z (x, y, z) Scale (float).
#define NEA_ModelScale(m, x, y, z) \
    NEA_ModelScaleI(m, floattof32(x), floattof32(y), floattof32(z))

/// Translate a model.
///
/// @param model Pointer to the model.
/// @param x (x, y, z) Translate vector (f32).
/// @param y (x, y, z) Translate vector (f32).
/// @param z (x, y, z) Translate vector (f32).
void NEA_ModelTranslateI(NEA_Model *model, int x, int y, int z);

/// Translate a model.
///
/// @param m  Pointer to the model.
/// @param x (x, y, z) Translate vector (float).
/// @param y (x, y, z) Translate vector (float).
/// @param z (x, y, z) Translate vector (float).
#define NEA_ModelTranslate(m, x, y, z) \
    NEA_ModelTranslateI(m, floattof32(x), floattof32(y), floattof32(z))

/// Set rotation of a model.
///
/// This function sets the rotation of the model to the provided values.
///
/// @param model Pointer to the model.
/// @param rx Rotation by X axis (0 - 511).
/// @param ry Rotation by Y axis (0 - 511).
/// @param rz Rotation by Z axis (0 - 511).
void NEA_ModelSetRot(NEA_Model *model, int rx, int ry, int rz);

/// Rotate a model.
///
/// This function adds the values to the current rotation of the model.
///
/// @param model Pointer to the model.
/// @param rx Rotation by X axis (0 - 511).
/// @param ry Rotation by Y axis (0 - 511).
/// @param rz Rotation by Z axis (0 - 511).
void NEA_ModelRotate(NEA_Model *model, int rx, int ry, int rz);

/// Assigns a 4x3 transformation matrix to a model.
///
/// When a matrix is assigned, the scale, position and rotation of the model
/// will be ignored.
///
/// Note that the provided matrix is copied to the internal state of the model,
/// so the caller of the function doesn't need to keep it in memory.
///
/// @param model Pointer to the model.
/// @param mat Matrix to be used.
/// @return Returns 1 on success, 0 on failure.
int NEA_ModelSetMatrix(NEA_Model *model, m4x3 *mat);

/// Clears the 4x3 transformation matrix of a model.
///
/// The model will start using the individual scale, translation and rotation
/// values.
///
/// @param model Pointer to the model.
void NEA_ModelClearMatrix(NEA_Model *model);

/// Update internal state of the animation of all models.
void NEA_ModelAnimateAll(void);

/// Starts the animation of an animated model.
///
/// The speed can be positive or negative. A speed of 0 stops the animation, a
/// speed of 1 << 12 means that the model will advance one model frame per NDS
/// frame. Anything in between will advance less than one model frame per NDS
/// frame.
///
/// @param model Pointer to the model.
/// @param type Animation type (NEA_ANIM_LOOP / NEA_ANIM_ONESHOT).
/// @param speed Animation speed. (f32)
void NEA_ModelAnimStart(NEA_Model *model, NEA_AnimationType type, int32_t speed);

/// Starts the secondary animation of an animated model.
///
/// @param model Pointer to the model.
/// @param type Animation type (NEA_ANIM_LOOP / NEA_ANIM_ONESHOT).
/// @param speed Animation speed. (f32)
void NEA_ModelAnimSecondaryStart(NEA_Model *model, NEA_AnimationType type,
                                int32_t speed);

/// Sets animation speed.
///
/// The speed can be positive or negative. A speed of 0 stops the animation, a
/// speed of 1 << 12 means that the model will advance one model frame per NDS
/// frame. Anything in between will advance less than one model frame per NDS
/// frame.
///
/// @param model Pointer to the model.
/// @param speed New speed. (f32)
void NEA_ModelAnimSetSpeed(NEA_Model *model, int32_t speed);

/// Sets animation speed of the secondary animation.
///
/// @param model Pointer to the model.
/// @param speed New speed. (f32)
void NEA_ModelAnimSecondarySetSpeed(NEA_Model *model, int32_t speed);

/// Returns the current frame of an animated model.
///
/// @param model Pointer to the model.
/// @return Returns the frame in f32 format.
int32_t NEA_ModelAnimGetFrame(const NEA_Model *model);

/// Returns the current frame of the secondary animation of an animated model.
///
/// @param model Pointer to the model.
/// @return Returns the frame in f32 format.
int32_t NEA_ModelAnimSecondaryGetFrame(const NEA_Model *model);

/// Sets the current frame of an animated model.
///
/// @param model Pointer to the model.
/// @param frame Frame to set. (f32)
void NEA_ModelAnimSetFrame(NEA_Model *model, int32_t frame);

/// Sets the current frame of the secondary animation of an animated model.
///
/// @param model Pointer to the model.
/// @param frame Frame to set. (f32)
void NEA_ModelAnimSecondarySetFrame(NEA_Model *model, int32_t frame);

/// Sets the current blending factor between animations.
///
/// This is a value between 0.0 and 1.0 where 0.0 means "display the main
/// animation only" and 1.0 means "display the secondary animation only".
///
/// @param model Pointer to the model.
/// @param factor Blending factor to set. (f32)
void NEA_ModelAnimSecondarySetFactor(NEA_Model *model, int32_t factor);

/// Clears the secondary animation of a model.
///
/// It is possible to replace the main animation by the secondary animation, or
/// to simply remove the secondary animation. In both cases, the secondary
/// animation will be cleared.
///
/// @param model Pointer to the model.
/// @param replace_base_anim Set to true to replace the base animation by the
///                          secondary animation.
void NEA_ModelAnimSecondaryClear(NEA_Model *model, bool replace_base_anim);

/// Loads a DSM file stored in RAM to a model.
///
/// @param model Pointer to the model.
/// @param pointer Pointer to the file.
/// @return It returns 1 on success, 0 on error.
int NEA_ModelLoadDSM(NEA_Model *model, const void *pointer);

/// Loads a DSM file stored in a filesystem to a model.
///
/// @param model Pointer to the model.
/// @param path Path to the file.
/// @return It returns 1 on success, 0 on error.
int NEA_ModelLoadDSMFAT(NEA_Model *model, const char *path);

/// Deletes all models and frees all memory used by them.
void NEA_ModelDeleteAll(void);

/// Resets the model system and sets the maximun number of models.
///
/// @param max_models Number of models. If it is lower than 1, it will create
///                   space for NEA_DEFAULT_MODELS.
/// @return Returns 0 on success.
int NEA_ModelSystemReset(int max_models);

/// Ends model system and frees all memory used by it.
void NEA_ModelSystemEnd(void);

/// @}

#endif // NEA_MODEL_H__
