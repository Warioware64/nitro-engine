# MD5 Import/Export Addon for Blender 5.0
# Original authors: nemyax, Samson
# Copyright (c) Antonio Nino Diaz, Warioware64
# Nitro Engine Advanced (NEA) fork

bl_info = {
    "name": "MD5 format (NDS/NEA)",
    "author": "nemyax, Samson, Warioware64",
    "version": (2, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import-Export",
    "description": "Import and export md5mesh and md5anim (Nitro Engine Advanced)",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Import-Export"}

import bpy
import os
import bmesh
import os.path
import mathutils as mu
import math
import re


from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    StringProperty,
    IntProperty,
    CollectionProperty)
from bpy_extras.io_utils import (
    ExportHelper,
    ImportHelper,
    path_reference_mode)
from bpy.types import (
    Operator,
    OperatorFileListElement)


msgLines = [] # global for error messages
prerequisites = None # global for exportable objects
bpy.types.Scene.md5_bone_collection = StringProperty(
        name="Bone Collection",
        description="Bone collection name used for MD5 export",
        default="MD5_Export")


###
### Bone collision system
###

class NEA_BoneCollisionProps(bpy.types.PropertyGroup):
    """Per-bone collision shape properties for NEA physics."""
    col_type: EnumProperty(
        items=[
            ('none', 'None', 'No collision shape'),
            ('sphere', 'Sphere', 'Spherical collision shape'),
            ('capsule', 'Capsule', 'Capsule collision shape (Y-axis)'),
            ('aabb', 'AABB', 'Axis-aligned bounding box'),
        ],
        name="Type",
        description="Collision shape type for this bone",
        default='none',
    )
    radius: FloatProperty(
        name="Radius",
        description="Collision sphere or capsule radius",
        default=0.5, min=0.01,
    )
    half_height: FloatProperty(
        name="Half Height",
        description="Capsule half-height along Y axis",
        default=0.5, min=0.01,
    )
    half_x: FloatProperty(name="Half X", default=0.5, min=0.01)
    half_y: FloatProperty(name="Half Y", default=0.5, min=0.01)
    half_z: FloatProperty(name="Half Z", default=0.5, min=0.01)
    offset_x: FloatProperty(name="X", default=0.0)
    offset_y: FloatProperty(name="Y", default=0.0)
    offset_z: FloatProperty(name="Z", default=0.0)


# Viewport overlay draw handler
_collision_draw_handler = None


def _make_circle_points(center, radius, axis, segments=16):
    """Generate circle vertices around center in the given plane."""
    pts = []
    for i in range(segments + 1):
        a = 2 * math.pi * i / segments
        c_a = math.cos(a) * radius
        s_a = math.sin(a) * radius
        if axis == 'X':
            v = mu.Vector((0, c_a, s_a))
        elif axis == 'Y':
            v = mu.Vector((c_a, 0, s_a))
        else:
            v = mu.Vector((c_a, s_a, 0))
        pts.append(center + v)
    return pts


def _draw_collision_overlays():
    """GPU draw callback for bone collision wireframe visualization."""
    import gpu
    from gpu_extras.batch import batch_for_shader

    context = bpy.context
    obj = context.active_object
    if not obj or obj.type != 'ARMATURE':
        return

    scene = context.scene
    if not getattr(scene, 'nea_show_collision', False):
        return

    arm = obj.data
    wm = obj.matrix_world
    lines = []

    for bone in arm.bones:
        props = bone.nea_collision
        if props.col_type == 'none':
            continue

        # Bone center in world space = bone head + offset transformed by bone
        bone_mat = wm @ bone.matrix_local
        offset = mu.Vector((props.offset_x, props.offset_y, props.offset_z))
        center = bone_mat @ offset

        if props.col_type == 'sphere':
            for axis in ('X', 'Y', 'Z'):
                pts = _make_circle_points(center, props.radius, axis)
                for i in range(len(pts) - 1):
                    lines.extend([pts[i][:], pts[i + 1][:]])

        elif props.col_type == 'capsule':
            r = props.radius
            hh = props.half_height
            rot3 = bone_mat.to_3x3()
            top = center + rot3 @ mu.Vector((0, hh, 0))
            bot = center + rot3 @ mu.Vector((0, -hh, 0))
            for c in (top, bot):
                for axis in ('X', 'Z'):
                    pts = _make_circle_points(c, r, axis)
                    for i in range(len(pts) - 1):
                        lines.extend([pts[i][:], pts[i + 1][:]])
            # Connecting lines at 4 cardinal points
            for angle in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
                dx = r * math.cos(angle)
                dz = r * math.sin(angle)
                p1 = top + mu.Vector((dx, 0, dz))
                p2 = bot + mu.Vector((dx, 0, dz))
                lines.extend([p1[:], p2[:]])

        elif props.col_type == 'aabb':
            hx, hy, hz = props.half_x, props.half_y, props.half_z
            corners = []
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corners.append(
                            (center + mu.Vector((sx * hx, sy * hy, sz * hz)))[:])
            edges = [
                (0, 1), (2, 3), (4, 5), (6, 7),
                (0, 2), (1, 3), (4, 6), (5, 7),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            for a, b in edges:
                lines.extend([corners[a], corners[b]])

    if not lines:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'LINES', {"pos": lines})
    shader.bind()
    shader.uniform_float("color", (0.0, 1.0, 0.0, 0.8))
    batch.draw(shader)


class NEA_OT_ToggleCollisionOverlay(bpy.types.Operator):
    """Toggle bone collision wireframe overlay in the 3D viewport"""
    bl_idname = "nea.toggle_collision_overlay"
    bl_label = "Toggle Collision Overlay"

    def execute(self, context):
        global _collision_draw_handler
        scene = context.scene
        scene.nea_show_collision = not scene.nea_show_collision

        if scene.nea_show_collision and _collision_draw_handler is None:
            _collision_draw_handler = bpy.types.SpaceView3D.draw_handler_add(
                _draw_collision_overlays, (), 'WINDOW', 'POST_VIEW')
        elif not scene.nea_show_collision and _collision_draw_handler is not None:
            bpy.types.SpaceView3D.draw_handler_remove(
                _collision_draw_handler, 'WINDOW')
            _collision_draw_handler = None

        # Redraw all 3D viewports
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

        return {'FINISHED'}


class NEA_OT_AutoFitCollision(bpy.types.Operator):
    """Auto-generate capsule collision shapes for all bones based on geometry"""
    bl_idname = "nea.autofit_collision"
    bl_label = "Auto-Fit All Bones"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.active_object is not None
                and context.active_object.type == 'ARMATURE')

    def execute(self, context):
        arm = context.active_object.data
        count = 0
        for bone in arm.bones:
            if bone.length < 0.001:
                continue
            props = bone.nea_collision
            env_radius = getattr(bone, 'head_radius', 0.0)
            if env_radius > 0.01:
                radius = env_radius
            else:
                radius = bone.length * 0.2
            props.col_type = 'capsule'
            props.radius = radius
            props.half_height = bone.length * 0.5
            props.offset_x = 0.0
            props.offset_y = bone.length * 0.5
            props.offset_z = 0.0
            count += 1
        self.report({'INFO'}, f"Auto-fitted {count} bone collision shapes")
        return {'FINISHED'}


###
### Scene management system
###

class NEA_SceneNodeProps(bpy.types.PropertyGroup):
    """Per-object scene node properties for NEA scene export."""
    node_type: EnumProperty(
        items=[
            ('auto', 'Auto', 'Detect type automatically (mesh/camera/empty)'),
            ('empty', 'Empty', 'Group/transform node'),
            ('mesh', 'Mesh', 'Renderable mesh node'),
            ('camera', 'Camera', 'Camera node'),
            ('trigger', 'Trigger', 'Collision zone with callbacks'),
        ],
        name="Node Type",
        description="Scene node type for NEA export",
        default='auto',
    )
    tags: StringProperty(
        name="Tags",
        description="Comma-separated tags (max 3, 15 chars each)",
        default="",
    )
    is_active_camera: BoolProperty(
        name="Active Camera",
        description="Set this camera as the active scene camera",
        default=False,
    )
    visible: BoolProperty(
        name="Visible",
        description="Node and its children are visible on load",
        default=True,
    )
    script_id: IntProperty(
        name="Script ID",
        description="Identifier for trigger scripts (0-255)",
        min=0, max=255, default=0,
    )
    trigger_shape: EnumProperty(
        items=[
            ('sphere', 'Sphere', 'Spherical trigger zone'),
            ('aabb', 'AABB', 'Axis-aligned bounding box trigger zone'),
        ],
        name="Trigger Shape",
        description="Collision shape for trigger zone",
        default='sphere',
    )
    trigger_radius: FloatProperty(
        name="Radius",
        description="Trigger sphere radius",
        default=1.0, min=0.01,
    )
    trigger_half_x: FloatProperty(
        name="Half X",
        description="AABB half-extent on X axis",
        default=1.0, min=0.01,
    )
    trigger_half_y: FloatProperty(
        name="Half Y",
        description="AABB half-extent on Y axis",
        default=1.0, min=0.01,
    )
    trigger_half_z: FloatProperty(
        name="Half Z",
        description="AABB half-extent on Z axis",
        default=1.0, min=0.01,
    )


class NEA_AddonPreferences(bpy.types.AddonPreferences):
    """Addon preferences for NEA tools (Edit > Preferences > Add-ons)."""
    bl_idname = __name__

    tools_path: StringProperty(
        name="NEA Repository Path",
        description=(
            "Path to the nitro-engine-advanced git repo root. "
            "Used to locate obj2dl, md5_to_dsma, etc. "
            "Leave empty to auto-detect from BlocksDS install"
        ),
        default="",
        subtype='DIR_PATH',
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "tools_path")
        if self.tools_path:
            tools_dir = os.path.join(bpy.path.abspath(self.tools_path), 'tools')
            if os.path.isdir(tools_dir):
                layout.label(text="Tools directory found", icon='CHECKMARK')
            else:
                layout.label(text="'tools/' not found at this path",
                             icon='ERROR')


class NEA_SceneSettings(bpy.types.PropertyGroup):
    """Per-scene settings for NEA scene export."""
    scene_name: StringProperty(
        name="Scene Name",
        description="Default file name for export (without extension)",
        default="level",
    )
    scale: FloatProperty(
        name="Scale",
        description="Scale factor for position values",
        default=1.0, min=0.01, max=1000.0,
    )


# Trigger zone viewport overlay draw handler
_trigger_draw_handler = None


def _draw_trigger_overlays():
    """GPU draw callback for trigger zone wireframe visualization."""
    import gpu
    from gpu_extras.batch import batch_for_shader

    context = bpy.context
    scene = context.scene
    if not getattr(scene, 'nea_show_triggers', False):
        return

    lines = []

    for obj in scene.objects:
        props = obj.nea_scene_node
        resolved = props.node_type
        if resolved == 'auto' and obj.type == 'EMPTY':
            resolved = 'empty'
        if resolved != 'trigger':
            continue

        center = obj.matrix_world.translation

        if props.trigger_shape == 'sphere':
            for axis in ('X', 'Y', 'Z'):
                pts = _make_circle_points(center, props.trigger_radius, axis)
                for i in range(len(pts) - 1):
                    lines.extend([pts[i][:], pts[i + 1][:]])

        elif props.trigger_shape == 'aabb':
            hx = props.trigger_half_x
            hy = props.trigger_half_y
            hz = props.trigger_half_z
            corners = []
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corners.append(
                            (center + mu.Vector((sx * hx, sy * hy, sz * hz)))[:])
            edges = [
                (0, 1), (2, 3), (4, 5), (6, 7),
                (0, 2), (1, 3), (4, 6), (5, 7),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            for a, b in edges:
                lines.extend([corners[a], corners[b]])

    if not lines:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'LINES', {"pos": lines})
    shader.bind()
    shader.uniform_float("color", (0.2, 1.0, 0.2, 0.8))
    batch.draw(shader)


class NEA_OT_ToggleTriggerOverlay(bpy.types.Operator):
    """Toggle trigger zone wireframe overlay in the 3D viewport"""
    bl_idname = "nea.toggle_trigger_overlay"
    bl_label = "Toggle Trigger Overlay"

    def execute(self, context):
        global _trigger_draw_handler
        scene = context.scene
        scene.nea_show_triggers = not scene.nea_show_triggers

        if scene.nea_show_triggers and _trigger_draw_handler is None:
            _trigger_draw_handler = bpy.types.SpaceView3D.draw_handler_add(
                _draw_trigger_overlays, (), 'WINDOW', 'POST_VIEW')
        elif not scene.nea_show_triggers and _trigger_draw_handler is not None:
            bpy.types.SpaceView3D.draw_handler_remove(
                _trigger_draw_handler, 'WINDOW')
            _trigger_draw_handler = None

        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

        return {'FINISHED'}


def _resolve_node_type(obj):
    """Determine the NEA node type for a Blender object."""
    ntype = obj.nea_scene_node.node_type
    if ntype != 'auto':
        return ntype
    if obj.type == 'CAMERA':
        return 'camera'
    if obj.type == 'MESH':
        return 'mesh'
    return 'empty'


def _collect_scene_nodes(context, scale):
    """Walk Blender's view layer objects and build the scene JSON dict."""
    import json

    # Gather all root-level and parented objects (skip armatures, lights, etc.
    # that don't map to scene nodes unless they are cameras)
    allowed_types = {'MESH', 'EMPTY', 'CAMERA'}
    objects = [o for o in context.view_layer.objects
               if o.type in allowed_types]

    # Build index map: object -> index
    obj_list = list(objects)
    obj_index = {o: i for i, o in enumerate(obj_list)}

    nodes = []
    assets = []
    materials = []
    active_camera_idx = 0xFFFF

    for i, obj in enumerate(obj_list):
        props = obj.nea_scene_node
        ntype = _resolve_node_type(obj)

        # Parent index
        parent_idx = 0xFF
        if obj.parent and obj.parent in obj_index:
            parent_idx = obj_index[obj.parent]

        # Position (in Blender coords, Y-up -> NDS: x, z, -y swap not needed
        # since the engine expects Blender-exported values)
        loc = obj.location * scale
        pos = [loc.x, loc.z, -loc.y]

        # Rotation (convert Blender euler to 0-511 range for NDS)
        euler = obj.rotation_euler
        rx = int(round(euler.x / (2 * math.pi) * 512)) & 0x1FF
        ry = int(round(euler.z / (2 * math.pi) * 512)) & 0x1FF
        rz = int(round(-euler.y / (2 * math.pi) * 512)) & 0x1FF

        # Scale
        scl = obj.scale
        scl_out = [scl.x, scl.z, scl.y]

        # Tags
        tag_list = []
        if props.tags.strip():
            tag_list = [t.strip()[:15] for t in props.tags.split(',')
                        if t.strip()][:3]

        node = {
            'name': obj.name[:23],
            'type': ntype,
            'parent_idx': parent_idx,
            'visible': props.visible,
            'position': pos,
            'rotation': [rx, ry, rz],
            'scale': scl_out,
            'tags': tag_list,
        }

        # Type-specific data
        if ntype == 'mesh':
            node['mesh'] = {
                'asset_index': 0,
                'material_index': 0xFFFF,
                'is_animated': False,
            }
        elif ntype == 'camera':
            # Compute look-at direction from camera's forward vector
            forward = obj.matrix_world.to_3x3() @ mu.Vector((0, 0, -1))
            up_vec = obj.matrix_world.to_3x3() @ mu.Vector((0, 1, 0))
            to = [
                (loc.x + forward.x) * scale,
                (loc.z + forward.z) * scale,
                -(loc.y + forward.y) * scale,
            ]
            up = [up_vec.x, up_vec.z, -up_vec.y]
            node['camera'] = {'to': to, 'up': up}

            if props.is_active_camera:
                active_camera_idx = i
        elif ntype == 'trigger':
            trig = {
                'shape': props.trigger_shape,
                'script_id': props.script_id,
            }
            if props.trigger_shape == 'sphere':
                trig['radius'] = props.trigger_radius * scale
            elif props.trigger_shape == 'aabb':
                trig['half_x'] = props.trigger_half_x * scale
                trig['half_y'] = props.trigger_half_y * scale
                trig['half_z'] = props.trigger_half_z * scale
            node['trigger'] = trig

        nodes.append(node)

    return {
        'nodes': nodes,
        'assets': assets,
        'materials': materials,
        'active_camera_idx': active_camera_idx,
    }


def _float_to_f32(val):
    """Convert a float to NDS 20.12 fixed-point (signed int32).

    Uses the same pattern as display_list.py float_to_v16.
    """
    res = int(val * (1 << 12))
    if res < -0x80000000:
        raise OverflowError(f"{val} too small for f32: {res:#010x}")
    if res > 0x7FFFFFFF:
        raise OverflowError(f"{val} too big for f32: {res:#010x}")
    if res < 0:
        res = 0x100000000 + res
    return res


def _pack_fixed_string(s, length):
    """Pack a string into a fixed-length null-padded bytes object."""
    encoded = s.encode('utf-8')[:length - 1]
    return encoded + b'\x00' * (length - len(encoded))


def _write_neascene_binary(output_path, nodes, assets, mat_refs,
                           active_camera_idx):
    """Write a binary .neascene file directly (no external tool needed).

    Binary layout matches NEAScene.c parser expectations:
        Header 16 B, Assets 64 B each, MatRefs 80 B each, Nodes 128 B each.
    """
    import struct

    NSCN_MAGIC = 0x4E53434E
    NSCN_VERSION = 1
    NODE_SIZE = 128
    NODE_NAME_LEN = 24
    TAG_LEN = 16
    MAX_TAGS = 6

    type_map = {'empty': 0, 'mesh': 1, 'camera': 2, 'trigger': 3}

    with open(output_path, 'wb') as f:
        # --- Header (16 bytes) ---
        f.write(struct.pack('<IIHHHH',
                            NSCN_MAGIC, NSCN_VERSION,
                            len(nodes), len(assets), len(mat_refs),
                            active_camera_idx))

        # --- Asset table (64 bytes each) ---
        for asset in assets:
            data = _pack_fixed_string(asset.get('path', ''), 48)
            data += struct.pack('<B', asset.get('type', 0))
            data += b'\x00' * 15
            f.write(data)

        # --- Material ref table (80 bytes each) ---
        for mat in mat_refs:
            f.write(_pack_fixed_string(mat.get('name', ''), 32))
            f.write(_pack_fixed_string(mat.get('tex_path', ''), 48))

        # --- Node table (128 bytes each) ---
        for node in nodes:
            buf = bytearray(NODE_SIZE)

            # Name (24 bytes at offset 0)
            buf[0:NODE_NAME_LEN] = _pack_fixed_string(
                node.get('name', ''), NODE_NAME_LEN)

            # Type, parent, num_tags, flags (offset 24-27)
            ntype = type_map.get(node.get('type', 'empty'), 0)
            buf[24] = ntype
            buf[25] = node.get('parent_idx', 0xFF) & 0xFF

            tags = node.get('tags', [])[:MAX_TAGS]
            buf[26] = len(tags)

            flags = 1 if node.get('visible', True) else 0
            buf[27] = flags

            # Position (f32 x3 at offset 28)
            pos = node.get('position', [0.0, 0.0, 0.0])
            struct.pack_into('<III', buf, 28,
                             _float_to_f32(pos[0]),
                             _float_to_f32(pos[1]),
                             _float_to_f32(pos[2]))

            # Rotation (int16 x3 at offset 40, +2 padding)
            rot = node.get('rotation', [0, 0, 0])
            struct.pack_into('<hhhh', buf, 40,
                             rot[0] & 0x1FF, rot[1] & 0x1FF,
                             rot[2] & 0x1FF, 0)

            # Scale (f32 x3 at offset 48)
            scl = node.get('scale', [1.0, 1.0, 1.0])
            struct.pack_into('<III', buf, 48,
                             _float_to_f32(scl[0]),
                             _float_to_f32(scl[1]),
                             _float_to_f32(scl[2]))

            # Type-specific data at offset 60
            type_str = node.get('type', 'empty')
            if type_str == 'mesh':
                mesh = node.get('mesh', {})
                struct.pack_into('<HHB', buf, 60,
                                 mesh.get('asset_index', 0),
                                 mesh.get('material_index', 0xFFFF),
                                 1 if mesh.get('is_animated', False) else 0)
            elif type_str == 'camera':
                cam = node.get('camera', {})
                to = cam.get('to', [0.0, 0.0, -1.0])
                up = cam.get('up', [0.0, 1.0, 0.0])
                struct.pack_into('<IIIIII', buf, 60,
                                 _float_to_f32(to[0]), _float_to_f32(to[1]),
                                 _float_to_f32(to[2]),
                                 _float_to_f32(up[0]), _float_to_f32(up[1]),
                                 _float_to_f32(up[2]))
            elif type_str == 'trigger':
                trig = node.get('trigger', {})
                shape_type = {'sphere': 1, 'aabb': 2}.get(
                    trig.get('shape', 'sphere'), 1)
                buf[60] = shape_type
                buf[61] = trig.get('script_id', 0)
                if shape_type == 1:
                    struct.pack_into('<I', buf, 64,
                                     _float_to_f32(trig.get('radius', 1.0)))
                elif shape_type == 2:
                    struct.pack_into('<III', buf, 64,
                                     _float_to_f32(trig.get('half_x', 1.0)),
                                     _float_to_f32(trig.get('half_y', 1.0)),
                                     _float_to_f32(trig.get('half_z', 1.0)))

            # Tags at offset 80 (3 * 16 = 48 bytes)
            for t in range(len(tags)):
                tag_bytes = _pack_fixed_string(tags[t], TAG_LEN)
                buf[80 + t * TAG_LEN:80 + (t + 1) * TAG_LEN] = tag_bytes

            f.write(bytes(buf))


class NEA_OT_ExportScene(bpy.types.Operator, ExportHelper):
    """Export the current Blender scene as a binary .neascene file"""
    bl_idname = "nea.export_scene"
    bl_label = "Export NEA Scene"
    bl_options = {'REGISTER', 'PRESET'}
    filename_ext = ".neascene"

    filter_glob: StringProperty(
        default="*.neascene",
        options={'HIDDEN'},
    )

    scaleFactor: FloatProperty(
        name="Scale",
        description="Scale factor for position values",
        default=1.0, min=0.01, max=1000.0,
    )

    export_json: BoolProperty(
        name="Also save .json",
        description="Keep the intermediate JSON file next to the binary",
        default=False,
    )

    path_mode = path_reference_mode
    check_extension = True

    def invoke(self, context, event):
        # Default filename from scene settings or blend file name
        settings = context.scene.nea_scene_settings
        if settings.scene_name:
            self.filepath = settings.scene_name
        else:
            self.filepath = "level"
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        import json
        import struct

        scale = self.scaleFactor

        # Build scene data
        scene_data = _collect_scene_nodes(context, scale)

        if not scene_data['nodes']:
            self.report({'ERROR'}, "No exportable objects in scene")
            return {'CANCELLED'}

        bin_path = self.filepath
        nodes = scene_data['nodes']
        assets = scene_data.get('assets', [])
        mat_refs = scene_data.get('materials', [])
        active_cam = scene_data.get('active_camera_idx', 0xFFFF)

        # Save optional JSON
        if self.export_json:
            export_dir = os.path.dirname(bin_path)
            base_name = os.path.splitext(os.path.basename(bin_path))[0]
            json_path = os.path.join(export_dir, base_name + '.neascene.json')
            with open(json_path, 'w') as f:
                json.dump(scene_data, f, indent=2)

        # Write binary .neascene directly
        _write_neascene_binary(bin_path, nodes, assets, mat_refs, active_cam)

        self.report({'INFO'},
                    f"Exported {len(nodes)} nodes to {bin_path}")
        return {'FINISHED'}


class OBJECT_PT_nea_scene_node(bpy.types.Panel):
    """NEA scene node properties"""
    bl_label = "NEA Scene Node"
    bl_idname = "OBJECT_PT_nea_scene_node"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    @classmethod
    def poll(cls, context):
        return context.object is not None

    def draw(self, context):
        layout = self.layout
        obj = context.object
        props = obj.nea_scene_node

        layout.prop(props, "node_type")
        layout.prop(props, "visible")
        layout.prop(props, "tags")

        resolved = _resolve_node_type(obj)

        if resolved == 'camera':
            layout.prop(props, "is_active_camera")

        if resolved == 'trigger':
            box = layout.box()
            box.label(text="Trigger Settings:")
            box.prop(props, "trigger_shape")
            box.prop(props, "script_id")
            if props.trigger_shape == 'sphere':
                box.prop(props, "trigger_radius")
            elif props.trigger_shape == 'aabb':
                col = box.column(align=True)
                col.prop(props, "trigger_half_x")
                col.prop(props, "trigger_half_y")
                col.prop(props, "trigger_half_z")


class VIEW3D_PT_nea_scene_export(bpy.types.Panel):
    """NEA scene export panel in the 3D viewport N-panel"""
    bl_label = "NEA Scene Export"
    bl_idname = "VIEW3D_PT_nea_scene_export"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "NEA"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.nea_scene_settings

        layout.prop(settings, "scene_name")
        layout.prop(settings, "scale")

        layout.separator()
        layout.operator("nea.export_scene", icon='EXPORT',
                         text="Export NEA Scene...")

        layout.separator()
        icon = 'HIDE_OFF' if context.scene.nea_show_triggers else 'HIDE_ON'
        layout.operator("nea.toggle_trigger_overlay", icon=icon,
                         depress=getattr(context.scene, 'nea_show_triggers',
                                         False))


###
### Texture VRAM size calculation
###

def _calc_texture_vram(tex_format, width, height):
    """Calculate NDS texture VRAM usage in bytes for a given format and size.
    Returns (tex_bytes, pal_bytes) tuple."""
    pixels = width * height
    tex_bytes = 0
    pal_bytes = 0

    if tex_format == 'tex4x4':
        # 4x4 block compression: 4 bytes per 4x4 block + 2-byte palette index
        block_count = (width // 4) * (height // 4)
        tex_bytes = block_count * 4 + block_count * 2
        pal_bytes = 4 * 2  # palette (4 colors * 2 bytes each, per block)
    elif tex_format == 'palette4':
        tex_bytes = pixels // 4       # 2 bits per pixel
        pal_bytes = 4 * 2             # 4 colors * 16-bit RGB
    elif tex_format == 'palette16':
        tex_bytes = pixels // 2       # 4 bits per pixel
        pal_bytes = 16 * 2            # 16 colors * 16-bit RGB
    elif tex_format == 'palette256':
        tex_bytes = pixels            # 8 bits per pixel
        pal_bytes = 256 * 2           # 256 colors * 16-bit RGB
    elif tex_format == 'a3i5':
        tex_bytes = pixels            # 8 bits per pixel
        pal_bytes = 32 * 2            # up to 32 palette entries
    elif tex_format == 'a5i3':
        tex_bytes = pixels            # 8 bits per pixel
        pal_bytes = 8 * 2             # up to 8 palette entries
    elif tex_format == 'direct':
        tex_bytes = pixels * 2        # 16 bits per pixel (RGB555)
        pal_bytes = 0
    return (tex_bytes, pal_bytes)


def _format_vram_size(size_bytes):
    """Format byte size as a readable string."""
    if size_bytes >= 1024:
        return f"{size_bytes / 1024:.1f} KB"
    return f"{size_bytes} B"


def _calc_scene_total_vram(context):
    """Calculate total texture VRAM for all materials in the scene."""
    total_tex = 0
    total_pal = 0
    seen = set()
    for obj in context.scene.objects:
        if obj.type != 'MESH':
            continue
        for mat in obj.data.materials:
            if mat is None or mat.name in seen:
                continue
            seen.add(mat.name)
            ptex = mat.nea_ptexconv
            # Find texture dimensions
            w, h = 0, 0
            if mat.node_tree:
                for node in mat.node_tree.nodes:
                    if node.type == 'TEX_IMAGE' and node.image:
                        w, h = node.image.size[0], node.image.size[1]
                        break
            if w > 0 and h > 0:
                tb, pb = _calc_texture_vram(ptex.tex_format, w, h)
                total_tex += tb
                total_pal += pb
    return (total_tex, total_pal)


###
### Polyformat settings (per-object polygon attributes)
###

def _float_to_rgb15(color):
    """Convert Blender RGB float (0-1) to NDS RGB15 integer."""
    r = int(color[0] * 31) & 31
    g = int(color[1] * 31) & 31
    b = int(color[2] * 31) & 31
    return r | (g << 5) | (b << 10)


class NEA_PolyformatProps(bpy.types.PropertyGroup):
    """Per-object polygon format settings for NEA."""
    light_enum: EnumProperty(
        items=[
            ('NEA_LIGHT_0', 'Light 0', ''),
            ('NEA_LIGHT_1', 'Light 1', ''),
            ('NEA_LIGHT_2', 'Light 2', ''),
            ('NEA_LIGHT_3', 'Light 3', ''),
            ('NEA_LIGHT_01', 'Light 0+1', ''),
            ('NEA_LIGHT_02', 'Light 0+2', ''),
            ('NEA_LIGHT_03', 'Light 0+3', ''),
            ('NEA_LIGHT_12', 'Light 1+2', ''),
            ('NEA_LIGHT_13', 'Light 1+3', ''),
            ('NEA_LIGHT_23', 'Light 2+3', ''),
            ('NEA_LIGHT_012', 'Light 0+1+2', ''),
            ('NEA_LIGHT_013', 'Light 0+1+3', ''),
            ('NEA_LIGHT_023', 'Light 0+2+3', ''),
            ('NEA_LIGHT_123', 'Light 1+2+3', ''),
            ('NEA_LIGHT_0123', 'All Lights', ''),
        ],
        name="Lights",
        description="Which lights affect this object",
        default='NEA_LIGHT_0',
    )
    alpha: IntProperty(
        name="Alpha",
        description="0 = wireframe, 31 = opaque, 1-30 = translucent",
        default=31, min=0, max=31,
    )
    polygon_id: IntProperty(
        name="Polygon ID",
        description="Used for antialias, blending, and outlining (0-63)",
        default=0, min=0, max=63,
    )
    culling: EnumProperty(
        items=[
            ('NEA_CULL_BACK', 'Back', 'Cull back-facing polygons'),
            ('NEA_CULL_FRONT', 'Front', 'Cull front-facing polygons'),
            ('NEA_CULL_NONE', 'None', 'Draw all polygons'),
        ],
        name="Culling",
        description="Polygon culling mode",
        default='NEA_CULL_BACK',
    )
    other_flag1_enabled: BoolProperty(name="Flag 1", default=False)
    other_flag1: EnumProperty(
        items=[
            ('NEA_MODULATION', 'Modulation', 'Normal shading'),
            ('NEA_DECAL', 'Decal', 'Decal mode'),
            ('NEA_TOON_HIGHLIGHT_SHADING', 'Toon/Highlight', 'Toon or highlight shading'),
            ('NEA_SHADOW_POLYGONS', 'Shadow', 'Shadow polygons'),
            ('NEA_TRANS_DEPTH_KEEP', 'Trans Depth Keep', 'Keep old depth for translucent'),
            ('NEA_TRANS_DEPTH_UPDATE', 'Trans Depth Update', 'Set new depth for translucent'),
            ('NEA_HIDE_FAR_CLIPPED', 'Hide Far Clipped', ''),
            ('NEA_RENDER_FAR_CLIPPED', 'Render Far Clipped', ''),
            ('NEA_FOG_ENABLE', 'Fog Enable', ''),
        ],
        name="", default='NEA_MODULATION',
    )
    other_flag2_enabled: BoolProperty(name="Flag 2", default=False)
    other_flag2: EnumProperty(
        items=[
            ('NEA_MODULATION', 'Modulation', ''),
            ('NEA_DECAL', 'Decal', ''),
            ('NEA_TOON_HIGHLIGHT_SHADING', 'Toon/Highlight', ''),
            ('NEA_SHADOW_POLYGONS', 'Shadow', ''),
            ('NEA_TRANS_DEPTH_KEEP', 'Trans Depth Keep', ''),
            ('NEA_TRANS_DEPTH_UPDATE', 'Trans Depth Update', ''),
            ('NEA_HIDE_FAR_CLIPPED', 'Hide Far Clipped', ''),
            ('NEA_RENDER_FAR_CLIPPED', 'Render Far Clipped', ''),
            ('NEA_FOG_ENABLE', 'Fog Enable', ''),
        ],
        name="", default='NEA_MODULATION',
    )


class OBJECT_PT_nea_polyformat(bpy.types.Panel):
    """NEA polygon format settings"""
    bl_label = "NEA Polyformat"
    bl_idname = "OBJECT_PT_nea_polyformat"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == 'MESH'

    def draw(self, context):
        layout = self.layout
        props = context.object.nea_polyformat

        layout.prop(props, "light_enum")

        box = layout.box()
        box.label(text="Polygon Format:")
        box.prop(props, "alpha")
        box.prop(props, "polygon_id")
        box.prop(props, "culling")

        box2 = layout.box()
        box2.label(text="Other Flags:")
        row = box2.row()
        row.prop(props, "other_flag1_enabled", text="")
        sub = row.row()
        sub.enabled = props.other_flag1_enabled
        sub.prop(props, "other_flag1")
        row = box2.row()
        row.prop(props, "other_flag2_enabled", text="")
        sub = row.row()
        sub.enabled = props.other_flag2_enabled
        sub.prop(props, "other_flag2")


###
### Material settings (ptexconv, texture flags, material properties)
###

class NEA_PtexconvProps(bpy.types.PropertyGroup):
    """Per-material ptexconv texture format settings."""
    tex_format: EnumProperty(
        items=[
            ('tex4x4', 'TEX4X4', 'Compressed 4x4 texels'),
            ('palette4', 'Palette 4', '4-color palette'),
            ('palette16', 'Palette 16', '16-color palette'),
            ('palette256', 'Palette 256', '256-color palette'),
            ('a3i5', 'A3I5', '3-bit alpha, 5-bit index'),
            ('a5i3', 'A5I3', '5-bit alpha, 3-bit index'),
            ('direct', 'Direct', 'Direct color (16-bit)'),
        ],
        name="Format",
        description="NDS texture format for ptexconv",
        default='palette256',
    )
    compression: IntProperty(
        name="TEX4X4 Compression",
        description="Compression level for TEX4X4 format (0-100)",
        default=0, min=0, max=100,
    )
    dithering: IntProperty(
        name="Dithering",
        description="Dithering level (0-100)",
        default=0, min=0, max=100,
    )


class NEA_TextureFlagsProps(bpy.types.PropertyGroup):
    """Per-material NDS texture flags."""
    wrap_s: BoolProperty(name="Wrap S", default=True,
                         description="NEA_TEXTURE_WRAP_S")
    wrap_t: BoolProperty(name="Wrap T", default=True,
                         description="NEA_TEXTURE_WRAP_T")
    flip_s: BoolProperty(name="Flip S", default=False,
                         description="NEA_TEXTURE_FLIP_S")
    flip_t: BoolProperty(name="Flip T", default=False,
                         description="NEA_TEXTURE_FLIP_T")
    color0_transparent: BoolProperty(name="Color 0 Transparent", default=False,
                                     description="NEA_TEXTURE_COLOR0_TRANSPARENT")
    texgen: EnumProperty(
        items=[
            ('NEA_TEXGEN_TEXCOORD', 'Texcoord', 'Use UV coordinates'),
            ('NEA_TEXGEN_NORMAL', 'Normal', 'Generate from normals'),
            ('NEA_TEXGEN_POSITION', 'Position', 'Generate from position'),
            ('NEA_TEXGEN_OFF', 'Off', 'No texture coordinate generation'),
        ],
        name="TexGen",
        description="Texture coordinate generation mode",
        default='NEA_TEXGEN_TEXCOORD',
    )


class NEA_MaterialProps(bpy.types.PropertyGroup):
    """Per-material NDS material color properties."""
    enabled: BoolProperty(
        name="Use Material Properties",
        description="Enable custom diffuse/ambient/specular/emission colors",
        default=False,
    )
    diffuse: FloatVectorProperty(
        name="Diffuse", subtype='COLOR',
        description="Light directly hitting the polygon",
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0,
    )
    ambient: FloatVectorProperty(
        name="Ambient", subtype='COLOR',
        description="Indirect light (reflections from environment)",
        default=(0.0, 0.0, 0.0), min=0.0, max=1.0,
    )
    specular: FloatVectorProperty(
        name="Specular", subtype='COLOR',
        description="Light reflected towards the camera (mirror)",
        default=(0.0, 0.0, 0.0), min=0.0, max=1.0,
    )
    emission: FloatVectorProperty(
        name="Emission", subtype='COLOR',
        description="Light emitted by the polygon",
        default=(0.0, 0.0, 0.0), min=0.0, max=1.0,
    )
    use_vtx_color: BoolProperty(
        name="Vertex Color",
        description="Diffuse reflection works as color command",
        default=False,
    )
    use_shininess: BoolProperty(
        name="Shininess Table",
        description="Specular uses the shininess table",
        default=False,
    )


class MATERIAL_PT_nea_ptexconv(bpy.types.Panel):
    """NEA ptexconv texture format settings"""
    bl_label = "NEA Texture Format (ptexconv)"
    bl_idname = "MATERIAL_PT_nea_ptexconv"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "material"

    @classmethod
    def poll(cls, context):
        return context.material is not None

    def draw(self, context):
        layout = self.layout
        mat = context.material
        ptex = mat.nea_ptexconv
        flags = mat.nea_tex_flags
        props = mat.nea_mat_props

        box = layout.box()
        box.label(text="Texture Format:")
        box.prop(ptex, "tex_format")
        if ptex.tex_format == 'tex4x4':
            box.prop(ptex, "compression")
        box.prop(ptex, "dithering")

        # Per-material VRAM size display
        w, h = 0, 0
        if mat.node_tree:
            for node in mat.node_tree.nodes:
                if node.type == 'TEX_IMAGE' and node.image:
                    w, h = node.image.size[0], node.image.size[1]
                    break
        if w > 0 and h > 0:
            tb, pb = _calc_texture_vram(ptex.tex_format, w, h)
            vram_box = box.box()
            vram_box.label(text=f"Texture: {w}x{h}")
            vram_box.label(text=f"VRAM: {_format_vram_size(tb)} tex"
                           f" + {_format_vram_size(pb)} pal"
                           f" = {_format_vram_size(tb + pb)}")

        box2 = layout.box()
        box2.label(text="Texture Flags:")
        row = box2.row()
        row.prop(flags, "wrap_s")
        row.prop(flags, "wrap_t")
        row = box2.row()
        row.prop(flags, "flip_s")
        row.prop(flags, "flip_t")
        box2.prop(flags, "color0_transparent")
        box2.prop(flags, "texgen")

        box3 = layout.box()
        box3.prop(props, "enabled")
        if props.enabled:
            box3.prop(props, "diffuse")
            box3.label(text=f"  RGB15: {_float_to_rgb15(props.diffuse):#06x}")
            box3.prop(props, "ambient")
            box3.label(text=f"  RGB15: {_float_to_rgb15(props.ambient):#06x}")
            box3.prop(props, "specular")
            box3.label(text=f"  RGB15: {_float_to_rgb15(props.specular):#06x}")
            box3.prop(props, "emission")
            box3.label(text=f"  RGB15: {_float_to_rgb15(props.emission):#06x}")
            box3.prop(props, "use_vtx_color")
            box3.prop(props, "use_shininess")


###
### Light settings (per-scene, 4 lights)
###

class NEA_LightProps(bpy.types.PropertyGroup):
    """Per-scene light configuration (4 NDS hardware lights)."""
    light0_enabled: BoolProperty(name="Enable Light 0", default=True)
    light0_color: FloatVectorProperty(
        name="Color", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0)
    light0_x: FloatProperty(name="X", default=-0.5, min=-0.98, max=0.98,
                            description="Direction vector X (v10 format)")
    light0_y: FloatProperty(name="Y", default=-0.5, min=-0.98, max=0.98)
    light0_z: FloatProperty(name="Z", default=-0.5, min=-0.98, max=0.98)

    light1_enabled: BoolProperty(name="Enable Light 1", default=False)
    light1_color: FloatVectorProperty(
        name="Color", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0)
    light1_x: FloatProperty(name="X", default=-0.5, min=-0.98, max=0.98)
    light1_y: FloatProperty(name="Y", default=-0.5, min=-0.98, max=0.98)
    light1_z: FloatProperty(name="Z", default=-0.5, min=-0.98, max=0.98)

    light2_enabled: BoolProperty(name="Enable Light 2", default=False)
    light2_color: FloatVectorProperty(
        name="Color", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0)
    light2_x: FloatProperty(name="X", default=-0.5, min=-0.98, max=0.98)
    light2_y: FloatProperty(name="Y", default=-0.5, min=-0.98, max=0.98)
    light2_z: FloatProperty(name="Z", default=-0.5, min=-0.98, max=0.98)

    light3_enabled: BoolProperty(name="Enable Light 3", default=False)
    light3_color: FloatVectorProperty(
        name="Color", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0)
    light3_x: FloatProperty(name="X", default=-0.5, min=-0.98, max=0.98)
    light3_y: FloatProperty(name="Y", default=-0.5, min=-0.98, max=0.98)
    light3_z: FloatProperty(name="Z", default=-0.5, min=-0.98, max=0.98)

    near_plane: FloatProperty(
        name="Near Plane", default=0.1, min=0.001,
        description="Near clipping plane distance")
    far_plane: FloatProperty(
        name="Far Plane", default=16.0, min=0.1,
        description="Far clipping plane distance")


class SCENE_PT_nea_lights(bpy.types.Panel):
    """NEA per-scene light and clipping settings"""
    bl_label = "NEA Lights & Clipping"
    bl_idname = "SCENE_PT_nea_lights"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        lp = context.scene.nea_light_props

        box = layout.box()
        box.label(text="Clipping Planes:")
        row = box.row()
        row.prop(lp, "near_plane")
        row.prop(lp, "far_plane")

        for i in range(4):
            box = layout.box()
            en = getattr(lp, f"light{i}_enabled")
            box.prop(lp, f"light{i}_enabled")
            if en:
                box.prop(lp, f"light{i}_color")
                c = getattr(lp, f"light{i}_color")
                box.label(text=f"  RGB15: {_float_to_rgb15(c):#06x}")
                row = box.row(align=True)
                row.prop(lp, f"light{i}_x")
                row.prop(lp, f"light{i}_y")
                row.prop(lp, f"light{i}_z")


class SCENE_PT_nea_vram(bpy.types.Panel):
    """Total texture VRAM usage for the scene"""
    bl_label = "NEA Texture VRAM"
    bl_idname = "SCENE_PT_nea_vram"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        total_tex, total_pal = _calc_scene_total_vram(context)
        total = total_tex + total_pal

        box = layout.box()
        box.label(text="Scene Texture VRAM Total:", icon='TEXTURE')
        box.label(text=f"Texture data: {_format_vram_size(total_tex)}")
        box.label(text=f"Palette data: {_format_vram_size(total_pal)}")
        box.separator()
        row = box.row()
        row.alert = total > 256 * 1024  # NDS has 256 KB texture VRAM per bank
        row.label(text=f"Total: {_format_vram_size(total)}")

        # NDS VRAM reference
        box2 = layout.box()
        box2.label(text="NDS VRAM Banks:")
        box2.label(text=f"  VRAM_A: 128 KB ({total * 100 / (128*1024):.0f}%%)" if total > 0 else "  VRAM_A: 128 KB")
        box2.label(text=f"  VRAM_AB: 256 KB ({total * 100 / (256*1024):.0f}%%)" if total > 0 else "  VRAM_AB: 256 KB")
        box2.label(text=f"  VRAM_ABCD: 512 KB ({total * 100 / (512*1024):.0f}%%)" if total > 0 else "  VRAM_ABCD: 512 KB")


###
### Mesh conversion tools (obj2dl, md5_to_dsma, ptexconv execution)
###

def _find_nea_tool(name):
    """Find a NEA tool by name, checking addon prefs, env var, then installed."""
    tool_file = name + '.py'

    # 1. Addon preferences (user-configured repo path)
    prefs = bpy.context.preferences.addons.get(__name__)
    if prefs and prefs.preferences.tools_path:
        path = os.path.join(
            bpy.path.abspath(prefs.preferences.tools_path),
            'tools', name, tool_file)
        if os.path.isfile(path):
            return path

    # 2. NEA_ROOT environment variable
    env_root = os.environ.get('NEA_ROOT', '')
    if env_root:
        path = os.path.join(env_root, 'tools', name, tool_file)
        if os.path.isfile(path):
            return path

    # 3. Adjacent to addon file (works if addon is run from repo)
    addon_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(addon_dir)
    path = os.path.join(repo_root, 'tools', name, tool_file)
    if os.path.isfile(path):
        return path

    # 4. Installed NEA in BlocksDS
    path = os.path.join(
        '/opt/wonderful/thirdparty/blocksds/external',
        'nitro-engine-advanced', 'tools', name, tool_file)
    if os.path.isfile(path):
        return path

    # 5. Original nitro-engine in BlocksDS
    path = os.path.join(
        '/opt/wonderful/thirdparty/blocksds/external',
        'nitro-engine', 'tools', name, tool_file)
    if os.path.isfile(path):
        return path

    return None


def _run_tool(cmd, report_fn):
    """Run a subprocess command and report results. Returns True on success."""
    import subprocess
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except FileNotFoundError as e:
        report_fn({'ERROR'}, f"Command not found: {e}")
        return False
    except subprocess.TimeoutExpired:
        report_fn({'ERROR'}, "Tool timed out (120s)")
        return False

    if result.returncode != 0:
        err = result.stderr.strip() or result.stdout.strip()
        report_fn({'ERROR'}, f"Tool failed:\n{err}")
        return False

    if result.stdout.strip():
        print(result.stdout.strip())
    return True


def _resolve_output_dir(ts):
    """Resolve output directory from tool settings. Returns absolute path or None."""
    raw = ts.output_dir
    resolved = bpy.path.abspath(raw) if raw else ''
    if not resolved or not os.path.isabs(resolved):
        # '//' doesn't resolve when .blend file is not saved
        blend = bpy.data.filepath
        if blend:
            resolved = os.path.dirname(blend)
        else:
            resolved = ''
    if not resolved:
        return None
    return resolved


class NEA_ToolSettings(bpy.types.PropertyGroup):
    """Per-scene settings for NEA conversion tools."""
    output_dir: StringProperty(
        name="Output Directory",
        description="Directory for output files (save .blend file first, or set absolute path)",
        default="//",
        subtype='DIR_PATH',
    )
    obj2dl_scale: FloatProperty(
        name="Scale", default=1.0, min=0.001, max=1000.0)
    obj2dl_vertex_color: BoolProperty(
        name="Vertex Colors", default=False)
    obj2dl_multi_material: BoolProperty(
        name="Multi-Material (DLMM)", default=False)
    obj2dl_collision: BoolProperty(
        name="Generate .colmesh", default=False)
    md5_mesh_path: StringProperty(
        name="MD5mesh File",
        description="Path to the .md5mesh file",
        subtype='FILE_PATH',
    )
    md5_model_name: StringProperty(
        name="Model Name", default="model")
    md5_blender_fix: BoolProperty(
        name="Blender Fix",
        description="Rotate model -90 degrees on X",
        default=True,
    )
    md5_export_base_pose: BoolProperty(
        name="Export Base Pose", default=False)
    md5_multi_material: BoolProperty(
        name="Multi-Material (DLMM)", default=False)


# =========================================================================
# Animated Material System (Action/F-Curve based)
# =========================================================================

class NEA_AnimMatProps(bpy.types.PropertyGroup):
    """Per-material NDS animated material properties.

    Keyframe these using Blender's animation tools (right-click > Insert
    Keyframe, or I key), then export. Only properties that have F-Curves
    in the material's Action are exported as tracks.
    """
    enabled: BoolProperty(
        name="Enable Animated Material",
        description="Enable animated material export for this material",
        default=False,
    )
    # -- Polygon format --
    nea_alpha: IntProperty(
        name="Alpha", min=0, max=31, default=31,
        description="NDS polygon alpha (0=transparent, 31=opaque)",
    )
    nea_lights: IntProperty(
        name="Lights Mask", min=0, max=15, default=1,
        description="NDS light enable bitmask (1=L0, 2=L1, 4=L2, 8=L3)",
    )
    nea_culling: IntProperty(
        name="Culling Mode", min=0, max=3, default=2,
        description="0=none, 1=front, 2=back, 3=both sides",
    )
    nea_polyid: IntProperty(
        name="Polygon ID", min=0, max=63, default=0,
        description="NDS polygon ID (0-63)",
    )
    # -- Colors --
    nea_color: FloatVectorProperty(
        name="Vertex Color", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0, size=3,
    )
    nea_diffuse: FloatVectorProperty(
        name="Diffuse", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0, size=3,
    )
    nea_ambient: FloatVectorProperty(
        name="Ambient", subtype='COLOR',
        default=(0.5, 0.5, 0.5), min=0.0, max=1.0, size=3,
    )
    nea_specular: FloatVectorProperty(
        name="Specular", subtype='COLOR',
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0, size=3,
    )
    nea_emission: FloatVectorProperty(
        name="Emission", subtype='COLOR',
        default=(0.0, 0.0, 0.0), min=0.0, max=1.0, size=3,
    )
    # -- Material swap --
    nea_mat_index: IntProperty(
        name="Material Index", min=0, max=255, default=0,
        description="Index into the material swap table",
    )
    # -- Texture matrix --
    nea_tex_scroll_x: FloatProperty(
        name="Tex Scroll X", default=0.0,
        description="Texture matrix scroll X (f32 units)",
    )
    nea_tex_scroll_y: FloatProperty(
        name="Tex Scroll Y", default=0.0,
        description="Texture matrix scroll Y (f32 units)",
    )
    nea_tex_rotate: IntProperty(
        name="Tex Rotate", min=0, max=511, default=0,
        description="Texture rotation angle (0-511 = full turn)",
    )
    nea_tex_scale_x: FloatProperty(
        name="Tex Scale X", default=1.0, min=0.0,
        description="Texture matrix scale X (1.0 = identity)",
    )
    nea_tex_scale_y: FloatProperty(
        name="Tex Scale Y", default=1.0, min=0.0,
        description="Texture matrix scale Y (1.0 = identity)",
    )


class MATERIAL_PT_nea_animmat(bpy.types.Panel):
    """NEA Animated Material panel (in Material Properties)."""
    bl_label = "NEA Animated Material"
    bl_idname = "MATERIAL_PT_nea_animmat"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "material"

    @classmethod
    def poll(cls, context):
        return (context.object is not None
                and context.object.type == 'MESH'
                and context.material is not None)

    def draw(self, context):
        layout = self.layout
        mat = context.material
        props = mat.nea_animmat

        layout.prop(props, "enabled")
        if not props.enabled:
            return

        # Show action info
        if mat.animation_data and mat.animation_data.action:
            action = mat.animation_data.action
            fs, fe = action.frame_range
            layout.label(text=f"Action: {action.name}  ({int(fe - fs + 1)} frames)",
                         icon='ACTION')
        else:
            layout.label(text="No Action assigned. Keyframe properties to create one.",
                         icon='INFO')

        # Polygon format
        box = layout.box()
        box.label(text="Polygon Format", icon='MESH_CUBE')
        row = box.row(align=True)
        row.prop(props, "nea_alpha")
        row.prop(props, "nea_polyid")
        row = box.row(align=True)
        row.prop(props, "nea_lights")
        row.prop(props, "nea_culling")

        # Colors
        box = layout.box()
        box.label(text="Colors", icon='COLOR')
        box.prop(props, "nea_color")
        row = box.row(align=True)
        row.prop(props, "nea_diffuse")
        row.prop(props, "nea_ambient")
        row = box.row(align=True)
        row.prop(props, "nea_specular")
        row.prop(props, "nea_emission")

        # Material swap
        box = layout.box()
        box.label(text="Material Swap", icon='MATERIAL')
        box.prop(props, "nea_mat_index")

        # Texture matrix
        box = layout.box()
        box.label(text="Texture Matrix", icon='TEXTURE')
        row = box.row(align=True)
        row.prop(props, "nea_tex_scroll_x")
        row.prop(props, "nea_tex_scroll_y")
        box.prop(props, "nea_tex_rotate")
        row = box.row(align=True)
        row.prop(props, "nea_tex_scale_x")
        row.prop(props, "nea_tex_scale_y")

        layout.separator()
        layout.label(text="Right-click a property > Insert Keyframe to animate.",
                     icon='INFO')
        layout.operator("nea.export_animmat", icon='EXPORT')


def _float_to_rgb15(r, g, b):
    """Convert float RGB (0-1) to NDS RGB15 packed value."""
    ri = int(round(r * 31)) & 0x1F
    gi = int(round(g * 31)) & 0x1F
    bi = int(round(b * 31)) & 0x1F
    return ri | (gi << 5) | (bi << 10)


def _float_to_f32(val):
    """Convert a float to NDS f32 (20.12 fixed-point) as uint32."""
    return int(round(val * 4096)) & 0xFFFFFFFF


class NEA_OT_ExportAnimMat(bpy.types.Operator):
    """Export animated material by reading F-Curves from material Action"""
    bl_idname = "nea.export_animmat"
    bl_label = "Export Animated Material"

    def execute(self, context):
        import struct
        from bpy_extras.anim_utils import action_ensure_channelbag_for_slot

        obj = context.active_object
        if obj is None or obj.type != 'MESH':
            self.report({'WARNING'}, "No active mesh object")
            return {'CANCELLED'}

        mat = obj.active_material
        if mat is None or not hasattr(mat, 'nea_animmat'):
            self.report({'WARNING'}, "No active material")
            return {'CANCELLED'}

        props = mat.nea_animmat
        if not props.enabled:
            self.report({'WARNING'}, "Animated material not enabled")
            return {'CANCELLED'}

        if not mat.animation_data or not mat.animation_data.action:
            self.report({'WARNING'},
                        "No Action on material. Keyframe properties first.")
            return {'CANCELLED'}

        action = mat.animation_data.action
        slot = mat.animation_data.action_slot
        channelbag = action_ensure_channelbag_for_slot(action, slot)

        # ---- Collect F-Curves by data_path ----
        fc_map = {}  # data_path -> {array_index: fcurve}
        for fc in channelbag.fcurves:
            dp = fc.data_path
            if dp.startswith("nea_animmat."):
                if dp not in fc_map:
                    fc_map[dp] = {}
                fc_map[dp][fc.array_index] = fc

        if not fc_map:
            self.report({'WARNING'}, "No animated NEA properties found")
            return {'CANCELLED'}

        # ---- Property mapping ----
        # Simple scalar tracks: one F-Curve -> one track
        SIMPLE_MAP = {
            'nea_animmat.nea_alpha':        (0, 'int', 0x1F),
            'nea_animmat.nea_lights':       (1, 'int', 0x0F),
            'nea_animmat.nea_culling':      (2, 'int', 0xC0),
            'nea_animmat.nea_mat_index':    (6, 'int', 0xFF),
            'nea_animmat.nea_polyid':       (7, 'int', 0x3F),
            'nea_animmat.nea_tex_scroll_x': (8, 'f32', None),
            'nea_animmat.nea_tex_scroll_y': (9, 'f32', None),
            'nea_animmat.nea_tex_rotate':   (10, 'int', 0x1FF),
            'nea_animmat.nea_tex_scale_x':  (11, 'f32', None),
            'nea_animmat.nea_tex_scale_y':  (12, 'f32', None),
        }
        # Force STEP for inherently discrete types
        FORCE_STEP = {1, 2, 6, 7}

        # ---- Build track list ----
        frame_start, frame_end = action.frame_range
        num_frames = int(frame_end - frame_start) + 1
        tracks = []  # list of (track_type_id, interp_mode, [(frame, value)])

        # Process simple scalar tracks
        for dp, (track_id, enc, mask) in SIMPLE_MAP.items():
            if dp not in fc_map:
                continue
            fc = fc_map[dp].get(0)
            if fc is None:
                continue

            # Determine interpolation from first keyframe
            interp = 0  # STEP
            if track_id not in FORCE_STEP and len(fc.keyframe_points) > 0:
                kf_type = fc.keyframe_points[0].interpolation
                if kf_type in ('LINEAR', 'BEZIER'):
                    interp = 1  # LINEAR

            keyframes = []
            for kp in fc.keyframe_points:
                frame = int(round(kp.co[0])) - int(frame_start)
                if frame < 0:
                    frame = 0
                raw = kp.co[1]
                if enc == 'f32':
                    value = _float_to_f32(raw)
                else:
                    value = int(round(raw)) & mask
                keyframes.append((frame, value))

            keyframes.sort(key=lambda x: x[0])
            if keyframes:
                tracks.append((track_id, interp, keyframes))

        # Process color tracks (3 F-Curves -> single RGB15 track)
        COLOR_MAP = {
            'nea_animmat.nea_color': 3,  # COLOR
        }
        for dp, track_id in COLOR_MAP.items():
            if dp not in fc_map:
                continue
            fcs = fc_map[dp]
            if not fcs:
                continue

            # Collect all unique frames
            frames_set = set()
            for idx, fc in fcs.items():
                for kp in fc.keyframe_points:
                    frames_set.add(int(round(kp.co[0])))
            frames_sorted = sorted(frames_set)

            interp = 1  # LINEAR default for colors
            keyframes = []
            for frame in frames_sorted:
                r = fcs[0].evaluate(frame) if 0 in fcs else 1.0
                g = fcs[1].evaluate(frame) if 1 in fcs else 1.0
                b = fcs[2].evaluate(frame) if 2 in fcs else 1.0
                value = _float_to_rgb15(r, g, b)
                keyframes.append((int(frame - frame_start), value))

            if keyframes:
                tracks.append((track_id, interp, keyframes))

        # Process compound tracks (diffuse+ambient -> packed, spec+emi -> packed)
        COMPOUND = [
            ('nea_animmat.nea_diffuse', 'nea_animmat.nea_ambient', 4),
            ('nea_animmat.nea_specular', 'nea_animmat.nea_emission', 5),
        ]
        for dp_a, dp_b, track_id in COMPOUND:
            fcs_a = fc_map.get(dp_a, {})
            fcs_b = fc_map.get(dp_b, {})
            if not fcs_a and not fcs_b:
                continue

            frames_set = set()
            for fcs in (fcs_a, fcs_b):
                for idx, fc in fcs.items():
                    for kp in fc.keyframe_points:
                        frames_set.add(int(round(kp.co[0])))
            frames_sorted = sorted(frames_set)

            interp = 1  # LINEAR
            keyframes = []
            for frame in frames_sorted:
                # Color A (diffuse or specular) in lower 16 bits
                ra = fcs_a[0].evaluate(frame) if 0 in fcs_a else 1.0
                ga = fcs_a[1].evaluate(frame) if 1 in fcs_a else 1.0
                ba = fcs_a[2].evaluate(frame) if 2 in fcs_a else 1.0
                lo = _float_to_rgb15(ra, ga, ba)
                # Color B (ambient or emission) in upper 16 bits
                rb = fcs_b[0].evaluate(frame) if 0 in fcs_b else 0.5
                gb = fcs_b[1].evaluate(frame) if 1 in fcs_b else 0.5
                bb = fcs_b[2].evaluate(frame) if 2 in fcs_b else 0.5
                hi = _float_to_rgb15(rb, gb, bb)
                value = lo | (hi << 16)
                keyframes.append((int(frame - frame_start), value))

            if keyframes:
                tracks.append((track_id, interp, keyframes))

        if not tracks:
            self.report({'WARNING'}, "No animated tracks found in Action")
            return {'CANCELLED'}

        if len(tracks) > 16:
            self.report({'WARNING'}, "Too many tracks (max 16)")
            return {'CANCELLED'}

        # ---- Write binary .neaanimmat ----
        ts = context.scene.nea_tool_settings
        out_dir = _resolve_output_dir(ts)
        if out_dir is None:
            self.report({'WARNING'},
                        "No output directory. Save the .blend file first.")
            return {'CANCELLED'}

        filename = f"{mat.name}_animmat.neaanimmat"
        filepath = os.path.join(out_dir, filename)

        try:
            MAGIC = 0x464B4D41  # "AMKF"
            VERSION = 1
            header_size = 16
            track_header_size = 12
            num_tracks = len(tracks)
            kf_data_start = header_size + track_header_size * num_tracks

            with open(filepath, 'wb') as f:
                # File header (16 bytes)
                f.write(struct.pack('<IIHH4x',
                                    MAGIC, VERSION, num_tracks,
                                    num_frames))

                # Track headers
                offset = kf_data_start
                for track_id, interp, keyframes in tracks:
                    num_kf = len(keyframes)
                    f.write(struct.pack('<BBHI4x',
                                        track_id, interp,
                                        num_kf, offset))
                    offset += num_kf * 8

                # Keyframe data
                for track_id, interp, keyframes in tracks:
                    for frame, value in keyframes:
                        f.write(struct.pack('<HHI', frame, 0, value))

            total = os.path.getsize(filepath)
            self.report({'INFO'},
                        f"Exported {filepath} ({total} bytes)")
            print(f"[NEA] Exported animated material: {filepath} "
                  f"({num_tracks} tracks, {num_frames} frames, {total} bytes)")

        except Exception as e:
            self.report({'WARNING'}, f"Export failed: {e}")
            print(f"[NEA] AnimMat export error: {e}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

        return {'FINISHED'}


# =========================================================================
# AnimMat visual preview (frame change handler)
# =========================================================================

_animmat_handler = None


def _nea_animmat_frame_handler(scene, depsgraph):
    """Frame change handler for visual preview of AnimMat properties.

    Applies animated material effects to the Blender viewport:
      - Material swap: sets the object's active material slot index.
      - Texture scroll: updates a Mapping node's Location in the shader tree.
      - Alpha: updates the Principled BSDF Alpha input.
    """
    for obj in scene.objects:
        if obj.type != 'MESH':
            continue

        for slot_idx, slot in enumerate(obj.material_slots):
            mat = slot.material
            if mat is None or not hasattr(mat, 'nea_animmat'):
                continue

            props = mat.nea_animmat
            if not props.enabled:
                continue

            # --- Material swap preview ---
            # Check if nea_mat_index is animated on this material
            if mat.animation_data and mat.animation_data.action:
                has_swap = False
                for fc in mat.animation_data.action.fcurves:
                    if fc.data_path == 'nea_animmat.nea_mat_index':
                        has_swap = True
                        break
                if has_swap:
                    idx = int(props.nea_mat_index)
                    if 0 <= idx < len(obj.material_slots):
                        obj.active_material_index = idx

            if mat.node_tree is None:
                continue

            # --- Texture scroll preview ---
            # Update the first Mapping node if tex scroll is animated
            if mat.animation_data and mat.animation_data.action:
                has_scroll = False
                for fc in mat.animation_data.action.fcurves:
                    if fc.data_path in ('nea_animmat.nea_tex_scroll_x',
                                        'nea_animmat.nea_tex_scroll_y'):
                        has_scroll = True
                        break
                if has_scroll:
                    for node in mat.node_tree.nodes:
                        if node.type == 'MAPPING':
                            node.inputs['Location'].default_value[0] = (
                                props.nea_tex_scroll_x / 256.0)
                            node.inputs['Location'].default_value[1] = (
                                props.nea_tex_scroll_y / 256.0)
                            break

            # --- Alpha preview ---
            for node in mat.node_tree.nodes:
                if node.type == 'BSDF_PRINCIPLED':
                    # Check if alpha is animated
                    has_alpha = False
                    if mat.animation_data and mat.animation_data.action:
                        for fc in mat.animation_data.action.fcurves:
                            if fc.data_path == 'nea_animmat.nea_alpha':
                                has_alpha = True
                                break
                    if has_alpha:
                        node.inputs['Alpha'].default_value = (
                            props.nea_alpha / 31.0)
                    break


class NEA_OT_RunObj2dl(bpy.types.Operator):
    """Export active mesh as OBJ and convert to NDS display list with obj2dl"""
    bl_idname = "nea.run_obj2dl"
    bl_label = "Convert Mesh (obj2dl)"
    bl_options = {'REGISTER'}

    def execute(self, context):
        try:
            return self._run(context)
        except Exception as e:
            msg = f"obj2dl error: {e}"
            print(f"NEA ERROR: {msg}")
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}

    def _run(self, context):
        obj = context.active_object
        if obj is None or obj.type != 'MESH':
            self.report({'WARNING'}, "Select a mesh object first")
            return {'CANCELLED'}

        ts = context.scene.nea_tool_settings
        out_dir = _resolve_output_dir(ts)
        if not out_dir:
            self.report({'WARNING'},
                        "Set Output Directory or save .blend file first")
            return {'CANCELLED'}
        os.makedirs(out_dir, exist_ok=True)
        print(f"NEA: Output directory: {out_dir}")

        name = re.sub(r'[^A-Za-z0-9_]', '_', obj.name)
        obj_path = os.path.join(out_dir, name + ".obj")
        bin_path = os.path.join(out_dir, name + ".bin")

        # Find obj2dl first (fail early)
        obj2dl_path = _find_nea_tool('obj2dl')
        if not obj2dl_path:
            self.report({'WARNING'},
                        "obj2dl.py not found. Set repo path in "
                        "Edit > Preferences > Add-ons > NEA")
            return {'CANCELLED'}
        print(f"NEA: Using obj2dl at {obj2dl_path}")

        # Select only this object and export OBJ
        prev_selection = context.selected_objects[:]
        prev_active = context.view_layer.objects.active
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj
        try:
            bpy.ops.wm.obj_export(
                filepath=obj_path,
                export_selected_objects=True,
                export_uv=True,
                export_normals=True,
                export_materials=False,
                forward_axis='NEGATIVE_Z',
                up_axis='Y',
            )
        finally:
            bpy.ops.object.select_all(action='DESELECT')
            for o in prev_selection:
                o.select_set(True)
            context.view_layer.objects.active = prev_active

        if not os.path.isfile(obj_path):
            self.report({'WARNING'}, "OBJ export produced no file")
            return {'CANCELLED'}
        print(f"NEA: Exported OBJ to {obj_path}")

        # Get texture size from first material
        tex_w, tex_h = 0, 0
        if obj.data.materials:
            mat = obj.data.materials[0]
            if mat and mat.node_tree:
                for node in mat.node_tree.nodes:
                    if node.type == 'TEX_IMAGE' and node.image:
                        tex_w, tex_h = node.image.size[0], node.image.size[1]
                        break

        cmd = ['/usr/bin/python3', obj2dl_path,
               '--input', obj_path,
               '--output', bin_path]
        if tex_w > 0 and tex_h > 0:
            cmd.extend(['--texture', str(tex_w), str(tex_h)])
        if ts.obj2dl_scale != 1.0:
            cmd.extend(['--scale', str(ts.obj2dl_scale)])
        if ts.obj2dl_vertex_color:
            cmd.append('--use-vertex-color')
        if ts.obj2dl_multi_material:
            cmd.append('--multi-material')
        if ts.obj2dl_collision:
            cmd.append('--collision')

        print(f"NEA: Running: {' '.join(cmd)}")
        if not _run_tool(cmd, self.report):
            return {'CANCELLED'}

        # Clean up temp OBJ/MTL
        for ext in ('.obj', '.mtl'):
            p = os.path.join(out_dir, name + ext)
            if os.path.isfile(p):
                os.remove(p)

        self.report({'INFO'}, f"OK: {bin_path}")
        print(f"NEA: Done -> {bin_path}")
        return {'FINISHED'}


class NEA_OT_RunMd5ToDsma(bpy.types.Operator):
    """Convert MD5 mesh/anim files to DSM/DSA with md5_to_dsma"""
    bl_idname = "nea.run_md5_to_dsma"
    bl_label = "Convert MD5 (md5_to_dsma)"
    bl_options = {'REGISTER'}

    def execute(self, context):
        try:
            return self._run(context)
        except Exception as e:
            msg = f"md5_to_dsma error: {e}"
            print(f"NEA ERROR: {msg}")
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}

    def _run(self, context):
        ts = context.scene.nea_tool_settings
        out_dir = _resolve_output_dir(ts)
        if not out_dir:
            self.report({'WARNING'},
                        "Set Output Directory or save .blend file first")
            return {'CANCELLED'}
        os.makedirs(out_dir, exist_ok=True)

        md5_to_dsma_path = _find_nea_tool('md5_to_dsma')
        if not md5_to_dsma_path:
            self.report({'WARNING'},
                        "md5_to_dsma.py not found. Set repo path in "
                        "Edit > Preferences > Add-ons > NEA")
            return {'CANCELLED'}
        print(f"NEA: Using md5_to_dsma at {md5_to_dsma_path}")

        mesh_path = bpy.path.abspath(ts.md5_mesh_path)
        if not mesh_path or not os.path.isfile(mesh_path):
            self.report({'WARNING'},
                        f"MD5mesh file not found: {mesh_path or '(empty)'}")
            return {'CANCELLED'}

        # Get texture size from active object's mesh children
        tex_w, tex_h = 0, 0
        ao = context.active_object
        if ao:
            meshes = [ao] if ao.type == 'MESH' else []
            meshes += [c for c in ao.children if c.type == 'MESH']
            for child in meshes:
                if child.data.materials:
                    mat = child.data.materials[0]
                    if mat and mat.node_tree:
                        for node in mat.node_tree.nodes:
                            if node.type == 'TEX_IMAGE' and node.image:
                                tex_w = node.image.size[0]
                                tex_h = node.image.size[1]
                                break
                if tex_w > 0:
                    break

        cmd = ['/usr/bin/python3', md5_to_dsma_path,
               '--name', ts.md5_model_name,
               '--output', out_dir,
               '--model', mesh_path,
               '--bin']
        if tex_w > 0 and tex_h > 0:
            cmd.extend(['--texture', str(tex_w), str(tex_h)])
        if ts.md5_blender_fix:
            cmd.append('--blender-fix')
        if ts.md5_export_base_pose:
            cmd.append('--export-base-pose')
        if ts.md5_multi_material:
            cmd.append('--multi-material')

        print(f"NEA: Running: {' '.join(cmd)}")
        if not _run_tool(cmd, self.report):
            return {'CANCELLED'}

        result_path = f"{out_dir}/{ts.md5_model_name}"
        self.report({'INFO'}, f"OK: {result_path}")
        print(f"NEA: Done -> {result_path}")
        return {'FINISHED'}


class NEA_OT_RunPtexconv(bpy.types.Operator):
    """Convert active object's material texture to NDS format with ptexconv"""
    bl_idname = "nea.run_ptexconv"
    bl_label = "Convert Texture (ptexconv)"
    bl_options = {'REGISTER'}

    def execute(self, context):
        try:
            return self._run(context)
        except Exception as e:
            msg = f"ptexconv error: {e}"
            print(f"NEA ERROR: {msg}")
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}

    def _run(self, context):
        import tempfile

        obj = context.active_object
        if obj is None or obj.type != 'MESH' or not obj.data.materials:
            self.report({'WARNING'},
                        "Select a mesh with a material first")
            return {'CANCELLED'}

        ptexconv_path = '/opt/wonderful/thirdparty/blocksds/external/' \
                        'ptexconv/ptexconv'
        if not os.path.isfile(ptexconv_path):
            self.report({'WARNING'},
                        f"ptexconv not found at {ptexconv_path}")
            return {'CANCELLED'}

        mat = obj.active_material
        if not mat:
            self.report({'WARNING'}, "No active material on object")
            return {'CANCELLED'}

        ptex = mat.nea_ptexconv

        # Find texture image from material's node tree
        tex_path = None
        if mat.node_tree:
            for node in mat.node_tree.nodes:
                if node.type == 'TEX_IMAGE' and node.image:
                    img = node.image
                    if img.packed_file:
                        tmp = os.path.join(tempfile.gettempdir(),
                                           img.name + '.png')
                        img.save(filepath=tmp)
                        tex_path = tmp
                    else:
                        tex_path = bpy.path.abspath(img.filepath)
                    break

        if not tex_path or not os.path.isfile(tex_path):
            self.report({'WARNING'},
                        "No texture image found on material. "
                        "Assign a texture via Image Texture node.")
            return {'CANCELLED'}

        ts = context.scene.nea_tool_settings
        out_dir = _resolve_output_dir(ts)
        if not out_dir:
            self.report({'WARNING'},
                        "Set Output Directory or save .blend file first")
            return {'CANCELLED'}
        mat_name = re.sub(r'[^A-Za-z0-9_]', '_', mat.name)
        mat_dir = os.path.join(out_dir, mat_name)
        os.makedirs(mat_dir, exist_ok=True)

        cmd = [ptexconv_path,
               '-gt', '-ob',
               '-o', mat_dir,
               tex_path,
               '-f', ptex.tex_format]

        print(f"NEA: Running: {' '.join(cmd)}")
        if not _run_tool(cmd, self.report):
            return {'CANCELLED'}

        self.report({'INFO'},
                    f"OK: {mat_dir} ({ptex.tex_format})")
        print(f"NEA: Done -> {mat_dir}")
        return {'FINISHED'}


class VIEW3D_PT_nea_tools(bpy.types.Panel):
    """NEA conversion tools panel in 3D viewport N-panel"""
    bl_label = "NEA Tools"
    bl_idname = "VIEW3D_PT_nea_tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "NEA"

    def draw(self, context):
        layout = self.layout
        ts = context.scene.nea_tool_settings

        # Show hint if no tools can be found
        if not _find_nea_tool('obj2dl'):
            layout.label(text="Tools not found!", icon='ERROR')
            layout.label(text="Set repo path in:")
            layout.label(text="  Preferences > Add-ons > NEA")
            layout.separator()

        # Shared output directory
        layout.prop(ts, "output_dir")
        resolved = _resolve_output_dir(ts)
        if resolved:
            layout.label(text=f"-> {resolved}", icon='CHECKMARK')
        else:
            layout.label(
                text="Save .blend or set absolute path!",
                icon='ERROR')
        layout.separator()

        # --- obj2dl ---
        box = layout.box()
        box.label(text="Static Mesh (obj2dl):", icon='MESH_DATA')
        box.prop(ts, "obj2dl_scale")
        box.prop(ts, "obj2dl_vertex_color")
        box.prop(ts, "obj2dl_multi_material")
        box.prop(ts, "obj2dl_collision")
        row = box.row()
        row.scale_y = 1.4
        row.operator("nea.run_obj2dl", icon='EXPORT')

        # --- md5_to_dsma ---
        box2 = layout.box()
        box2.label(text="Animated Mesh (md5_to_dsma):", icon='ARMATURE_DATA')
        box2.prop(ts, "md5_mesh_path")
        box2.prop(ts, "md5_model_name")
        box2.prop(ts, "md5_blender_fix")
        box2.prop(ts, "md5_export_base_pose")
        box2.prop(ts, "md5_multi_material")
        row2 = box2.row()
        row2.scale_y = 1.4
        row2.operator("nea.run_md5_to_dsma", icon='EXPORT')

        # --- ptexconv ---
        box3 = layout.box()
        box3.label(text="Texture (ptexconv):", icon='TEXTURE')
        obj = context.active_object
        if obj and obj.type == 'MESH' and obj.active_material:
            mat = obj.active_material
            ptex = mat.nea_ptexconv
            box3.prop(ptex, "tex_format")
            box3.label(text=f"Material: {mat.name}")
        else:
            box3.label(text="Select mesh with material", icon='INFO')
        row3 = box3.row()
        row3.scale_y = 1.4
        row3.operator("nea.run_ptexconv", icon='EXPORT')


###
### Import functions
###

### .md5mesh import

def read_md5mesh(path, matrix, mergeVertices):
    meshName = path.split(os.sep)[-1].split(".")[-2]
    collection = bpy.data.collections.new(meshName)
    bpy.context.scene.collection.children.link(collection)
        
    i = "\s+(\d+)"
    w = "\s+(.+?)"
    a = "(.+?)"
    j_re  = re.compile(
        "\s*\""+a+"\""+w+"\s+\("+w*3+"\s+\)\s+\("+w*3+"\s+\).*")
    v_re  = re.compile("\s*vert"+i+"\s+\("+w*2+"\s+\)"+i*2+".*")
    t_re  = re.compile("\s*tri"+i*4+".*")
    w_re  = re.compile("\s*weight"+i*2+w+"\s+\("+w*3+"\).*")
    e_re  = re.compile("\s*}.*")
    js_re = re.compile("\s*joints\s+{.*")
    n_re  = re.compile("\s*(numverts).*")
    m_re  = re.compile("\s*mesh\s+{.*")
    s_re  = re.compile("\s*shader\s+\""+a+"\".*")
    fh = open(path, "r")
    md5mesh = fh.readlines()
    fh.close()
    m = None
    while not m:
        m = js_re.match(md5mesh.pop(0))
    arm_o, ms = do_joints(md5mesh, j_re, e_re, matrix, meshName, collection)
    pairs = []
    while md5mesh:
        mat_name, bm = do_mesh(md5mesh, s_re, v_re, t_re, w_re, e_re, n_re, ms)
        if mergeVertices > 0.00:
            bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=mergeVertices)
        pairs.append((mat_name, bm))
        skip_until(m_re, md5mesh)
    for mat_name, bm in pairs:
        
        translationtable = str.maketrans("\\", "/")
        tempstring = str.translate(mat_name, translationtable)
        lindex = str.rfind(tempstring, "/")
        if lindex==-1: lindex=0
        tempstring = tempstring[lindex+1:len(tempstring)]
        mesh = bpy.data.meshes.new(tempstring+"_mesh")
        bm.to_mesh(mesh)
        bm.free()
        mesh_o = bpy.data.objects.new(tempstring, mesh)
        vgs = mesh_o.vertex_groups
        for jn, _ in ms:
            vgs.new(name=jn)
        arm_mod = mesh_o.modifiers.new(type='ARMATURE', name=meshName+"_MD5_skeleton")
        arm_mod.object = arm_o
        #bpy.context.scene.collection.objects.link(mesh_o)
        collection.objects.link(mesh_o)
        oldactive = bpy.context.view_layer.objects.active
        bpy.context.view_layer.objects.active = mesh_o
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.object.material_slot_add()
        try:
            mat = bpy.data.materials[mat_name]
        except KeyError:
            mat = bpy.data.materials.new(mat_name)
        mesh_o.material_slots[-1].material = mat
        bpy.ops.object.mode_set()
        bpy.context.view_layer.objects.active = oldactive

def do_mesh(md5mesh, s_re, v_re, t_re, w_re, e_re, n_re, ms):
    bm = bmesh.new()
    mat_name = gather(s_re, n_re, md5mesh)[0][0]
    vs, ts, ws = gather_multi([v_re, t_re, w_re], e_re, md5mesh)
    wd  = bm.verts.layers.deform.verify()
    uvs = bm.loops.layers.uv.verify()
    for vi in range(len(vs)):
        wt, nwt = map(int, vs[vi][3:])
        w0 = ws[wt]
        mtx = ms[int(w0[1])][1]
        xyz = mtx @ mu.Vector(map(float, w0[3:]))
        new_v = bm.verts.new(xyz)
        bm.verts.index_update()
        for i in ws[wt:wt+nwt]:
            index = int(i[1])
            val = float(i[2])
            new_v[wd][index] = val
    bm.verts.ensure_lookup_table()
    for t in ts:
        bm.verts.ensure_lookup_table()
        tvs = [bm.verts[a] for a in map(int, t[1:])]
        tvy = tvs
        try:
            new_f = bm.faces.new([tvy[2],tvy[0],tvy[1]]) # fix windings for eyedeform issue
            new_f.normal_flip() #seems normals need to be flipped for 2.8
        except:
            continue
        bm.faces.index_update()
        for vn in tvs:
            ln = [l for l in new_f.loops if l.vert == vn][0]
            u0, v0 = map(float, vs[vn.index][1:3])
            ln[uvs].uv = (u0, 1.0 - v0)
    return mat_name, bm

def do_joints(md5mesh, j_re, e_re, correctMatrix, meshName, collection):
    joints = {}
    jdata = gather(j_re, e_re, md5mesh)
    for i in range(len(jdata)):
        joints[i] = jdata[i]
    arm = bpy.data.armatures.new(meshName+"_MD5_Skeleton")
    arm_o = bpy.data.objects.new(meshName+"_MD5_Armature", arm)
    collection.objects.link(arm_o)
    bpy.context.view_layer.objects.active = arm_o
    bpy.ops.object.mode_set()
    bpy.ops.object.mode_set(mode='EDIT')
    ebs = arm.edit_bones
    ms = []
    for j in joints.values():
        j_name = j[0]
        eb = ebs.new(j_name)
        p = int(j[1])
        if p >= 0:
            eb.parent = ebs[joints[p][0]]
        tx, ty, tz, rx, ry, rz = [float(a) for a in j[2:]]
        quat = -mu.Quaternion(restore_quat(rx, ry, rz))
        mtx = mu.Matrix.Translation((tx, ty, tz)) @ quat.to_matrix().to_4x4()
        mtx =  correctMatrix.to_4x4() @ mtx
        ms.append((j_name, mtx))
        eb.head = (0.0, 0.0, 0.0)
        eb.tail = (0.0, 1.0, 0.0)
        eb.matrix = mtx
        eb.length = 5.0
    bpy.ops.object.mode_set()
    bcoll_name = bpy.context.scene.md5_bone_collection
    bcoll = arm.collections.get(bcoll_name)
    if bcoll is None:
        bcoll = arm.collections.new(bcoll_name)
    for b in arm.bones:
        bcoll.assign(b)
    return arm_o, ms

### .md5anim import functions

def read_md5anim(fullPath,animName,prepend,correctionMatrix):
    
    label = fullPath.split(os.sep)[-1].split(".")[-2]
    action_name = label
    
    ao = bpy.context.active_object
    skel = bone_tree_blender(ao.data)
    
    print("Importing anim " + animName)
       
    if not ao.animation_data:
        ao.animation_data_create()
    
    if prepend:
        prefix = "(" + ao.name.replace("_MD5_Armature","") + ")_" 
        animName = prefix+animName
        
    ao.animation_data.action = bpy.data.actions.new(name=animName)
    action = ao.animation_data.action
    slot = action.slots.new(for_id=ao)
    ao.animation_data.action_slot = slot

    fh = open(fullPath, "r")
    md5anim = fh.readlines()
    fh.close()
    w = "\s+(.+?)"
    a = "(.+?)"
    j_re   = re.compile("\s*\""+a+"\""+w*3+".*")
    e_re   = re.compile("\s*}.*")
    bf0_re = re.compile("\s*(baseframe)\s+{.*")
    bf1_re = re.compile("\s*\("+w*3+"\s+\)\s+\("+w*3+"\s+\).*")
    f_re   = re.compile("\s*(frame).*")
    hier = gather(j_re, e_re, md5anim)
    for i in range(len(hier)):
        jname, par, flags = hier[i][:-1]
        hier[i] = (i, jname, int(par), int(flags))
    md5skel = bone_tree_md5(hier)
    if skel != md5skel:
        return message("no_arm_match"), {'CANCELLED'}
    skip_until(bf0_re, md5anim)
    bframe = gather(bf1_re, e_re, md5anim)
    bxfs = get_base_xforms(bframe)
    skip_until(f_re, md5anim)
    pbs = [ao.pose.bones[j[1]] for j in hier]
    frames = pad_frames(hier, bxfs, do_frames(e_re, md5anim))
    fcurves = get_fcurves(pbs, action, slot)
    xf_keys = convert_xforms(pbs, transpose(frames),correctionMatrix)
   
    end_frame = len(frames) - 1
    set_keys(
        flatten_channels(fcurves),
        flatten_frames(xf_keys),
        0)
    
   
    return "Animation imported successfully.", {'FINISHED'}


def do_frames(e_re, md5anim):
    valid = re.compile("[-\.0-9\s]+")
    val   = re.compile("(\S+)")
    result = [[]]
    while md5anim:
        l = md5anim.pop(0)
        if e_re.match(l):
            result.append([])
            continue
        if valid.match(l):
            vals = [float(a) for a in val.findall(l)]
            if vals:
                result[-1].append(vals)
    return [a for a in result if a]

def get_base_xforms(bframe):
    return [[float(b) for b in a] for a in bframe]

def convert_xforms(pbs, val_lists, correctionMatrix):
    result = []
    wm = bpy.context.window_manager
    print("cursor update max = "+str(len(val_lists)))
    wm.progress_begin(0, len(val_lists))
    curfr = 0
    for pb, states in zip(pbs, val_lists):
        curfr = curfr + 1
        wm.progress_update(curfr)
        result.append([])
        par = pb.parent
        if par:
            tweak = pb.bone.matrix_local.inverted() @ par.bone.matrix_local
        else:
            tweak =  pb.bone.matrix_local.inverted() @ correctionMatrix
        for vl in states:
            l = mu.Vector(vl[:3])
            q = mu.Quaternion(restore_quat(vl[3], vl[4], vl[5]))
            mtx = q.to_matrix().to_4x4()
            mtx.translation = l
            mtx = tweak @ mtx
            xf = []
            xf.extend(mtx.translation[:])
            xf.extend(mtx.to_quaternion()[:])
            xf.extend(mtx.to_euler()[:])
            result[-1].append(xf)
    wm.progress_end()
    return result

def pad_frames(hier, bxfs, frames):
    result = []
    for val_lists in frames:
        result.append([])
        for j, xf in zip(hier, bxfs):
            xf0 = xf[:]
            flags = j[3]
            if not flags:
                vl = []
            else:
                vl = val_lists.pop(0)
            mask = 1
            for i in range(6):
                if mask & flags:
                    xf0[i] = vl.pop(0)
                mask *= 2
            result[-1].append(xf0)
    return result

def transpose(table):
    result = []
    if table:
        while table[0]:
            result.append([])
            for col in table:
                result[-1].append(col.pop(0))
    return result

def get_fcurves(pbs, action, slot):
    from bpy_extras.anim_utils import action_ensure_channelbag_for_slot
    channelbag = action_ensure_channelbag_for_slot(action, slot)
    cf = float(bpy.context.scene.frame_current)
    fcurves = []
    l = "location"
    q = "rotation_quaternion"
    e = "rotation_euler"
    for pb in pbs:
        actiongroup = channelbag.channel_groups.new(pb.name)
        pb.keyframe_insert(l)
        pb.keyframe_insert(q)
        pb.keyframe_insert(e)
        entry = {l:{},q:{},e:{}}
        pbn = pb.name
        fc_re = re.compile("pose\.bones\[."+pbn+".\]\.("+l+"|"+q+"|"+e+")")
        for fc in channelbag.fcurves:
            m = fc_re.match(fc.data_path)
            if m:
                key1 = m.group(1)
                key2 = fc.array_index
                entry[key1][key2] = fc
                fc.group = actiongroup
        fcurves.append(entry)
    return fcurves

def list_fcurves(fcurves):
    l = "location"
    q = "rotation_quaternion"
    e = "rotation_euler"
    return [
        fcurves[l][0], fcurves[l][1], fcurves[l][2],
        fcurves[q][0], fcurves[q][1], fcurves[q][2], fcurves[q][3],
        fcurves[e][0], fcurves[e][1], fcurves[e][2]]

def flatten_channels(fcurves):
    result = []
    for a in fcurves:
        result.extend([b.keyframe_points for b in list_fcurves(a)])
    return result

def flatten_frames(pbs): # :: [[[a]]] -> [[a]]
    result = []
    for b in pbs:
        temp = [[] for _ in range(10)]
        for frame in b:
            for i in range(10):
                temp[i].append(frame[i])
        result.extend(temp)
    return result

def set_keys(channels, val_lists, f_start):
    for ch, vl in zip(channels, val_lists):
        i = f_start
        for v in vl:
            ch.insert(i, v)
            i += 1

### parsing and utility functions

def gather(regex, end_regex, ls):
    return gather_multi([regex], end_regex, ls)[0]
 
def gather_multi(regexes, end_regex, ls):
    result = [[] for _ in regexes]
    n = len(regexes)
    while ls:
        l = ls.pop(0)
        if end_regex.match(l):
            break
        for i in range(n):
            m = regexes[i].match(l)
            if m:
                result[i].append(m.groups())
                break
    return result

def skip_until(regex, ls):
    while ls:
        if regex.match(ls.pop(0)):
            break

def restore_quat(rx, ry, rz):
    t = 1.0 - (rx * rx) - (ry * ry) - (rz * rz)
    if t < 0.0:
        return (0.0, rx, ry, rz)
    else:
        return (-math.sqrt(t), rx, ry, rz)

def bone_tree_blender(arm):
    bcoll_name = bpy.context.scene.md5_bone_collection
    bcoll = arm.collections.get(bcoll_name)
    if bcoll is None:
        return btb(None, list(arm.bones))
    return btb(None, [b for b in arm.bones if bcoll_name in [c.name for c in b.collections]])

def btb(b, bs): # recursive; shouldn't matter for poxy md5 skeletons
    ch = sorted([a for a in bs if a.parent == b], key=lambda x: x.name)
    return [[c.name, btb(c, bs)] for c in ch]

def bone_tree_md5(lst):
    root = [a for a in lst if a[2] == -1][0]
    return [[root[1], btm(root, lst)]]

def btm(e, l):
    ch = sorted([a for a in l if a[2] == e[0]], key=lambda x: x[1])
    return [[c[1], btm(c, l)] for c in ch]

###
### Export functions
###

def get_ranges(markerFilter):
    markers = bpy.context.scene.timeline_markers
    starts = [m for m in markers if
        m.name.startswith(markerFilter) and
        m.name.endswith("_start", 2)]
    ends = [m for m in markers if
        m.name.startswith(markerFilter) and
        m.name.endswith("_end", 2)]
    if not starts or not ends:
        return None
    else:
        return find_matches(starts, ends)
    
def find_matches(starts, ends):
    pairs = {}
    for s in starts:
        basename = s.name[:s.name.rfind("_start")]
        matches = [e for e in ends if
            e.name[:e.name.rfind("_end")] == basename]
        if matches:
            m = matches[0]
            pairs[basename] = (min(s.frame, m.frame), max(s.frame, m.frame))
    return pairs

def record_parameters(correctionMatrix):
    return "".join([
        " // Parameters used during export:",
        " Reorient: {};".format(bool(correctionMatrix.to_euler()[2])),
        " Scale: {}".format(correctionMatrix.decompose()[2][0])])

def define_components(obj, bm, bones, correctionMatrix):
    scaleFactor = correctionMatrix.to_scale()[0]
    armature = [a for a in bpy.data.armatures if bones[0] in a.bones[:]][0]
    armatureObj = [o for o in bpy.data.objects if o.data == armature][0]
    boneNames = [b.name for b in bones]
    allVertGroups = obj.vertex_groups[:]
    weightGroupIndexes = [vg.index for vg in allVertGroups if vg.name in boneNames]
    uvData = bm.loops.layers.uv.active
    weightData = bm.verts.layers.deform.active
    tris = [[f.index, f.verts[2].index, f.verts[1].index, f.verts[0].index]
        for f in bm.faces] # reverse vert order to flip normal
    verts = []
    weights = []
    wtIndex = 0
    firstWt = 0
    for vert in bm.verts:
        vGroupDict = vert[weightData]
        wtDict = dict([(k, vGroupDict[k]) for k in vGroupDict.keys()
            if k in weightGroupIndexes])
        u = vert.link_loops[0][uvData].uv.x
        v = 1 - vert.link_loops[0][uvData].uv.y # MD5 wants it flipped
        numWts = len(wtDict.keys())
        verts.append([vert.index, u, v, firstWt, numWts])
        wtScaleFactor = 1.0 / sum(wtDict.values())
        firstWt += numWts
        for vGroup in wtDict:
            bone = [b for b in bones
                if b.name == allVertGroups[vGroup].name][0]
            boneIndex = bones.index(bone)
            coords4d =\
                bone.matrix_local.inverted() @\
                armatureObj.matrix_world.inverted() @\
                obj.matrix_world @\
                (vert.co.to_4d() * scaleFactor)
            x, y, z = coords4d[:3]
            weight = wtDict[vGroup] * wtScaleFactor
            wtEntry = [wtIndex, boneIndex, weight, x, y, z]
            weights.append(wtEntry)
            wtIndex += 1
    return (verts, tris, weights)

def make_hierarchy_block(bones, boneIndexLookup):
    block = ["hierarchy {\n"]
    xformIndex = 0
    for b in bones:
        if b.parent:
            parentIndex = boneIndexLookup[b.parent.name]
        else:
            parentIndex = -1
        block.append("  \"{}\" {} 63 {} //\n".format(
            b.name, parentIndex, xformIndex))
        xformIndex += 6
    block.append("}\n")
    block.append("\n")
    return block

def make_baseframe_block(bones, correctionMatrix):
    block = ["baseframe {\n"]
    armature = bones[0].id_data
    armObject = [o for o in bpy.data.objects
        if o.data == armature][0]
    armMatrix = armObject.matrix_world
    for b in bones:
        objSpaceMatrix = b.matrix_local
        if b.parent:
            bMatrix =\
            b.parent.matrix_local.inverted() @\
            armMatrix @\
            objSpaceMatrix
        else:
            bMatrix = correctionMatrix @ objSpaceMatrix
        xPos, yPos, zPos = bMatrix.translation
        xOrient, yOrient, zOrient = (-bMatrix.to_quaternion()).normalized()[1:]
        block.append("  ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(xPos, yPos, zPos, xOrient, yOrient, zOrient))
    block.append("}\n")
    block.append("\n")
    return block

def make_joints_block(bones, boneIndexLookup, correctionMatrix):
    block = []
    block.append("joints {\n")
    for b in bones:
        if b.parent:
            parentIndex = boneIndexLookup[b.parent.name]
        else:
            parentIndex = -1
        boneMatrix = correctionMatrix @ b.matrix_local
        xPos, yPos, zPos = boneMatrix.translation
        xOrient, yOrient, zOrient =\
        (-boneMatrix.to_quaternion()).normalized()[1:] # MD5 wants it negated
        block.append(\
        "  \"{}\" {} ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(b.name, parentIndex,\
        xPos, yPos, zPos,\
        xOrient, yOrient, zOrient))
    block.append("}\n")
    block.append("\n")
    return block

def get_shader_name(material):
    """Prefer texture image path from node tree, fallback to material name."""
    if material and material.node_tree:
        for node in material.node_tree.nodes:
            if node.type == 'TEX_IMAGE' and node.image:
                path = node.image.filepath
                if path:
                    return os.path.basename(bpy.path.abspath(path))
    return material.name if material else "default"

def make_mesh_block(obj, bones, correctionMatrix, fixWindings, material_index=None):
    shaderName = "default"
    ms = obj.material_slots
    if material_index is not None and material_index < len(ms):
        mat = ms[material_index].material
        if mat:
            shaderName = get_shader_name(mat)
    elif ms:
        taken = [s for s in ms if s.material]
        if taken:
            shaderName = get_shader_name(taken[0].material)
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    if material_index is not None:
        # Remove faces not belonging to this material
        for f in [f for f in bm.faces if f.material_index != material_index]:
            bm.faces.remove(f)
        for e in [e for e in bm.edges if not e.link_faces[:]]:
            bm.edges.remove(e)
        for v in [v for v in bm.verts if not v.link_faces[:]]:
            bm.verts.remove(v)
        for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
    triangulate(cut_up(strip_wires(bm)))
    verts, tris, weights = define_components(obj, bm, bones, correctionMatrix)
    bm.free()
    block = []
    block.append("mesh {\n")
    block.append("  shader \"{}\"\n".format(shaderName))
    block.append("\n  numverts {}\n".format(len(verts)))
    for v in verts:
        block.append(\
        "  vert {} ( {:.10f} {:.10f} ) {} {}\n".\
        format(v[0], v[1], v[2], v[3], v[4]))
    block.append("\n  numtris {}\n".format(len(tris)))
    for t in tris:
        if fixWindings:
            block.append("  tri {} {} {} {}\n".format(t[0], t[3], t[1], t[2])) # fix windings - current blender windings break eyeDeform in D3 materials
        else:
            block.append("  tri {} {} {} {}\n".format(t[0], t[1], t[2], t[3]))
        
    block.append("\n  numweights {}\n".format(len(weights)))
    for w in weights:
        block.append(\
        "  weight {} {} {:.10f} ( {:.10f} {:.10f} {:.10f} )\n".\
        format(w[0], w[1], w[2], w[3], w[4], w[5]))
    block.append("}\n")
    block.append("\n")
    return block

def strip_wires(bm):
    [bm.faces.remove(f) for f in bm.faces if len(f.verts) < 3]
    [bm.edges.remove(e) for e in bm.edges if not e.link_faces[:]]
    [bm.verts.remove(v) for v in bm.verts if v.is_wire]
    for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
    return bm

def cut_up(bm):
    uvData = bm.loops.layers.uv.active
    for v in bm.verts:
        for e in v.link_edges:
            linkedFaces = e.link_faces
            if len(linkedFaces) > 1:
                uvSets = []
                for lf in linkedFaces:
                    uvSets.append([l1[uvData].uv for l1 in lf.loops
                        if l1.vert == v][0])
                if uvSets.count(uvSets[0]) != len(uvSets):
                    e.tag = True
                    v.tag = True
        if v.tag:
            seams = [e for e in v.link_edges if e.tag]
            v.tag = False
            bmesh.utils.vert_separate(v, seams)
    for maybeBowTie in bm.verts: # seems there's no point in a proper test
        boundaries = [e for e in maybeBowTie.link_edges
            if len(e.link_faces) == 1]
        bmesh.utils.vert_separate(maybeBowTie, boundaries)
    for seq in [bm.verts, bm.faces, bm.edges]: seq.index_update()
    return bm

def triangulate(bm):
    nonTris = [f for f in bm.faces if len(f.verts) > 3]
    bmesh.ops.triangulate(bm, faces=nonTris)
    return bm

def write_md5mesh(filePath, prerequisites, correctionMatrix, fixWindings):
    bones, meshObjects = prerequisites
    boneIndexLookup = {}
    for b in bones:
        boneIndexLookup[b.name] = bones.index(b)
    md5joints = make_joints_block(bones, boneIndexLookup, correctionMatrix)
    md5meshes = []
    for mo in meshObjects:
        if len(mo.material_slots) > 1:
            # Check which material indices actually have faces assigned
            bm_check = bmesh.new()
            bm_check.from_mesh(mo.data)
            used_materials = set(f.material_index for f in bm_check.faces)
            bm_check.free()
            for mat_idx in sorted(used_materials):
                md5meshes.append(make_mesh_block(
                    mo, bones, correctionMatrix, fixWindings,
                    material_index=mat_idx))
        else:
            md5meshes.append(make_mesh_block(mo, bones, correctionMatrix, fixWindings))
    f = open(filePath, 'w')
    lines = []
    lines.append("MD5Version 10" + record_parameters(correctionMatrix) + "\n")
    lines.append("commandline \"\"\n")
    lines.append("\n")
    lines.append("numJoints " + str(len(bones)) + "\n")
    lines.append("numMeshes " + str(len(md5meshes)) + "\n")
    lines.append("\n")
    lines.extend(md5joints)
    for m in md5meshes: lines.extend(m)
    for line in lines: f.write(line)
    f.close()
    return

def write_md5anim(filePath, prerequisites, correctionMatrix, previewKeys, frame_range ):
    
    #export the .md5anim for the action currently associated with the armature animation
      
    bpy.context.view_layer.update()
    
    goBack = bpy.context.scene.frame_current
    area = bpy.context.area
    if ( area ):
        previous_type = area.type
    
    if ( previewKeys ):
        startFrame = bpy.context.scene.frame_preview_start
        endFrame = bpy.context.scene.frame_preview_end
    else:
        startFrame = int(frame_range[0])
        endFrame = int(frame_range[-1])

    bones, meshObjects = prerequisites
    armObj = [o for o in bpy.data.objects if o.data == bones[0].id_data][0]
    pBones = armObj.pose.bones
    boneIndexLookup = {}
    for b in bones:
        boneIndexLookup[b.name] = bones.index(b)
    hierarchy = make_hierarchy_block(bones, boneIndexLookup)
    baseframe = make_baseframe_block(bones, correctionMatrix)
    bounds = []
    frames = []
    for frame in range(startFrame, endFrame + 1):
        bpy.context.scene.frame_set(frame)
        verts = []
        depsgraph = bpy.context.evaluated_depsgraph_get()
        for mo in meshObjects:
            mo_eval = mo.evaluated_get(depsgraph)
            mesh_eval = mo_eval.to_mesh()
            verts.extend([correctionMatrix @ mo.matrix_world @ v.co.to_4d()
                for v in mesh_eval.vertices])
            mo_eval.to_mesh_clear()
        if verts:
            minX = min([co[0] for co in verts])
            minY = min([co[1] for co in verts])
            minZ = min([co[2] for co in verts])
            maxX = max([co[0] for co in verts])
            maxY = max([co[1] for co in verts])
            maxZ = max([co[2] for co in verts])
        else:
            minX = minY = minZ = maxX = maxY = maxZ = 0.0
        bounds.append(\
        "  ( {:.10f} {:.10f} {:.10f} ) ( {:.10f} {:.10f} {:.10f} )\n".\
        format(minX, minY, minZ, maxX, maxY, maxZ))
        frameBlock = ["frame {} {{\n".format(frame - startFrame)]
        scaleFactor = correctionMatrix.to_scale()[0]
        for b in bones:
            pBone = pBones[b.name]
            pBoneMatrix = pBone.matrix
            if pBone.parent:
                diffMatrix = pBone.parent.matrix.inverted() @ armObj.matrix_world @ (pBoneMatrix * scaleFactor)
            else:
                diffMatrix = correctionMatrix @ pBoneMatrix
            xPos, yPos, zPos = diffMatrix.translation
            xOrient, yOrient, zOrient =\
            (-diffMatrix.to_quaternion()).normalized()[1:]
            frameBlock.append(\
            "  {:.10f} {:.10f} {:.10f} {:.10f} {:.10f} {:.10f}\n".\
            format(xPos, yPos, zPos, xOrient, yOrient, zOrient))
        frameBlock.append("}\n")
        frameBlock.append("\n")
        frames.extend(frameBlock)
    f = open(filePath, 'w')
    numJoints = len(bones)
    bounds.insert(0, "bounds {\n")
    bounds.append("}\n")
    bounds.append("\n")
    lines = []
    lines.append("MD5Version 10" + record_parameters(correctionMatrix) + "\n")
    lines.append("commandline \"\"\n")
    lines.append("\n")
    lines.append("numFrames " + str(endFrame - startFrame + 1) + "\n")
    lines.append("numJoints " + str(numJoints) + "\n")
    lines.append("frameRate " + str(bpy.context.scene.render.fps) + "\n")
    lines.append("numAnimatedComponents " + str(numJoints * 6) + "\n")
    lines.append("\n")
    for chunk in [hierarchy, bounds, baseframe, frames]:
        lines.extend(chunk)
    for line in lines:
        f.write(line)
    bpy.context.scene.frame_set(goBack)
    return


def _auto_capsule_from_bone(bone):
    """Generate capsule collision parameters from bone geometry.

    Uses the bone's length for half_height and a fraction of length for radius.
    The offset centers the capsule along the bone's local Y axis (head to tail).
    """
    length = bone.length
    if length < 0.001:
        return None

    # Use envelope radius if available and reasonable, otherwise derive from
    # bone length.  Blender bones have head_radius / tail_radius from envelope
    # display, but these are only meaningful when the user has edited them.
    # Fall back to a fraction of bone length.
    env_radius = getattr(bone, 'head_radius', 0.0)
    if env_radius > 0.01:
        radius = env_radius
    else:
        radius = length * 0.2

    half_height = length * 0.5
    # Offset to center of bone along its local Y axis
    offset = (0.0, length * 0.5, 0.0)

    return {
        'type': 'capsule',
        'radius': radius,
        'half_height': half_height,
        'offset': offset,
    }


def write_md5collimesh(filepath, bones):
    """Write a .md5collimesh text file from bone collision properties.

    Bones with manually configured collision shapes (col_type != 'none') use
    those settings.  Bones left at the default 'none' get an auto-generated
    capsule shape derived from the bone's length and envelope radius.
    """
    count = 0

    with open(filepath, 'w') as f:
        # First pass: count entries (manual + auto-generated)
        entries = []
        for bone in bones:
            props = bone.nea_collision
            if props.col_type != 'none':
                entries.append(('manual', bone, props))
            else:
                auto = _auto_capsule_from_bone(bone)
                if auto is not None:
                    entries.append(('auto', bone, auto))

        f.write("MD5CollisionVersion 1\n")
        f.write(f"numBones {len(entries)}\n\n")

        for kind, bone, data in entries:
            f.write(f'bone "{bone.name}" {{\n')

            if kind == 'manual':
                props = data
                f.write(f'  type {props.col_type}\n')
                if props.col_type in ('sphere', 'capsule'):
                    f.write(f'  radius {props.radius:.6f}\n')
                if props.col_type == 'capsule':
                    f.write(f'  half_height {props.half_height:.6f}\n')
                if props.col_type == 'aabb':
                    f.write(f'  half_extents {props.half_x:.6f} '
                            f'{props.half_y:.6f} {props.half_z:.6f}\n')
                f.write(f'  offset {props.offset_x:.6f} '
                        f'{props.offset_y:.6f} {props.offset_z:.6f}\n')
            else:
                # Auto-generated capsule from bone geometry
                f.write(f'  type {data["type"]}\n')
                f.write(f'  radius {data["radius"]:.6f}\n')
                f.write(f'  half_height {data["half_height"]:.6f}\n')
                ox, oy, oz = data['offset']
                f.write(f'  offset {ox:.6f} {oy:.6f} {oz:.6f}\n')

            f.write('}\n\n')

        count = len(entries)

    return count


###
### Operators and auxiliary functions
###

# Functions

def concat_strings(strings):
    result = ""
    for s in strings:
        result = result + "\n" + s
    return result

def message(id, *details):
    if id == 'no_deformables':
        return """No armature-deformed meshes found.\n
Select the collection or object you want to export, and retry export."""
    elif id == 'multiple_armatures':
        return """The selected object, or the collection it belongs to, contains more than one\n armature.
Select an object in a collection that cantains and uses only one armature, and try again."""
    elif id == 'no_armature':
        return """No deforming armature is associated with the selected object or it's collection.
Select the collection, or an object in the collection you want to export, and try again"""
    elif id == 'collection_empty':
        bc = bpy.context.scene.md5_bone_collection
        return "The deforming armature has no bones in collection '" + bc + """'.\n
Add all of the bones you want to export to the bone collection '""" +\
        bc + """',
\nor change the bone collection name in the scene properties, and retry export.\n
  Bone collections can be managed in the 'Object Data Properties' section of the\n
properties toolbar of the armature."""
    elif id == 'missing_parents':
        bc = bpy.context.scene.md5_bone_collection
        return "One or more bones in the armature have parents outside collection '" + bc + """'.\n
Revise your armature's bone collection '""" + bc + """' membership,\n
or change the bone collection name, and retry export.\n
(Bone collections can be managed in the 'Object Data Properties' section of the\n
properties toolbar of the armature.)\n
Offending bones:""" + concat_strings(details[0])
    elif id == 'orphans':
        bc = bpy.context.scene.md5_bone_collection
        return """There are multiple root bones (listed below)
in the export-bound collection, but only one root bone\n
is allowed in MD5. Revise your bone collection '""" + bc + """' membership,\n
or change the bone collection name and retry export.\n
(Bone collections can be managed in the 'Object Data Properties' section of the\n
properties toolbar of the armature.)\n
Root bones:""" + concat_strings(details[0])
    elif id == 'unweighted_verts':
        if details[0][1] == 1:
            count = " 1 vertex "
        else:
            count = " " + str(details[0][1]) + " vertices "
        return "The '" + details[0][0] + "' object contains" + count +\
        """with no deformation weights assigned.\n
Valid MD5 data cannot be produced. Paint non-zero weights\n
on all the vertices in the mesh, and retry export."""
    elif id == 'zero_weight_verts':
        if details[0][1] == 1:
            count = " 1 vertex "
        else:
            count = " " + str(details[0][1]) + " vertices "
        return "The '" + details[0][0] + "' object contains" + count +\
        """with zero weights assigned.\n
This can cause adverse effects.\n
Paint non-zero weights on all the vertices in the mesh,\n
or use the Clean operation in the weight paint tools.\n
( if using Clean, anything with a weight less 0.000001 \n
is considered a zero weight, so use this limit in the clean tool)\n
Please correct zero weights and retry export. """
    elif id == 'no_uvs':
        return "The '" + details[0] + """' object has no UV coordinates.\n
Valid MD5 data cannot be produced. Unwrap the object\n
or exclude it from your selection, and retry export."""
    elif id == 'no_arm':
        return """No armature found to add animation to.\n  
The active object is not an Armature, or the collection the active\n
object belongs to does not contain a valid armature.\n
Select a valid armature or object in the desired collection, and retry import."""
    elif id == 'no_arm_match':
        return """The selected armature does not match the skeleton\n
in the file you are trying to import."""



def check_weighting(obj, bm, bones):
    boneNames = [b.name for b in bones]
    allVertGroups = obj.vertex_groups[:]
    weightGroups = [vg for vg in allVertGroups if vg.name in boneNames]
    weightGroupIndexes = [vg.index for vg in allVertGroups if vg.name in boneNames]
    weightData = bm.verts.layers.deform.active
    unweightedVerts = 0
    zeroWeightVerts = 0
    for v in bm.verts:
        influences = [wgi for wgi in weightGroupIndexes
            if wgi in v[weightData].keys()]
        if not influences:
            unweightedVerts += 1
        else:
            for wgi in influences:
                if v[weightData][wgi] < 0.000001:
                    zeroWeightVerts += 1
                    print("Zero Weight %s ( checking against limit of 0.000001 )",v[weightData][wgi])
                    v.select_set(True)
    return (unweightedVerts, zeroWeightVerts)

def is_export_go(what, collection):
    bcoll_name = bpy.context.scene.md5_bone_collection

    meshObjects = []

    #support single mesh export.
    #only supported by commandline, normal export is still as documented.
    if what == 'mesh':
        meshObjects = [ bpy.context.active_object ]
    else:
        meshObjects =[o for o in bpy.data.collections[collection.name].objects
            if o.data in bpy.data.meshes[:] and o.find_armature()]

    armatures = [a.find_armature() for a in meshObjects]
    if not meshObjects:
        return ['no_deformables', None]
    armature = armatures[0]
    if armatures.count(armature) < len(meshObjects):
        return ['multiple_armatures', None]
    bcoll = armature.data.collections.get(bcoll_name)
    if bcoll is not None:
        bones = [b for b in armature.data.bones if bcoll_name in [c.name for c in b.collections]]
    else:
        bones = list(armature.data.bones)
    if not bones:
        return ['collection_empty', None]
    rootBones = [i for i in bones if not i.parent]
    if len(rootBones) > 1:
        boneList = []
        for rb in rootBones:
            boneList.append("- " + str(rb.name))
        return ['orphans', boneList]
    abandonedBones = [i for i in bones
        if i.parent and i.parent not in bones[:]]
    if abandonedBones:
        boneList = []
        for ab in abandonedBones:
            boneList.append("- " + str(ab.name))
        return ['missing_parents', boneList]
    if what != 'anim':
        for mo in meshObjects:
            bm = bmesh.new()
            bm.from_mesh(mo.data)
            (unweightedVerts, zeroWeightVerts) = check_weighting(mo, bm, bones)
            uvLayer = bm.loops.layers.uv.active
            bm.free()
            if unweightedVerts > 0:
                return ['unweighted_verts', (mo.name, unweightedVerts)]
            if zeroWeightVerts > 0:
                return ['zero_weight_verts', (mo.name, zeroWeightVerts)]
            if not uvLayer:
                return ['no_uvs', mo.name]
    return ['ok', (bones, meshObjects)]

def manage_bone_collections(doWhat):
    bcoll_name = bpy.context.scene.md5_bone_collection
    arm = bpy.context.active_object.data
    bcoll = arm.collections.get(bcoll_name)
    if bcoll is None:
        bcoll = arm.collections.new(bcoll_name)
    mode = bpy.context.mode
    if mode == 'POSE':
        allBones = [pb.bone for pb in bpy.context.active_object.pose.bones]
        selBones = [pb.bone for pb in bpy.context.selected_pose_bones]
    elif mode == 'EDIT_ARMATURE':
        allBones = list(arm.edit_bones)
        selBones = list(bpy.context.selected_editable_bones)
    else:
        return
    unselBones = [b for b in allBones if b not in selBones]
    if doWhat == 'replace':
        for x in selBones:
            bcoll.assign(x)
        for y in unselBones:
            bcoll.unassign(y)
        return
    elif doWhat == 'add':
        for x in selBones:
            bcoll.assign(x)
        return
    elif doWhat == 'remove':
        for x in selBones:
            bcoll.unassign(x)
        return
    elif doWhat == 'clear':
        for x in allBones:
            bcoll.unassign(x)
        return
    else: return

def delete_action(prepend,actionNameToDelete):

    print ("Deleting Action prepend: %s, action name %s", prepend, actionNameToDelete)
    ao = bpy.context.active_object
    collection = ao.users_collection[0]
    
    meshObjects = [o for o in bpy.data.collections[collection.name].objects
        if o.data in bpy.data.meshes[:] and o.find_armature()]
    
    armatures = [a.find_armature() for a in meshObjects]
    armature = armatures[0]   
    if (prepend):
        actionName = "("+collection.name+")_"+actionNameToDelete
    
    if armature.animation_data.action.name == actionName:
        
        bpy.data.actions.remove(bpy.data.actions[actionName])
 
    return

def remove_prefix(text, prefix):
    if text.startswith(prefix):
        return text[len(prefix):]
    return text 

# Operators

### Import UI

class ImportMD5Mesh(bpy.types.Operator, ImportHelper):
    '''Import an .MD5mesh file as a new Collection'''
    bl_idname = "import_scene.md5mesh"
    bl_label = 'Import MD5MESH'
    bl_options = {'PRESET'}
    filename_ext = ".md5mesh"
    path_mode = path_reference_mode
    check_extension = True
        
    
    filter_glob : StringProperty(
        default="*.md5mesh",
        options={'HIDDEN'})

    reorientDegrees : bpy.props.EnumProperty(
            items= (('0', '0 Degrees', 'Do not reorient'),
                    ('90', '90 Degrees ( X to Y )', 'Rotate 90 degrees (e.g. reorient facing +X to facing +Y)'),
                    ('-90', '-90 Degrees ( Y to X )', 'Rotate -90 degrees (e.g. reorient facing +Y to facing +X' ),
                    ('180', '180 Degrees', 'Rotate 180 degrees')),
            name = "Reorient Model",
            description = "Degrees to rotate model during import.  Useful to reorient models to face Y axis if desired. 90 Degrees rotates clockwise from above. -90 Rotates counter-clockwise from above.",
            default = '0')

    scaleFactor : bpy.props.FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0)

    mergeVerticesCM : bpy.props.FloatProperty(
            name="Merge Vertices",
            description="Automatically weld near vertices (in centimetres). ( 0.0 = Default Disabled )",
            min=0.00, max=1.00,
            soft_min=0.00,
            soft_max=1.00,
            default=0.00)

    boneCollection : bpy.props.StringProperty(
            name="Bone Collection",
            description="Bones will be assigned to this collection. Make sure the collection name matches the one in 'Object Data Properties' of the armature for export.",
            default="MD5_Export")


    path_mode = path_reference_mode
    check_extension = True
    def execute(self, context):
                    
        rotdeg = float(self.reorientDegrees)
        orientationTweak = mu.Matrix.Rotation(math.radians( rotdeg ),4,'Z')
        
        scaleTweak = mu.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak @ scaleTweak
        bpy.context.scene.md5_bone_collection = self.boneCollection
        read_md5mesh(self.filepath, correctionMatrix, self.mergeVerticesCM * 0.01)
        return {'FINISHED'}

class MaybeImportMD5Anim(bpy.types.Operator):
    '''Import one or more .MD5anim files into dopesheet actions associated with the collection of the active object.'''
    bl_idname = "export_scene.maybe_import_md5anim"
    bl_label = 'Import MD5ANIM'
    def invoke(self, context, event):
        
        #check the active object first 
        ao = bpy.context.active_object
        if ao and ao.type == 'ARMATURE' and ao.data.bones[:]:
            return bpy.ops.import_scene.md5anim('INVOKE_DEFAULT')
        
        #if the active object isn't a valid armature, get it's collection and check
             
        if ao:
            collection = ao.users_collection[0]
            print("Using armature")
        else:
            collection = bpy.context.view_layer.active_layer_collection      

        print(collection)
        
        meshObjects = [o for o in bpy.data.collections[collection.name].objects
            if o.data in bpy.data.meshes[:] and o.find_armature()]
        
        armatures = [a.find_armature() for a in meshObjects]
        if meshObjects:
            armature = armatures[0]
            if armature.data.bones[:]:
                bpy.context.view_layer.objects.active = armature
                return bpy.ops.import_scene.md5anim('INVOKE_DEFAULT')
               
        #no valid armature selected or in the active collection
        msg = message("no_arm")
        bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msg )
        print(msg)
        self.report({'ERROR'}, msg)
        return {'CANCELLED'}
        

class ImportMD5Anim(bpy.types.Operator, ImportHelper):
    '''Load an MD5 Animation File'''
    
    
    bl_idname = "import_scene.md5anim"
    bl_label = 'Import MD5ANIM'
    bl_options = {'PRESET'}
    filename_ext = ".md5anim"
    
    path_mode = path_reference_mode
    check_extension = True
    check_extension = True
    #there must be an easier way to add a comment block to the import helper
    
            
    md5info : StringProperty(
        name="",
        default=".MD5anim import",
        )

    md5info1 : StringProperty(
            name="",
            default="Select one or more .md5anim files",
            )
    md5info2 : StringProperty(
            name="",
            default="to import as Blender actions.",
            )
    md5info3 : StringProperty(
            name="",
            default="(Batch import supported.)",
            )
    md5info4 : StringProperty(
            name="",
            default="After import, actions available",
            )
    md5info5 : StringProperty(
            name="",
            default="in the dopesheet action editor.",
            )

    prepend : BoolProperty(
            name="Prepend action name",
            description="Prepend the collection name to the animation name in the action editor. Necessary to bulk export actions and identify which actions are associated with which collection.",
            default=True,
            )

    filter_glob : StringProperty(
        default="*.md5anim",
        options={'HIDDEN'})

    files : CollectionProperty(
        name = "MD5Anim files",
        type = OperatorFileListElement,
        )

    directory : StringProperty(subtype='DIR_PATH')

    reorientDegrees : bpy.props.EnumProperty(
            items= (('0', '0 Degrees', 'Do not reorient'),
                    ('90', '90 Degrees ( X to Y )', 'Rotate 90 degrees (e.g. reorient facing +X to facing +Y)'),
                    ('-90', '-90 Degrees ( Y to X )', 'Rotate -90 degrees (e.g. reorient facing +Y to facing +X' ),
                    ('180', '180 Degrees', 'Rotate 180 degrees')),
            name = "Reorient Animation",
            description = "Degrees to rotate animation during import.  Useful to reorient to face Y axis if desired. 90 Degrees rotates clockwise from above. -90 Rotates counter-clockwise from above.",
            default = '0')

    scaleFactor : bpy.props.FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0)
                
            
    def execute(self, context):
        
        import os
        errors = 0
        successes = 0
        errorString = ""
        res = []
        msg2 = ""
                    
        rotdeg = float(self.reorientDegrees)
        orientationTweak = mu.Matrix.Rotation(math.radians( rotdeg ),4,'Z')
        scaleTweak = mu.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak @ scaleTweak
        
        for newAnim in self.files:
            fullAnimPath = os.path.join(self.directory,newAnim.name)
            print("Importing: " + fullAnimPath)
            
            try:
                msg, res = read_md5anim(fullAnimPath,newAnim.name,self.prepend,correctionMatrix)
                if res == {'CANCELLED'}:
                    self.report({'ERROR'}, msg)
                    print(msg)
                    errors = errors + 1
                    errorString = errorString + newAnim.name + ","
                    #return res
                   
                    #delete the newly created action since it failed
                    delete_action(self.prepend,newAnim.name)
                    
                else:
                    successes = successes + 1
            except:
                errors = errors + 1
                errorString = errorString + newAnim.name + ","
                #delete the newly created action since it failed
                delete_action(self.prepend,newAnim.name)
                continue    
            
        msg = str(successes) + " md5anim files successfully imported as actions.\n" 
            
        if errors >= 1:
            msg = msg + "ERROR! The following" + str(errors) + " imports failed - see System Console for details.\n" + errorString
             
        bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msg )
        
        if res:
            return res
        else:
            return {'CANCELLED'}
        
### Bone collection management

class MD5BonesAdd(bpy.types.Operator):
    '''Add the selected bones to the bone collection reserved for MD5'''
    bl_idname = "data.md5_bones_add"
    bl_label = 'Add Selected'
    def invoke(self, context, event):
        manage_bone_collections('add')
        return {'FINISHED'}

class MD5BonesRemove(bpy.types.Operator):
    '''Remove the selected bones from the bone collection reserved for MD5'''
    bl_idname = "data.md5_bones_remove"
    bl_label = 'Remove Selected'
    def invoke(self, context, event):
        manage_bone_collections('remove')
        return {'FINISHED'}

class MD5BonesReplace(bpy.types.Operator):
    '''Include only the selected bones in the bone collection reserved for MD5'''
    bl_idname = "data.md5_bones_replace"
    bl_label = 'Replace with Selected'
    def invoke(self, context, event):
        manage_bone_collections('replace')
        return {'FINISHED'}

class MD5BonesClear(bpy.types.Operator):
    '''Clear the bone collection reserved for MD5'''
    bl_idname = "data.md5_bones_clear"
    bl_label = 'Clear All'
    def invoke(self, context, event):
        manage_bone_collections('clear')
        return {'FINISHED'}

class MD5Panel(bpy.types.Panel):
    """MD5 parameters panel in the scene context of the properties editor"""
    bl_label = "MD5 Export Setup"
    bl_idname = "DATA_PT_md5"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"
    def draw(self, context):

        layout = self.layout
        bc = context.scene.md5_bone_collection
        layout.prop(context.scene, "md5_bone_collection")
        column1 = layout.column()
        column1.label(text="Manage collection '" + bc + "' membership:")
        column2 = column1.column(align=True)
        column2.operator("data.md5_bones_add")
        column2.operator("data.md5_bones_remove")
        column2.operator("data.md5_bones_replace")
        column2.operator("data.md5_bones_clear")
        if context.mode in {'POSE','EDIT_ARMATURE'}:
            column1.enabled = True
        else:
            column1.enabled = False


class BONE_PT_nea_collision(bpy.types.Panel):
    """NEA per-bone collision shape properties"""
    bl_label = "NEA Bone Collision"
    bl_idname = "BONE_PT_nea_collision"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "bone"

    @classmethod
    def poll(cls, context):
        return context.bone is not None

    def draw(self, context):
        layout = self.layout
        bone = context.bone
        props = bone.nea_collision

        layout.prop(props, "col_type")

        if props.col_type == 'sphere':
            layout.prop(props, "radius")
        elif props.col_type == 'capsule':
            layout.prop(props, "radius")
            layout.prop(props, "half_height")
        elif props.col_type == 'aabb':
            col = layout.column(align=True)
            col.prop(props, "half_x")
            col.prop(props, "half_y")
            col.prop(props, "half_z")

        if props.col_type != 'none':
            box = layout.box()
            box.label(text="Offset:")
            row = box.row(align=True)
            row.prop(props, "offset_x")
            row.prop(props, "offset_y")
            row.prop(props, "offset_z")

        layout.separator()
        row = layout.row()
        row.operator("nea.autofit_collision", icon='MOD_PHYSICS')

        row = layout.row()
        icon = 'HIDE_OFF' if context.scene.nea_show_collision else 'HIDE_ON'
        row.operator("nea.toggle_collision_overlay", icon=icon,
                     depress=context.scene.nea_show_collision)


### Export UI

class MaybeExportMD5Mesh(bpy.types.Operator):
    '''Export All objects in the parent collection of the active object as an .MD5mesh.'''
    bl_idname = "export_scene.maybe_export_md5mesh"
    bl_label = 'Export MD5MESH'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects

        #try the collection of the active object first
        ao = bpy.context.active_object
                
        if ao:
            collection = ao.users_collection[0]
        else:
            msgLines = "Nothing selected to export.\n Please select an object in the collection you would like to export, and try again."
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}
            
        checkResult = is_export_go('meshes', collection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5mesh('INVOKE_DEFAULT')
        else:
                        
            msgLines = message(checkResult[0], checkResult[1])
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class MaybeExportMD5Anim(bpy.types.Operator):
    '''Export the action currently associated with the active object as an .MD5anim'''
    bl_idname = "export_scene.maybe_export_md5anim"
    bl_label = 'Export MD5ANIM'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects
        
        #try the collection of the active object 
        ao = bpy.context.active_object
                
        if ao:
            collection = ao.users_collection[0]
        else:
            msgLines = "Nothing selected to export.\n Please select an object in the collection you would like to export, and try again."
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}
            
        
        checkResult = is_export_go('anim', collection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5anim('INVOKE_DEFAULT')
        else:
                                    
            msgLines = message(checkResult[0], checkResult[1])
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class MaybeExportMD5Batch(bpy.types.Operator):
    '''Export all objects in the parent collection of the active object as an .MD5mesh. Export the active action or all actions as .MD5anim files'''
    bl_idname = "export_scene.maybe_export_md5batch"
    bl_label = 'Export MD5 Files'
    def invoke(self, context, event):
        global msgLines, prerequisites
        selection = context.selected_objects
        
        #try the collection of the active object 
        ao = bpy.context.active_object
                
        if ao:
            collection = ao.users_collection[0]
        else:
            msgLines = "Nothing selected to export.\n Please select an object in the collection you would like to export, and try again."
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            print(msgLines)
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}        
        
        
        checkResult = is_export_go('batch', collection)
        if checkResult[0] == 'ok':
            prerequisites = checkResult[-1]
            return bpy.ops.export_scene.md5batch('INVOKE_DEFAULT')
        else:
            msgLines = message(checkResult[0], checkResult[1])
            print(msgLines)
            bpy.ops.message.messagebox('INVOKE_DEFAULT', message = msgLines )
            self.report({'ERROR'}, msgLines)
            return {'CANCELLED'}

class ExportMD5Mesh(bpy.types.Operator, ExportHelper):
    '''Save an MD5 Mesh File'''
    global prerequisites
    bl_idname = "export_scene.md5mesh"
    bl_label = 'Export MD5MESH'
    bl_options = {'PRESET'}
    filename_ext = ".md5mesh"
    path_mode = path_reference_mode
    check_extension = True
    
    filter_glob : StringProperty(
        default="*.md5mesh",
        options={'HIDDEN'},
        )

    reorientDegrees : bpy.props.EnumProperty(
        items= (('0', '0 Degrees', 'Do not reorient'),
                ('90', '90 Degrees ( X to Y )', 'Rotate 90 degrees (e.g. reorient facing +X to facing +Y)'),
                ('-90', '-90 Degrees ( Y to X )', 'Rotate -90 degrees (e.g. reorient facing +Y to facing +X' ),
                ('180', '180 Degrees', 'Rotate 180 degrees')),
        name = "Reorient Model",
        description = "Degrees to rotate model during export.  Useful to reorient models to face Y axis if desired. 90 Degrees rotates clockwise from above. -90 Rotates counter-clockwise from above.",
        default = '0')

    scaleFactor : FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )

    fixWindings : BoolProperty(
            name="Fix tri indices for eye deform",
            description="Only select if having issues with materials flagged with eyeDeform",
            default=False
            )

    exportCollision : BoolProperty(
            name="Export bone collision (.md5collimesh)",
            description="Export per-bone collision data alongside the mesh",
            default=False
            )


    def invoke(self, context, event):
        ao = bpy.context.active_object
        collection = ao.users_collection[0]
        self.filepath = collection.name
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        global prerequisites
        #for cmdline export
        if not prerequisites:
            #'mesh' only returns the active object.
            checkResult = is_export_go('mesh',[])
            if checkResult[0] == 'ok':
                prerequisites = checkResult[-1]
            else:
                self.report({'ERROR'}, "\n" + message(checkResult[0],checkResult[1]) )
                return {'CANCELLED'}

        rotdeg = float(self.reorientDegrees)
        orientationTweak = mu.Matrix.Rotation(math.radians( rotdeg ),4,'Z')
        scaleTweak = mu.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak @ scaleTweak
        write_md5mesh(self.filepath, prerequisites, correctionMatrix, self.fixWindings)

        if self.exportCollision:
            bones = prerequisites[0]
            collimesh_path = os.path.splitext(self.filepath)[0] + '.md5collimesh'
            n = write_md5collimesh(collimesh_path, bones)
            self.report({'INFO'}, f"Exported {n} bone collision(s) to {collimesh_path}")

        return {'FINISHED'}

class ExportMD5Anim(bpy.types.Operator, ExportHelper):
    '''Save an MD5 Animation File'''
    global prerequisites
    bl_idname = "export_scene.md5anim"
    bl_label = 'Export MD5ANIM'
    bl_options = {'PRESET'}
    filename_ext = ".md5anim"
    path_mode = path_reference_mode
    check_extension = True
    
    filter_glob : StringProperty(
        default="*.md5anim",
        options={'HIDDEN'},
        )

    reorientDegrees : bpy.props.EnumProperty(
            items= (('0', '0 Degrees', 'Do not reorient'),
                    ('90', '90 Degrees ( X to Y )', 'Rotate 90 degrees (e.g. reorient facing +X to facing +Y)'),
                    ('-90', '-90 Degrees ( Y to X )', 'Rotate -90 degrees (e.g. reorient facing +Y to facing +X' ),
                    ('180', '180 Degrees', 'Rotate 180 degrees')),
            name = "Reorient Anim",
            description = "Degrees to rotate animation during export.  Useful to reorient animations to face Y axis if desired. 90 Degrees rotates clockwise from above. -90 Rotates counter-clockwise from above.",
            default = '0')

    scaleFactor : FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )

    previewKeysOnly : BoolProperty(
            name="Use timeline Start/End frames.",
            description="Only export frames indicated by timeline preview 'Start' and 'End' frames values - otherwise all action frames will be exported.",
            default=False,
            )

    check_extension = True
    
    def invoke(self, context, event):
        ao = bpy.context.active_object
        collection = ao.users_collection[0]
        collection_Prefix = "("+collection.name+")_"
        meshObjects = [o for o in bpy.data.collections[collection.name].objects
            if o.data in bpy.data.meshes[:] and o.find_armature()]
        armatures = [a.find_armature() for a in meshObjects]
        armature = armatures[0]
        action=armature.animation_data.action
        self.filepath = remove_prefix(action.name,collection_Prefix)
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
    
    def execute(self, context):
        global prerequisites
        
        #for cmdline export
        if not prerequisites:
            checkResult = is_export_go('anim', collection)
            if checkResult[0] == 'ok':
                prerequisites = checkResult[-1]
            else:
                self.report({'ERROR'}, "\n" + message(checkResult[0],checkResult[1]) )
                return {'CANCELLED'}

        #this should all be safe as it was just checked by MaybeExportMD5Anim/is_export_go
        ao = bpy.context.active_object
        collection = ao.users_collection[0]
        meshObjects = [o for o in bpy.data.collections[collection.name].objects
            if o.data in bpy.data.meshes[:] and o.find_armature()]
        armatures = [a.find_armature() for a in meshObjects]
        armature = armatures[0]
        
        action=armature.animation_data.action
        name = action.name
        frame_range = action.frame_range
                    
        rotdeg = float(self.reorientDegrees)
        orientationTweak = mu.Matrix.Rotation(math.radians( rotdeg ),4,'Z')
        scaleTweak = mu.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak @ scaleTweak
        
        write_md5anim(self.filepath, prerequisites, correctionMatrix, self.previewKeysOnly,frame_range)
        return {'FINISHED'}

class ExportMD5Batch(bpy.types.Operator, ExportHelper):
    '''Save MD5 Files'''
    global prerequisites
    bl_idname = "export_scene.md5batch"
    bl_label = 'Export MD5 Files'
    bl_options = {'PRESET'}
    filename_ext = ".md5mesh"
    path_mode = path_reference_mode
    check_extension = True
    
   
    filter_glob : StringProperty(
        default="*.md5mesh",
        options={'HIDDEN'},
        )
    exportAllAnims : BoolProperty(
            name="Export All Anims",
            description="""Export all actions associated with the object/collection as MD5 anims.
    All keyframes for each action will be exported.
    ( This exports all actions in the action editor that are prepended with the object/collection name. )""",
            default=False,
            )

    onlyPrepend : BoolProperty(
            name="Prepended action names only",
            description="Only export actions prepended with the collection name.",
            default=False,
            )

    stripPrepend : BoolProperty(
            name="Strip action name prepend",
            description="Strip the prepended collection name from exported action names.",
            default=True,
            )
    previewKeysOnly : BoolProperty(
            name="Use timeline Start/End frames",
            description="""Only export frames indicated by timeline preview 'Start' and 'End' frames values
    - otherwise all action frames will be exported.  Has no effect if 'Export All Anims' is selected.""",
            default=False,
            )

    reorientDegrees : bpy.props.EnumProperty(
        items= (('0', '0 Degrees', 'Do not reorient'),
                ('90', '90 Degrees ( X to Y )', 'Rotate 90 degrees (e.g. reorient facing +X to facing +Y)'),
                ('-90', '-90 Degrees ( Y to X )', 'Rotate -90 degrees (e.g. reorient facing +Y to facing +X' ),
                ('180', '180 Degrees', 'Rotate 180 degrees')),
        name = "Reorient Model/Anims",
        description = "Degrees to rotate model/anims during export.  Useful to reorient to face Y axis if desired. 90 Degrees rotates clockwise from above. -90 Rotates counter-clockwise from above.",
        default = '0')

    scaleFactor : FloatProperty(
            name="Scale",
            description="Scale all data",
            min=0.01, max=1000.0,
            soft_min=0.01,
            soft_max=1000.0,
            default=1.0,
            )

    fixWindings : BoolProperty(
    name="Fix tri indices for eye deform",
    description="Only select if having issues with materials flagged with eyeDeform",
    default=False
    )

    exportCollision : BoolProperty(
            name="Export bone collision (.md5collimesh)",
            description="Export per-bone collision data alongside the mesh",
            default=False
            )

    path_mode = path_reference_mode
    check_extension = True

    def invoke(self, context, event):
        ao = bpy.context.active_object
        collection = ao.users_collection[0]
        self.filepath = collection.name
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):

        rotdeg = float(self.reorientDegrees)
        orientationTweak = mu.Matrix.Rotation(math.radians( rotdeg ),4,'Z')
        scaleTweak = mu.Matrix.Scale(self.scaleFactor, 4)
        correctionMatrix = orientationTweak @ scaleTweak


        batch_directory = os.path.dirname(self.filepath)

        ao = bpy.context.active_object
        collection = ao.users_collection[0]
        meshObjects = [o for o in bpy.data.collections[collection.name].objects
            if o.data in bpy.data.meshes[:] and o.find_armature()]
        armatures = [a.find_armature() for a in meshObjects]
        armature = armatures[0]
        collection_Prefix = "("+collection.name+")_"

        #write the mesh
        write_md5mesh(self.filepath, prerequisites, correctionMatrix, self.fixWindings )

        if self.exportCollision:
            bones = prerequisites[0]
            collimesh_path = os.path.splitext(self.filepath)[0] + '.md5collimesh'
            n = write_md5collimesh(collimesh_path, bones)
            self.report({'INFO'}, f"Exported {n} bone collision(s) to {collimesh_path}")
                
        if not self.exportAllAnims:
            #write the active action
                        
            action=armature.animation_data.action
            name = action.name
            frame_range = action.frame_range
            
            if self.stripPrepend:
                name = remove_prefix(name,collection_Prefix)
                                     
            self.filepath = os.path.join( batch_directory, name )
            write_md5anim( self.filepath, prerequisites, correctionMatrix, self.previewKeysOnly, frame_range)
        else:
            
            #write all frames for all actions
            oldAction = armature.animation_data.action
            for exportAction in bpy.data.actions:
                name = exportAction.name
                frame_range = exportAction.frame_range
                print("Checking actino name " + name + " to see if in collection " + collection_Prefix)
                if name.startswith(collection_Prefix) or not self.onlyPrepend :
                    #export this action
                    armature.animation_data.action = exportAction                   
                    if self.stripPrepend:
                        name = remove_prefix(name,collection_Prefix)
                    
                    if not name.endswith(".md5anim"):
                        name = name + ".md5anim"
                    self.filepath = os.path.join( batch_directory, name )
                    print("Exporting animation "+self.filepath)
                    write_md5anim( self.filepath, prerequisites, correctionMatrix, False, frame_range)
            
            armature.animation_data.action = oldAction
            

        return {'FINISHED'}

class MessageBox(bpy.types.Operator):
    bl_idname = "message.messagebox"
    bl_label = ""
 
    message : bpy.props.StringProperty(
        name = "message",
        description = "message",
        default = ''
    )

    message2 : bpy.props.StringProperty(
        name = "message2",
        description = "message2",
        default = ''
    )

        
 
    def execute(self, context):
        self.report({'INFO'}, self.message)
        print(self.message)
        return {'FINISHED'}
 
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width = 600)
    
    def chunkstring(self,string, length):
        return (string[0+i:length+i] for i in range(0, len(string), length))

    def draw(self, context):
        
        lines = self.message.split('\n')
        for i in range(len(lines)):
            for chunk in self.chunkstring(lines[i], 80):
                self.message2 = chunk
                print (self.message2)
                self.layout.label(text=self.message2)
            
        
        self.layout.label(text="")
        
        
        
        
def menu_func_import_mesh(self, context):
    self.layout.operator(
        ImportMD5Mesh.bl_idname, text="MD5 Mesh (.md5mesh)")
def menu_func_import_anim(self, context):
    self.layout.operator(
        MaybeImportMD5Anim.bl_idname, text="MD5 Animation(s) (.md5anim)")

def menu_func_export_mesh(self, context):
    self.layout.operator(
        MaybeExportMD5Mesh.bl_idname, text="MD5 Mesh (.md5mesh)")
def menu_func_export_anim(self, context):
    self.layout.operator(
        MaybeExportMD5Anim.bl_idname, text="MD5 Animation (.md5anim)")
def menu_func_export_batch(self, context):
    self.layout.operator(
        MaybeExportMD5Batch.bl_idname, text="MD5 Mesh and Animation(s)")
def menu_func_export_scene(self, context):
    self.layout.operator(
        NEA_OT_ExportScene.bl_idname, text="NEA Scene (.neascene)")

classes = (
    # Addon preferences (must be first)
    NEA_AddonPreferences,
    # PropertyGroups (must be registered before panels that use them)
    NEA_BoneCollisionProps,
    NEA_SceneNodeProps,
    NEA_SceneSettings,
    NEA_ToolSettings,
    NEA_PolyformatProps,
    NEA_PtexconvProps,
    NEA_TextureFlagsProps,
    NEA_MaterialProps,
    NEA_LightProps,
    # Animated material
    NEA_AnimMatProps,
    # Import operators
    ImportMD5Mesh,
    MaybeImportMD5Anim,
    ImportMD5Anim,
    # Bone collection management
    MD5BonesAdd,
    MD5BonesRemove,
    MD5BonesReplace,
    MD5BonesClear,
    # Panels
    MD5Panel,
    BONE_PT_nea_collision,
    OBJECT_PT_nea_polyformat,
    OBJECT_PT_nea_scene_node,
    MATERIAL_PT_nea_ptexconv,
    SCENE_PT_nea_lights,
    SCENE_PT_nea_vram,
    VIEW3D_PT_nea_scene_export,
    VIEW3D_PT_nea_tools,
    MATERIAL_PT_nea_animmat,
    # Operators
    NEA_OT_ToggleCollisionOverlay,
    NEA_OT_AutoFitCollision,
    NEA_OT_ExportScene,
    NEA_OT_ToggleTriggerOverlay,
    NEA_OT_RunObj2dl,
    NEA_OT_RunMd5ToDsma,
    NEA_OT_RunPtexconv,
    NEA_OT_ExportAnimMat,
    # Export operators
    MaybeExportMD5Mesh,
    MaybeExportMD5Anim,
    MaybeExportMD5Batch,
    ExportMD5Mesh,
    ExportMD5Anim,
    ExportMD5Batch,
    MessageBox,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Bone.nea_collision = bpy.props.PointerProperty(
        type=NEA_BoneCollisionProps)
    bpy.types.Scene.nea_show_collision = BoolProperty(
        name="Show Collision Overlays",
        default=False)

    bpy.types.Object.nea_scene_node = bpy.props.PointerProperty(
        type=NEA_SceneNodeProps)
    bpy.types.Object.nea_polyformat = bpy.props.PointerProperty(
        type=NEA_PolyformatProps)
    bpy.types.Scene.nea_scene_settings = bpy.props.PointerProperty(
        type=NEA_SceneSettings)
    bpy.types.Scene.nea_show_triggers = BoolProperty(
        name="Show Trigger Overlays",
        default=False)
    bpy.types.Scene.nea_tool_settings = bpy.props.PointerProperty(
        type=NEA_ToolSettings)
    bpy.types.Scene.nea_light_props = bpy.props.PointerProperty(
        type=NEA_LightProps)
    bpy.types.Material.nea_ptexconv = bpy.props.PointerProperty(
        type=NEA_PtexconvProps)
    bpy.types.Material.nea_tex_flags = bpy.props.PointerProperty(
        type=NEA_TextureFlagsProps)
    bpy.types.Material.nea_mat_props = bpy.props.PointerProperty(
        type=NEA_MaterialProps)
    bpy.types.Material.nea_animmat = bpy.props.PointerProperty(
        type=NEA_AnimMatProps)

    # Register AnimMat visual preview handler
    global _animmat_handler
    if _animmat_handler is None:
        bpy.app.handlers.frame_change_post.append(_nea_animmat_frame_handler)
        _animmat_handler = _nea_animmat_frame_handler

    bpy.types.TOPBAR_MT_file_import.append(menu_func_import_mesh)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import_anim)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_mesh)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_anim)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_batch)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_scene)

def unregister():
    global _collision_draw_handler, _trigger_draw_handler
    if _collision_draw_handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(
            _collision_draw_handler, 'WINDOW')
        _collision_draw_handler = None
    if _trigger_draw_handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(
            _trigger_draw_handler, 'WINDOW')
        _trigger_draw_handler = None

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    del bpy.types.Bone.nea_collision
    del bpy.types.Scene.nea_show_collision
    del bpy.types.Object.nea_scene_node
    del bpy.types.Object.nea_polyformat
    del bpy.types.Scene.nea_scene_settings
    del bpy.types.Scene.nea_show_triggers
    del bpy.types.Scene.nea_tool_settings
    del bpy.types.Scene.nea_light_props
    del bpy.types.Material.nea_ptexconv
    del bpy.types.Material.nea_tex_flags
    del bpy.types.Material.nea_mat_props
    del bpy.types.Material.nea_animmat

    # Remove AnimMat visual preview handler
    global _animmat_handler
    if _animmat_handler is not None:
        if _animmat_handler in bpy.app.handlers.frame_change_post:
            bpy.app.handlers.frame_change_post.remove(_animmat_handler)
        _animmat_handler = None
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import_mesh)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import_anim)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_mesh)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_anim)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_batch)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_scene)
    del bpy.types.Scene.md5_bone_collection

if __name__ == "__main__":
    register()
