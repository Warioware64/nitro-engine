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

classes = (
    NEA_BoneCollisionProps,
    ImportMD5Mesh,
    MaybeImportMD5Anim,
    ImportMD5Anim,
    MD5BonesAdd,
    MD5BonesRemove,
    MD5BonesReplace,
    MD5BonesClear,
    MD5Panel,
    BONE_PT_nea_collision,
    NEA_OT_ToggleCollisionOverlay,
    NEA_OT_AutoFitCollision,
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

    bpy.types.TOPBAR_MT_file_import.append(menu_func_import_mesh)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import_anim)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_mesh)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_anim)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export_batch)

def unregister():
    global _collision_draw_handler
    if _collision_draw_handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(
            _collision_draw_handler, 'WINDOW')
        _collision_draw_handler = None

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    del bpy.types.Bone.nea_collision
    del bpy.types.Scene.nea_show_collision
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import_mesh)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import_anim)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_mesh)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_anim)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export_batch)
    del bpy.types.Scene.md5_bone_collection

if __name__ == "__main__":
    register()
