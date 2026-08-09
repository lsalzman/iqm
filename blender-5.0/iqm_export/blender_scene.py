import os
import mathutils

from .skeleton import Bone, Animation
from .geometry import Mesh, Vertex


# removed useskel -> added findArmature
def findArmature(context):
    armature = None
    for obj in context.selected_objects:
        if obj.type == "ARMATURE":
            armature = obj
            break
    if not armature:
        for obj in context.selected_objects:
            if obj.type == "MESH":
                armature = obj.find_armature()
                if armature:
                    break
    return armature


def poseArmature(context, armature, pose):
    if armature:
        armature.data.pose_position = pose
        armature.data.update_tag()
        context.scene.frame_set(context.scene.frame_current)


def derigifyBones(context, armature, scale):
    data = armature.data

    defnames = []
    orgbones = {}
    defbones = {}
    org2defs = {}
    def2org = {}
    defparent = {}
    defchildren = {}
    for bone in data.bones.values():
        if bone.name.startswith("ORG-"):
            orgbones[bone.name[4:]] = bone
            org2defs[bone.name[4:]] = []
        elif bone.name.startswith("DEF-"):
            defnames.append(bone.name[4:])
            defbones[bone.name[4:]] = bone
            defchildren[bone.name[4:]] = []
    for name, bone in defbones.items():
        orgname = name
        orgbone = orgbones.get(orgname)
        splitname = -1
        if not orgbone:
            splitname = name.rfind(".")
            suffix = ""
            if splitname >= 0 and name[splitname + 1 :] in ["l", "r", "L", "R"]:
                suffix = name[splitname:]
                splitname = name.rfind(".", 0, splitname)
            if splitname >= 0 and name[splitname + 1 : splitname + 2].isdigit():
                orgname = name[:splitname] + suffix
                orgbone = orgbones.get(orgname)
        org2defs[orgname].append(name)
        def2org[name] = orgname
    for defs in org2defs.values():
        defs.sort()
    for name in defnames:
        bone = defbones[name]
        orgname = def2org[name]
        orgbone = orgbones.get(orgname)
        defs = org2defs[orgname]
        if orgbone:
            i = defs.index(name)
            if i == 0:
                orgparent = orgbone.parent
                if orgparent and orgparent.name.startswith("ORG-"):
                    orgpname = orgparent.name[4:]
                    defparent[name] = org2defs[orgpname][-1]
            else:
                defparent[name] = defs[i - 1]
        if name in defparent:
            defchildren[defparent[name]].append(name)

    bones = {}
    worldmatrix = armature.matrix_world
    worklist = [bone for bone in defnames if bone not in defparent]
    for index, bname in enumerate(worklist):
        bone = defbones[bname]
        bonematrix = worldmatrix @ bone.matrix_local
        if scale != 1.0:
            bonematrix.translation *= scale
        bones[bone.name] = Bone(
            bname,
            bone.name,
            index,
            bname in defparent and bones.get(defbones[defparent[bname]].name),
            bonematrix,
        )
        worklist.extend(defchildren[bname])
    print("De-rigified %d bones" % len(worklist))
    return bones


def collectBones(context, armature, scale):
    data = armature.data
    bones = {}
    worldmatrix = armature.matrix_world
    worklist = [bone for bone in data.bones.values() if not bone.parent]
    for index, bone in enumerate(worklist):
        bonematrix = worldmatrix @ bone.matrix_local
        if scale != 1.0:
            bonematrix.translation *= scale
        bones[bone.name] = Bone(
            bone.name,
            bone.name,
            index,
            bone.parent and bones.get(bone.parent.name),
            bonematrix,
        )
        for child in bone.children:
            if child not in worklist:
                worklist.append(child)
    print("Collected %d bones" % len(worklist))
    return bones


def collectAnim(
    context, armature, scale, bones, action, startframe=None, endframe=None
):
    if startframe is None or endframe is None:
        startframe, endframe = action.frame_range
        startframe = int(startframe)
        endframe = int(endframe)
    print('Exporting action "%s" frames %d-%d' % (action.name, startframe, endframe))
    scene = context.scene
    worldmatrix = armature.matrix_world

    armature.animation_data.action = action
    # Blender 4.4+ requires explicit slot binding; assignment alone
    # may leave the armature unanimated without raising an error.
    anim_data = armature.animation_data
    if hasattr(anim_data, "action_suitable_slots") and anim_data.action_suitable_slots:
        anim_data.action_slot = anim_data.action_suitable_slots[0]

    outdata = []
    for time in range(startframe, endframe + 1):
        scene.frame_set(time)
        pose = armature.pose
        outframe = []
        for bone in bones:
            posematrix = pose.bones[bone.origname].matrix
            if bone.parent:
                posematrix = (
                    pose.bones[bone.parent.origname].matrix.inverted_safe() @ posematrix
                )
            else:
                posematrix = worldmatrix @ posematrix
            if scale != 1.0:
                posematrix.translation *= scale
            loc = posematrix.to_translation()
            quat = posematrix.to_3x3().inverted_safe().transposed().to_quaternion()
            quat.normalize()
            if quat.w > 0:
                quat.negate()
            pscale = posematrix.to_scale()
            pscale.x = round(pscale.x * 0x10000) / 0x10000
            pscale.y = round(pscale.y * 0x10000) / 0x10000
            pscale.z = round(pscale.z * 0x10000) / 0x10000
            outframe.append((loc, quat, pscale, posematrix))
        outdata.append(outframe)
    return outdata


# find animations from armature
def collectAnimsAuto(context, armature, scale, bones):
    if not armature.animation_data:
        print("Armature has no animation data")
        return []

    anims = []
    scene = context.scene
    fps = float(scene.render.fps)

    # store initial states to restore later
    oldaction = armature.animation_data.action
    oldframe = scene.frame_current

    processed_actions = set()

    # parse NLA Tracks and Strips assigned to this specific armature
    if armature.animation_data.nla_tracks:
        for track in armature.animation_data.nla_tracks:
            for strip in track.strips:
                if strip.action and strip.action.name not in processed_actions:
                    action = strip.action

                    # verify active frame range flags
                    if getattr(action, "use_frame_range", False):
                        start = int(action.frame_start)
                        end = int(action.frame_end)
                    else:
                        # fallback to NLA track limits
                        start = int(strip.action_frame_start)
                        end = int(strip.action_frame_end)

                    framedata = collectAnim(
                        context, armature, scale, bones, action, start, end
                    )
                    anims.append(Animation(action.name, framedata, fps, 0))
                    processed_actions.add(action.name)

    # parse the currently active action if not processed via NLA
    if armature.animation_data.action:
        action = armature.animation_data.action
        if action.name not in processed_actions:
            if getattr(action, "use_frame_range", False):
                start = int(action.frame_start)
                end = int(action.frame_end)
            else:
                # fallback to absolute action range
                start, end = [int(f) for f in action.frame_range]

            framedata = collectAnim(context, armature, scale, bones, action, start, end)
            anims.append(Animation(action.name, framedata, fps, 0))
            processed_actions.add(action.name)

    # restore initial states
    armature.animation_data.action = oldaction
    scene.frame_set(oldframe)

    return anims


def collectMeshes(
    context,
    bones,
    scale,
    matfun,
    usecol=False,
    usemods=False,
    filetype="IQM",
    namedmaterialmeshes=False,
):
    vertwarn = []
    objs = context.selected_objects
    meshes = []

    # check Depsgraph once
    dg = context.evaluated_depsgraph_get()

    for obj in objs:
        if obj.type == "MESH":
            obj_eval = None

            if usemods:
                # get the obj and creates temp mesh
                obj_eval = obj.evaluated_get(dg)
                data = obj_eval.to_mesh(preserve_all_data_layers=True, depsgraph=dg)

            else:
                # get original mesh
                data = obj.data

            if not data.polygons:
                if obj_eval:
                    obj_eval.to_mesh_clear()
                continue

            coordmatrix = obj.matrix_world
            normalmatrix = coordmatrix.inverted_safe().transposed()

            if scale != 1.0:
                coordmatrix = mathutils.Matrix.Scale(scale, 4) @ coordmatrix

            materials = {}
            matnames = {}
            groups = obj.vertex_groups
            uvlayer = data.uv_layers.active and data.uv_layers.active.data
            colors = None
            alpha = None

            if usecol:
                if data.color_attributes:
                    if data.color_attributes.active_color.name.startswith("alpha"):
                        alpha = data.color_attributes.active_color.data
                    else:
                        colors = data.color_attributes.active_color.data

                # potencial legacy / dead code
                # for layer in data.vertex_colors:
                #     if layer.name.startswith("alpha"):
                #         if not alpha:
                #             alpha = layer.data
                #     elif not colors:
                #         colors = layer.data

            if data.materials:
                for idx, mat in enumerate(data.materials):
                    if not mat:
                        continue

                    matprefix = mat.name or ""
                    matimage = ""
                    if mat.node_tree:
                        for n in mat.node_tree.nodes:
                            if n.type == "TEX_IMAGE" and n.image:
                                matimage = os.path.basename(n.image.filepath)
                                break
                    matnames[idx] = matfun(matprefix, matimage)
            for face in data.polygons:
                if len(face.vertices) < 3:
                    continue

                if all(
                    [
                        data.vertices[i].co == data.vertices[face.vertices[0]].co
                        for i in face.vertices[1:]
                    ]
                ):
                    continue

                matindex = face.material_index
                try:
                    mesh = materials[obj.name, matindex]
                except:
                    matname = matnames.get(matindex, "")
                    if namedmaterialmeshes and matname:
                        newmeshname = f"{obj.name}_{matname}"
                    else:
                        newmeshname = obj.name
                    mesh = Mesh(newmeshname, matname, data.vertices)
                    materials[obj.name, matindex] = mesh

                verts = mesh.verts
                vertmap = mesh.vertmap
                faceverts = []
                for loopidx in face.loop_indices:
                    loop = data.loops[loopidx]
                    v = data.vertices[loop.vertex_index]
                    vertco = coordmatrix @ v.co

                    # 4.1 - 5.0+ adaptation
                    # Mesh.corner_normals unifies flat/smooth/custom-split normal resolution;
                    # no need to branch on face.use_smooth.
                    vertno = normalmatrix @ mathutils.Vector(
                        data.corner_normals[loopidx].vector
                    )
                    vertno.normalize()

                    # flip V axis of texture space
                    if uvlayer:
                        uv = uvlayer[loopidx].uv
                        vertuv = mathutils.Vector((uv[0], 1.0 - uv[1]))
                    else:
                        vertuv = mathutils.Vector((0.0, 0.0))

                    if colors:
                        vertcol = colors[loopidx].color
                        vertcol = (
                            int(round(vertcol[0] * 255.0)),
                            int(round(vertcol[1] * 255.0)),
                            int(round(vertcol[2] * 255.0)),
                            255,
                        )
                    else:
                        vertcol = None

                    if alpha:
                        vertalpha = alpha[loopidx].color
                        if vertcol:
                            vertcol = (
                                vertcol[0],
                                vertcol[1],
                                vertcol[2],
                                int(round(vertalpha[0] * 255.0)),
                            )
                        else:
                            vertcol = (255, 255, 255, int(round(vertalpha[0] * 255.0)))

                    vertweights = []

                    if bones:
                        for g in v.groups:
                            try:
                                vertweights.append(
                                    (g.weight, bones[groups[g.group].name].index)
                                )
                            except:
                                if (groups[g.group].name, mesh.name) not in vertwarn:
                                    vertwarn.append((groups[g.group].name, mesh.name))
                                    print(
                                        "Vertex depends on non-existent bone: %s in mesh: %s"
                                        % (groups[g.group].name, mesh.name)
                                    )

                    vertkey = Vertex(
                        v.index, vertco, vertno, vertuv, vertweights, vertcol
                    )
                    if filetype == "IQM":
                        vertkey.normalizeWeights()
                    if not verts[v.index]:
                        verts[v.index] = vertkey
                        faceverts.append(vertkey)
                    elif verts[v.index] == vertkey:
                        faceverts.append(verts[v.index])
                    else:
                        try:
                            vertindex = vertmap[vertkey]
                            faceverts.append(verts[vertindex])
                        except:
                            vertindex = len(verts)
                            vertmap[vertkey] = vertindex
                            verts.append(vertkey)
                            faceverts.append(vertkey)

                # Quake winding is reversed
                for i in range(2, len(faceverts)):
                    mesh.tris.append((faceverts[0], faceverts[i], faceverts[i - 1]))

            # Export materials in the order of their assigned index
            # FIXED: is now possible to export objects without material
            max_mat_index = max(len(matnames), 1)
            for i in range(max_mat_index):
                mesh = materials.get((obj.name, i))
                if mesh:
                    meshes.append(mesh)

            # RAM clearance for each obj
            if obj_eval:
                obj_eval.to_mesh_clear()

    for mesh in meshes:
        mesh.optimize()
        if filetype == "IQM":
            mesh.calcTangents()
        print(
            "%s %s: generated %d triangles" % (mesh.name, mesh.material, len(mesh.tris))
        )

    return meshes
