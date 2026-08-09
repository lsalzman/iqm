from .iqm_format import IQM_LOOP


def exportIQE(file, meshes, bones, anims):
    file.write("# Inter-Quake Export\n\n")

    for bone in bones:
        if bone.parent:
            parent = bone.parent.index
        else:
            parent = -1
        file.write('joint "%s" %d\n' % (bone.name, parent))
        if meshes:
            pos = bone.localmatrix.to_translation()
            orient = bone.localmatrix.to_quaternion()
            orient.normalize()
            if orient.w > 0:
                orient.negate()
            scale = bone.localmatrix.to_scale()
            scale.x = round(scale.x * 0x10000) / 0x10000
            scale.y = round(scale.y * 0x10000) / 0x10000
            scale.z = round(scale.z * 0x10000) / 0x10000
            if scale.x == 1.0 and scale.y == 1.0 and scale.z == 1.0:
                file.write(
                    "\tpq %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n"
                    % (pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w)
                )
            else:
                file.write(
                    "\tpq %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n"
                    % (
                        pos.x,
                        pos.y,
                        pos.z,
                        orient.x,
                        orient.y,
                        orient.z,
                        orient.w,
                        scale.x,
                        scale.y,
                        scale.z,
                    )
                )

    hascolors = any(mesh.verts and mesh.verts[0].color for mesh in meshes)
    for mesh in meshes:
        file.write('\nmesh "%s"\n\tmaterial "%s"\n\n' % (mesh.name, mesh.material))
        for v in mesh.verts:
            file.write(
                "vp %.8f %.8f %.8f\n\tvt %.8f %.8f\n\tvn %.8f %.8f %.8f\n"
                % (
                    v.coord.x,
                    v.coord.y,
                    v.coord.z,
                    v.uv.x,
                    v.uv.y,
                    v.normal.x,
                    v.normal.y,
                    v.normal.z,
                )
            )
            if bones:
                weights = "\tvb"
                for weight in v.weights:
                    weights += " %d %.8f" % (weight[1], weight[0])
                file.write(weights + "\n")
            if hascolors:
                if v.color:
                    file.write(
                        "\tvc %.8f %.8f %.8f %.8f\n"
                        % (
                            v.color[0] / 255.0,
                            v.color[1] / 255.0,
                            v.color[2] / 255.0,
                            v.color[3] / 255.0,
                        )
                    )
                else:
                    # file.write("\tvc 0 0 0 1\n")
                    file.write("\tvc 0.8 0.8 0.8 1.0\n")

        file.write("\n")
        for v0, v1, v2 in mesh.tris:
            file.write("fm %d %d %d\n" % (v0.index, v1.index, v2.index))

    for anim in anims:
        file.write('\nanimation "%s"\n\tframerate %.8f\n' % (anim.name, anim.fps))
        if anim.flags & IQM_LOOP:
            file.write("\tloop\n")
        for frame in anim.frames:
            file.write("\nframe\n")
            for pos, orient, scale, mat in frame:
                if scale.x == 1.0 and scale.y == 1.0 and scale.z == 1.0:
                    file.write(
                        "pq %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n"
                        % (pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w)
                    )
                else:
                    file.write(
                        "pq %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n"
                        % (
                            pos.x,
                            pos.y,
                            pos.z,
                            orient.x,
                            orient.y,
                            orient.z,
                            orient.w,
                            scale.x,
                            scale.y,
                            scale.z,
                        )
                    )

    file.write("\n")
