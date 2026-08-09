# This script is licensed as public domain.

import os
import struct
import bpy
import bpy_extras.io_utils

# import modules directly to ensure proper reloading
from . import blender_scene
from . import iqm_writer
from . import iqe_writer


def exportIQM(
    context,
    filename,
    filetype="IQM",
    usemesh=True,
    usemods=False,
    usebbox=True,
    usecol=False,
    scale=1.0,
    matfun=(lambda prefix, image: image),
    derigify=False,
    boneorder=None,
    namedmaterialmeshes=False,
):
    armature = blender_scene.findArmature(context)
    has_armature = armature is not None

    if has_armature:
        if derigify:
            bones = blender_scene.derigifyBones(context, armature, scale)
        else:
            bones = blender_scene.collectBones(context, armature, scale)
    else:
        bones = {}

    if boneorder:
        try:
            f = open(
                bpy_extras.io_utils.path_reference(
                    boneorder,
                    os.path.dirname(bpy.data.filepath),
                    os.path.dirname(filename),
                ),
                "r",
                encoding="utf-8",
            )
            names = [line.strip() for line in f.readlines()]
            f.close()
            names = [
                name for name in names if name in [bone.name for bone in bones.values()]
            ]
            if len(names) != len(bones):
                print(
                    "Bone order (%d) does not match skeleton (%d)"
                    % (len(names), len(bones))
                )
                return
            print("Reordering bones")
            for bone in bones.values():
                bone.index = names.index(bone.name)
        except:
            print("Failed opening bone order: %s" % boneorder)
            return

    if armature:
        oldpose = armature.data.pose_position
        blender_scene.poseArmature(context, armature, "REST")

    bonelist = sorted(bones.values(), key=lambda bone: bone.index)
    if usemesh:
        meshes = blender_scene.collectMeshes(
            context,
            bones,
            scale,
            matfun,
            usecol,
            usemods,
            filetype,
            namedmaterialmeshes,
        )
    else:
        meshes = []

    if armature:
        blender_scene.poseArmature(context, armature, oldpose)

    if has_armature:
        anims = blender_scene.collectAnimsAuto(context, armature, scale, bonelist)
    else:
        anims = []

    if filetype == "IQM":
        iqm = iqm_writer.IQMFile()
        iqm.addMeshes(meshes)
        iqm.addJoints(bonelist)
        iqm.addAnims(anims)
        iqm.calcFrameSize()
        iqm.calcNeighbors()

    if filename:
        try:
            if filetype == "IQM":
                file = open(filename, "wb")
            else:
                file = open(filename, "w")
        except:
            print("Failed writing to %s" % (filename))
            return

        if filetype == "IQM":
            iqm.export(file, usebbox)

        elif filetype == "IQE":
            iqe_writer.exportIQE(file, meshes, bonelist, anims)

        file.close()
        print("Saved %s file to %s" % (filetype, filename))
    else:
        print("No %s file was generated" % (filetype))


class ExportIQM(bpy.types.Operator, bpy_extras.io_utils.ExportHelper):
    """Export an Inter-Quake Model IQE or IQM file"""

    bl_idname = "export.iqm"
    bl_label = "Export IQE/IQM"
    bl_options = {"REGISTER"}

    filename_ext = ""

    file_format: bpy.props.EnumProperty(
        name="Format",
        description="Choose export format",
        items=(
            ("IQE", "IQE (.iqe)", "Export as text IQE"),
            ("IQM", "IQM (.iqm)", "Export as binary IQM"),
        ),
        default="IQM",
    )

    usemesh: bpy.props.BoolProperty(
        name="Meshes", description="Generate meshes", default=True
    )
    usemods: bpy.props.BoolProperty(
        name="Modifiers", description="Apply modifiers", default=True
    )
    usebbox: bpy.props.BoolProperty(
        name="Bounding boxes", description="Generate bounding boxes", default=True
    )
    usecol: bpy.props.BoolProperty(
        name="Vertex colors", description="Export vertex colors", default=False
    )
    usescale: bpy.props.FloatProperty(
        name="Scale",
        description="Scale of exported model",
        default=1.0,
        min=0.0,
        step=50,
        precision=2,
    )
    matfmt: bpy.props.EnumProperty(
        name="Materials",
        description="Material name format",
        items=[
            ("m+i-e", "material+image-ext", ""),
            ("m", "material", ""),
            ("i", "image", ""),
        ],
        default="m+i-e",
    )
    derigify: bpy.props.BoolProperty(
        name="De-rigify",
        description="Export only deformation bones from rigify",
        default=False,
    )
    namedmaterialmeshes: bpy.props.BoolProperty(
        name="Named material meshes",
        description="Append material names to individual exported mesh objects, for meshes with multiple materials",
        default=False,
    )
    boneorder: bpy.props.StringProperty(
        name="Bone order",
        description="Override ordering of bones",
        subtype="FILE_NAME",
        default="",
    )

    def execute(self, context):
        if self.properties.matfmt == "m+i-e":
            matfun = lambda prefix, image: prefix + os.path.splitext(image)[0]
        elif self.properties.matfmt == "m":
            matfun = lambda prefix, image: prefix
        else:
            matfun = lambda prefix, image: image

        exportIQM(
            context,
            self.properties.filepath,
            self.properties.file_format,
            self.properties.usemesh,
            self.properties.usemods,
            self.properties.usebbox,
            self.properties.usecol,
            self.properties.usescale,
            matfun,
            self.properties.derigify,
            self.properties.boneorder,
            self.properties.namedmaterialmeshes,
        )
        return {"FINISHED"}

    def check(self, context):

        ext = ".iqm" if self.file_format == "IQM" else ".iqe"
        filepath = self.filepath

        # remove existing extensions to prevent stacking
        if filepath.endswith((".iqm", ".iqe")):
            filepath = filepath[:-4]

        filepath = bpy.path.ensure_ext(filepath, ext)

        if filepath != self.filepath:
            self.filepath = filepath
            return True

        return False


def menu_func(self, context):
    self.layout.operator(ExportIQM.bl_idname, text="Inter-Quake Model (.iqe, .iqm)")


def register():
    bpy.utils.register_class(ExportIQM)
    bpy.types.TOPBAR_MT_file_export.append(menu_func)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func)
    bpy.utils.unregister_class(ExportIQM)


if __name__ == "__main__":
    register()
