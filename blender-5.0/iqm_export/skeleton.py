import struct
import mathutils
import math

from .iqm_format import IQM_BOUNDS


class Bone:
    def __init__(self, name, origname, index, parent, matrix):
        self.name = name
        self.origname = origname
        self.index = index
        self.parent = parent
        self.matrix = matrix
        self.localmatrix = matrix
        if self.parent:
            self.localmatrix = parent.matrix.inverted_safe() @ self.localmatrix
        self.numchannels = 0
        self.channelmask = 0
        self.channeloffsets = [
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
            1.0e10,
        ]
        self.channelscales = [
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
            -1.0e10,
        ]

    def jointData(self, iqm):
        if self.parent:
            parent = self.parent.index
        else:
            parent = -1
        pos = self.localmatrix.to_translation()
        orient = self.localmatrix.to_quaternion()
        orient.normalize()
        if orient.w > 0:
            orient.negate()
        scale = self.localmatrix.to_scale()
        scale.x = round(scale.x * 0x10000) / 0x10000
        scale.y = round(scale.y * 0x10000) / 0x10000
        scale.z = round(scale.z * 0x10000) / 0x10000
        return [
            iqm.addText(self.name),
            parent,
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
        ]

    def poseData(self, iqm):
        if self.parent:
            parent = self.parent.index
        else:
            parent = -1
        return [parent, self.channelmask] + self.channeloffsets + self.channelscales

    def calcChannelMask(self):
        for i in range(0, 10):
            self.channelscales[i] -= self.channeloffsets[i]
            if self.channelscales[i] >= 1.0e-10:
                self.numchannels += 1
                self.channelmask |= 1 << i
                self.channelscales[i] /= 0xFFFF
            else:
                self.channelscales[i] = 0.0
        return self.numchannels


class Animation:
    def __init__(self, name, frames, fps=0.0, flags=0):
        self.name = name
        self.frames = frames
        self.fps = fps
        self.flags = flags

    def calcFrameLimits(self, bones):
        for frame in self.frames:
            for i, bone in enumerate(bones):
                loc, quat, scale, mat = frame[i]
                bone.channeloffsets[0] = min(bone.channeloffsets[0], loc.x)
                bone.channeloffsets[1] = min(bone.channeloffsets[1], loc.y)
                bone.channeloffsets[2] = min(bone.channeloffsets[2], loc.z)
                bone.channeloffsets[3] = min(bone.channeloffsets[3], quat.x)
                bone.channeloffsets[4] = min(bone.channeloffsets[4], quat.y)
                bone.channeloffsets[5] = min(bone.channeloffsets[5], quat.z)
                bone.channeloffsets[6] = min(bone.channeloffsets[6], quat.w)
                bone.channeloffsets[7] = min(bone.channeloffsets[7], scale.x)
                bone.channeloffsets[8] = min(bone.channeloffsets[8], scale.y)
                bone.channeloffsets[9] = min(bone.channeloffsets[9], scale.z)
                bone.channelscales[0] = max(bone.channelscales[0], loc.x)
                bone.channelscales[1] = max(bone.channelscales[1], loc.y)
                bone.channelscales[2] = max(bone.channelscales[2], loc.z)
                bone.channelscales[3] = max(bone.channelscales[3], quat.x)
                bone.channelscales[4] = max(bone.channelscales[4], quat.y)
                bone.channelscales[5] = max(bone.channelscales[5], quat.z)
                bone.channelscales[6] = max(bone.channelscales[6], quat.w)
                bone.channelscales[7] = max(bone.channelscales[7], scale.x)
                bone.channelscales[8] = max(bone.channelscales[8], scale.y)
                bone.channelscales[9] = max(bone.channelscales[9], scale.z)

    def animData(self, iqm):
        return [
            iqm.addText(self.name),
            self.firstframe,
            len(self.frames),
            self.fps,
            self.flags,
        ]

    def frameData(self, bones):
        data = b""
        for frame in self.frames:
            for i, bone in enumerate(bones):
                loc, quat, scale, mat = frame[i]
                if (bone.channelmask & 0x7F) == 0x7F:
                    lx = int(
                        round((loc.x - bone.channeloffsets[0]) / bone.channelscales[0])
                    )
                    ly = int(
                        round((loc.y - bone.channeloffsets[1]) / bone.channelscales[1])
                    )
                    lz = int(
                        round((loc.z - bone.channeloffsets[2]) / bone.channelscales[2])
                    )
                    qx = int(
                        round((quat.x - bone.channeloffsets[3]) / bone.channelscales[3])
                    )
                    qy = int(
                        round((quat.y - bone.channeloffsets[4]) / bone.channelscales[4])
                    )
                    qz = int(
                        round((quat.z - bone.channeloffsets[5]) / bone.channelscales[5])
                    )
                    qw = int(
                        round((quat.w - bone.channeloffsets[6]) / bone.channelscales[6])
                    )
                    data += struct.pack("<7H", lx, ly, lz, qx, qy, qz, qw)
                else:
                    if bone.channelmask & 1:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (loc.x - bone.channeloffsets[0])
                                    / bone.channelscales[0]
                                )
                            ),
                        )
                    if bone.channelmask & 2:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (loc.y - bone.channeloffsets[1])
                                    / bone.channelscales[1]
                                )
                            ),
                        )
                    if bone.channelmask & 4:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (loc.z - bone.channeloffsets[2])
                                    / bone.channelscales[2]
                                )
                            ),
                        )
                    if bone.channelmask & 8:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (quat.x - bone.channeloffsets[3])
                                    / bone.channelscales[3]
                                )
                            ),
                        )
                    if bone.channelmask & 16:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (quat.y - bone.channeloffsets[4])
                                    / bone.channelscales[4]
                                )
                            ),
                        )
                    if bone.channelmask & 32:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (quat.z - bone.channeloffsets[5])
                                    / bone.channelscales[5]
                                )
                            ),
                        )
                    if bone.channelmask & 64:
                        data += struct.pack(
                            "<H",
                            int(
                                round(
                                    (quat.w - bone.channeloffsets[6])
                                    / bone.channelscales[6]
                                )
                            ),
                        )
                if bone.channelmask & 128:
                    data += struct.pack(
                        "<H",
                        int(
                            round(
                                (scale.x - bone.channeloffsets[7])
                                / bone.channelscales[7]
                            )
                        ),
                    )
                if bone.channelmask & 256:
                    data += struct.pack(
                        "<H",
                        int(
                            round(
                                (scale.y - bone.channeloffsets[8])
                                / bone.channelscales[8]
                            )
                        ),
                    )
                if bone.channelmask & 512:
                    data += struct.pack(
                        "<H",
                        int(
                            round(
                                (scale.z - bone.channeloffsets[9])
                                / bone.channelscales[9]
                            )
                        ),
                    )
        return data

    def frameBoundsData(self, bones, meshes, frame, invbase):
        bbmin = bbmax = None
        xyradius = 0.0
        radius = 0.0
        transforms = []
        for i, bone in enumerate(bones):
            loc, quat, scale, mat = frame[i]
            if bone.parent:
                mat = transforms[bone.parent.index] @ mat
            transforms.append(mat)
        for i, mat in enumerate(transforms):
            transforms[i] = mat @ invbase[i]
        for mesh in meshes:
            for v in mesh.verts:
                pos = mathutils.Vector((0.0, 0.0, 0.0))
                for weight, bone in v.weights:
                    if weight > 0:
                        pos += (transforms[bone] @ v.coord) * (weight / 255.0)
                if bbmin:
                    bbmin.x = min(bbmin.x, pos.x)
                    bbmin.y = min(bbmin.y, pos.y)
                    bbmin.z = min(bbmin.z, pos.z)
                    bbmax.x = max(bbmax.x, pos.x)
                    bbmax.y = max(bbmax.y, pos.y)
                    bbmax.z = max(bbmax.z, pos.z)
                else:
                    bbmin = pos.copy()
                    bbmax = pos.copy()
                pradius = pos.x * pos.x + pos.y * pos.y
                if pradius > xyradius:
                    xyradius = pradius
                pradius += pos.z * pos.z
                if pradius > radius:
                    radius = pradius
        if bbmin:
            xyradius = math.sqrt(xyradius)
            radius = math.sqrt(radius)
        else:
            bbmin = bbmax = mathutils.Vector((0.0, 0.0, 0.0))
        return IQM_BOUNDS.pack(
            bbmin.x, bbmin.y, bbmin.z, bbmax.x, bbmax.y, bbmax.z, xyradius, radius
        )

    def boundsData(self, bones, meshes):
        invbase = []
        for bone in bones:
            invbase.append(bone.matrix.inverted_safe())
        data = b""
        for i, frame in enumerate(self.frames):
            print("Calculating bounding box for %s:%d" % (self.name, i))
            data += self.frameBoundsData(bones, meshes, frame, invbase)
        return data
