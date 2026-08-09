import struct

from .iqm_format import (
    IQM_VERTEXARRAY,
    IQM_POSITION,
    IQM_FLOAT,
    IQM_TEXCOORD,
    IQM_NORMAL,
    IQM_TANGENT,
    IQM_BLENDINDEXES,
    IQM_BLENDWEIGHTS,
    IQM_COLOR,
    IQM_TRIANGLE,
    IQM_POSE,
    IQM_UBYTE,
    IQM_JOINT,
    IQM_BOUNDS,
    IQM_MESH,
    IQM_HEADER,
    IQM_ANIMATION,
)


class IQMFile:
    def __init__(self):
        self.textoffsets = {}
        self.textdata = b""
        self.meshes = []
        self.meshdata = []
        self.numverts = 0
        self.numtris = 0
        self.joints = []
        self.jointdata = []
        self.numframes = 0
        self.framesize = 0
        self.anims = []
        self.posedata = []
        self.animdata = []
        self.framedata = []
        self.vertdata = []

    def addText(self, str):
        if not self.textdata:
            self.textdata += b"\x00"
            self.textoffsets[""] = 0
        try:
            return self.textoffsets[str]
        except:
            offset = len(self.textdata)
            self.textoffsets[str] = offset
            self.textdata += bytes(str, encoding="utf8") + b"\x00"
            return offset

    def addJoints(self, bones):
        for bone in bones:
            self.joints.append(bone)
            if self.meshes:
                self.jointdata.append(bone.jointData(self))

    def addMeshes(self, meshes):
        self.meshes += meshes
        for mesh in meshes:
            mesh.firstvert = self.numverts
            mesh.firsttri = self.numtris
            self.meshdata.append(mesh.meshData(self))
            self.numverts += len(mesh.verts)
            self.numtris += len(mesh.tris)

    def addAnims(self, anims):
        self.anims += anims
        for anim in anims:
            anim.firstframe = self.numframes
            self.animdata.append(anim.animData(self))
            self.numframes += len(anim.frames)

    def calcFrameSize(self):
        for anim in self.anims:
            anim.calcFrameLimits(self.joints)
        self.framesize = 0
        for joint in self.joints:
            self.framesize += joint.calcChannelMask()
        for joint in self.joints:
            if self.anims:
                self.posedata.append(joint.poseData(self))
        print("Exporting %d frames of size %d" % (self.numframes, self.framesize))

    def writeVerts(self, file, offset):
        if self.numverts <= 0:
            return

        file.write(IQM_VERTEXARRAY.pack(IQM_POSITION, 0, IQM_FLOAT, 3, offset))
        offset += self.numverts * struct.calcsize("<3f")
        file.write(IQM_VERTEXARRAY.pack(IQM_TEXCOORD, 0, IQM_FLOAT, 2, offset))
        offset += self.numverts * struct.calcsize("<2f")
        file.write(IQM_VERTEXARRAY.pack(IQM_NORMAL, 0, IQM_FLOAT, 3, offset))
        offset += self.numverts * struct.calcsize("<3f")
        file.write(IQM_VERTEXARRAY.pack(IQM_TANGENT, 0, IQM_FLOAT, 4, offset))
        offset += self.numverts * struct.calcsize("<4f")
        if self.joints:
            file.write(IQM_VERTEXARRAY.pack(IQM_BLENDINDEXES, 0, IQM_UBYTE, 4, offset))
            offset += self.numverts * struct.calcsize("<4B")
            file.write(IQM_VERTEXARRAY.pack(IQM_BLENDWEIGHTS, 0, IQM_UBYTE, 4, offset))
            offset += self.numverts * struct.calcsize("<4B")
        hascolors = any(mesh.verts and mesh.verts[0].color for mesh in self.meshes)
        if hascolors:
            file.write(IQM_VERTEXARRAY.pack(IQM_COLOR, 0, IQM_UBYTE, 4, offset))
            offset += self.numverts * struct.calcsize("<4B")

        for mesh in self.meshes:
            for v in mesh.verts:
                file.write(struct.pack("<3f", *v.coord))
        for mesh in self.meshes:
            for v in mesh.verts:
                file.write(struct.pack("<2f", *v.uv))
        for mesh in self.meshes:
            for v in mesh.verts:
                file.write(struct.pack("<3f", *v.normal))
        for mesh in self.meshes:
            for v in mesh.verts:
                file.write(
                    struct.pack(
                        "<4f", v.tangent.x, v.tangent.y, v.tangent.z, v.bitangent
                    )
                )
        if self.joints:
            for mesh in self.meshes:
                for v in mesh.verts:
                    file.write(
                        struct.pack(
                            "<4B",
                            v.weights[0][1],
                            v.weights[1][1],
                            v.weights[2][1],
                            v.weights[3][1],
                        )
                    )
            for mesh in self.meshes:
                for v in mesh.verts:
                    file.write(
                        struct.pack(
                            "<4B",
                            v.weights[0][0],
                            v.weights[1][0],
                            v.weights[2][0],
                            v.weights[3][0],
                        )
                    )
        if hascolors:
            for mesh in self.meshes:
                for v in mesh.verts:
                    if v.color:
                        file.write(
                            struct.pack(
                                "<4B", v.color[0], v.color[1], v.color[2], v.color[3]
                            )
                        )
                    else:
                        # file.write(struct.pack("<4B", 0, 0, 0, 255))
                        file.write(struct.pack("<4B", 204, 204, 204, 255))

    def calcNeighbors(self):
        edges = {}
        for mesh in self.meshes:
            for i, (v0, v1, v2) in enumerate(mesh.tris):
                e0 = v0.neighborKey(v1)
                e1 = v1.neighborKey(v2)
                e2 = v2.neighborKey(v0)
                tri = mesh.firsttri + i
                try:
                    edges[e0].append(tri)
                except:
                    edges[e0] = [tri]
                try:
                    edges[e1].append(tri)
                except:
                    edges[e1] = [tri]
                try:
                    edges[e2].append(tri)
                except:
                    edges[e2] = [tri]
        neighbors = []
        for mesh in self.meshes:
            for i, (v0, v1, v2) in enumerate(mesh.tris):
                e0 = edges[v0.neighborKey(v1)]
                e1 = edges[v1.neighborKey(v2)]
                e2 = edges[v2.neighborKey(v0)]
                tri = mesh.firsttri + i
                match0 = match1 = match2 = -1
                if len(e0) == 2:
                    match0 = e0[e0.index(tri) ^ 1]
                if len(e1) == 2:
                    match1 = e1[e1.index(tri) ^ 1]
                if len(e2) == 2:
                    match2 = e2[e2.index(tri) ^ 1]
                neighbors.append((match0, match1, match2))
        self.neighbors = neighbors

    def writeTris(self, file):
        for mesh in self.meshes:
            for v0, v1, v2 in mesh.tris:
                file.write(
                    struct.pack(
                        "<3I",
                        v0.index + mesh.firstvert,
                        v1.index + mesh.firstvert,
                        v2.index + mesh.firstvert,
                    )
                )
        for n0, n1, n2 in self.neighbors:
            if n0 < 0:
                n0 = 0xFFFFFFFF
            if n1 < 0:
                n1 = 0xFFFFFFFF
            if n2 < 0:
                n2 = 0xFFFFFFFF
            file.write(struct.pack("<3I", n0, n1, n2))

    def export(self, file, usebbox=True):
        self.filesize = IQM_HEADER.size
        if self.textdata:
            while len(self.textdata) % 4:
                self.textdata += b"\x00"
            ofs_text = self.filesize
            self.filesize += len(self.textdata)
        else:
            ofs_text = 0
        if self.meshdata:
            ofs_meshes = self.filesize
            self.filesize += len(self.meshdata) * IQM_MESH.size
        else:
            ofs_meshes = 0
        if self.numverts > 0:
            ofs_vertexarrays = self.filesize
            num_vertexarrays = 4
            if self.joints:
                num_vertexarrays += 2
            hascolors = any(mesh.verts and mesh.verts[0].color for mesh in self.meshes)
            if hascolors:
                num_vertexarrays += 1
            self.filesize += num_vertexarrays * IQM_VERTEXARRAY.size
            ofs_vdata = self.filesize
            self.filesize += self.numverts * struct.calcsize("<3f2f3f4f")
            if self.joints:
                self.filesize += self.numverts * struct.calcsize("<4B4B")
            if hascolors:
                self.filesize += self.numverts * struct.calcsize("<4B")
        else:
            ofs_vertexarrays = 0
            num_vertexarrays = 0
            ofs_vdata = 0
        if self.numtris > 0:
            ofs_triangles = self.filesize
            self.filesize += self.numtris * IQM_TRIANGLE.size
            ofs_neighbors = self.filesize
            self.filesize += self.numtris * IQM_TRIANGLE.size
        else:
            ofs_triangles = 0
            ofs_neighbors = 0
        if self.jointdata:
            ofs_joints = self.filesize
            self.filesize += len(self.jointdata) * IQM_JOINT.size
        else:
            ofs_joints = 0
        if self.posedata:
            ofs_poses = self.filesize
            self.filesize += len(self.posedata) * IQM_POSE.size
        else:
            ofs_poses = 0
        if self.animdata:
            ofs_anims = self.filesize
            self.filesize += len(self.animdata) * IQM_ANIMATION.size
        else:
            ofs_anims = 0
        falign = 0
        if self.framesize * self.numframes > 0:
            ofs_frames = self.filesize
            self.filesize += self.framesize * self.numframes * struct.calcsize("<H")
            falign = (4 - (self.filesize % 4)) % 4
            self.filesize += falign
        else:
            ofs_frames = 0
        if usebbox and self.numverts > 0 and self.numframes > 0:
            ofs_bounds = self.filesize
            self.filesize += self.numframes * IQM_BOUNDS.size
        else:
            ofs_bounds = 0

        file.write(
            IQM_HEADER.pack(
                "INTERQUAKEMODEL".encode("ascii"),
                2,
                self.filesize,
                0,
                len(self.textdata),
                ofs_text,
                len(self.meshdata),
                ofs_meshes,
                num_vertexarrays,
                self.numverts,
                ofs_vertexarrays,
                self.numtris,
                ofs_triangles,
                ofs_neighbors,
                len(self.jointdata),
                ofs_joints,
                len(self.posedata),
                ofs_poses,
                len(self.animdata),
                ofs_anims,
                self.numframes,
                self.framesize,
                ofs_frames,
                ofs_bounds,
                0,
                0,
                0,
                0,
            )
        )
        file.write(self.textdata)
        for mesh in self.meshdata:
            file.write(IQM_MESH.pack(*mesh))
        self.writeVerts(file, ofs_vdata)
        self.writeTris(file)
        for joint in self.jointdata:
            file.write(IQM_JOINT.pack(*joint))
        for pose in self.posedata:
            file.write(IQM_POSE.pack(*pose))
        for anim in self.animdata:
            file.write(IQM_ANIMATION.pack(*anim))
        for anim in self.anims:
            file.write(anim.frameData(self.joints))
        file.write(b"\x00" * falign)
        if usebbox and self.numverts > 0 and self.numframes > 0:
            for anim in self.anims:
                file.write(anim.boundsData(self.joints, self.meshes))
