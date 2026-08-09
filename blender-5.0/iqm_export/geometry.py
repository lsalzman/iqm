import mathutils

MAXVCACHE = 32


class Vertex:
    def __init__(self, index, coord, normal, uv, weights, color):
        self.index = index
        self.coord = coord
        self.normal = normal
        self.uv = uv
        self.weights = weights
        self.color = color

    def normalizeWeights(self):
        # renormalizes all weights such that they add up to 255
        # the list is chopped/padded to exactly 4 weights if necessary
        if not self.weights:
            self.weights = [(0, 0), (0, 0), (0, 0), (0, 0)]
            return
        self.weights.sort(key=lambda weight: weight[0], reverse=True)
        if len(self.weights) > 4:
            del self.weights[4:]
        totalweight = sum([weight for (weight, bone) in self.weights])
        if totalweight > 0:
            self.weights = [
                (int(round(weight * 255.0 / totalweight)), bone)
                for (weight, bone) in self.weights
            ]
            while len(self.weights) > 1 and self.weights[-1][0] <= 0:
                self.weights.pop()
        else:
            totalweight = len(self.weights)
            self.weights = [
                (int(round(255.0 / totalweight)), bone)
                for (weight, bone) in self.weights
            ]
        totalweight = sum([weight for (weight, bone) in self.weights])
        while totalweight != 255:
            for i, (weight, bone) in enumerate(self.weights):
                if totalweight > 255 and weight > 0:
                    self.weights[i] = (weight - 1, bone)
                    totalweight -= 1
                elif totalweight < 255 and weight < 255:
                    self.weights[i] = (weight + 1, bone)
                    totalweight += 1
        while len(self.weights) < 4:
            self.weights.append((0, self.weights[-1][1]))

    def calcScore(self):
        if self.uses:
            self.score = 2.0 * pow(len(self.uses), -0.5)
            if self.cacherank >= 3:
                self.score += pow(1.0 - float(self.cacherank - 3) / MAXVCACHE, 1.5)
            elif self.cacherank >= 0:
                self.score += 0.75
        else:
            self.score = -1.0

    def neighborKey(self, other):
        if self.coord < other.coord:
            return (
                self.coord.x,
                self.coord.y,
                self.coord.z,
                other.coord.x,
                other.coord.y,
                other.coord.z,
                tuple(self.weights),
                tuple(other.weights),
            )
        else:
            return (
                other.coord.x,
                other.coord.y,
                other.coord.z,
                self.coord.x,
                self.coord.y,
                self.coord.z,
                tuple(other.weights),
                tuple(self.weights),
            )

    def __hash__(self):
        return self.index

    def __eq__(self, v):
        return (
            self.coord == v.coord
            and self.normal == v.normal
            and self.uv == v.uv
            and self.weights == v.weights
            and self.color == v.color
        )


class Mesh:
    def __init__(self, name, material, verts):
        self.name = name
        self.material = material
        self.verts = [None for v in verts]
        self.vertmap = {}
        self.tris = []

    def calcTangents(self):
        # See "Tangent Space Calculation" at http://www.terathon.com/code/tangent.html
        for v in self.verts:
            v.tangent = mathutils.Vector((0.0, 0.0, 0.0))
            v.bitangent = mathutils.Vector((0.0, 0.0, 0.0))
        for v0, v1, v2 in self.tris:
            dco1 = v1.coord - v0.coord
            dco2 = v2.coord - v0.coord
            duv1 = v1.uv - v0.uv
            duv2 = v2.uv - v0.uv
            tangent = dco2 * duv1.y - dco1 * duv2.y
            bitangent = dco2 * duv1.x - dco1 * duv2.x
            if dco2.cross(dco1).dot(bitangent.cross(tangent)) < 0:
                tangent.negate()
                bitangent.negate()
            v0.tangent += tangent
            v1.tangent += tangent
            v2.tangent += tangent
            v0.bitangent += bitangent
            v1.bitangent += bitangent
            v2.bitangent += bitangent
        for v in self.verts:
            v.tangent = v.tangent - v.normal * v.tangent.dot(v.normal)
            v.tangent.normalize()
            if v.normal.cross(v.tangent).dot(v.bitangent) < 0:
                v.bitangent = -1.0
            else:
                v.bitangent = 1.0

    def optimize(self):
        # Linear-speed vertex cache optimization algorithm by Tom Forsyth
        for v in self.verts:
            if v:
                v.index = -1
                v.uses = []
                v.cacherank = -1
        for i, (v0, v1, v2) in enumerate(self.tris):
            v0.uses.append(i)
            v1.uses.append(i)
            v2.uses.append(i)
        for v in self.verts:
            if v:
                v.calcScore()

        besttri = -1
        bestscore = -42.0
        scores = []
        for i, (v0, v1, v2) in enumerate(self.tris):
            scores.append(v0.score + v1.score + v2.score)
            if scores[i] > bestscore:
                besttri = i
                bestscore = scores[i]

        vertloads = 0  # debug info
        vertschedule = []
        trischedule = []
        vcache = []
        while besttri >= 0:
            tri = self.tris[besttri]
            scores[besttri] = -666.0
            trischedule.append(tri)
            for v in tri:
                if v.cacherank < 0:  # debug info
                    vertloads += 1  # debug info
                if v.index < 0:
                    v.index = len(vertschedule)
                    vertschedule.append(v)
                v.uses.remove(besttri)
                v.cacherank = -1
                v.score = -1.0
            vcache = [v for v in tri if v.uses] + [
                v for v in vcache if v.cacherank >= 0
            ]
            for i, v in enumerate(vcache):
                v.cacherank = i
                v.calcScore()

            besttri = -1
            bestscore = -42.0
            for v in vcache:
                for i in v.uses:
                    v0, v1, v2 = self.tris[i]
                    scores[i] = v0.score + v1.score + v2.score
                    if scores[i] > bestscore:
                        besttri = i
                        bestscore = scores[i]
            while len(vcache) > MAXVCACHE:
                vcache.pop().cacherank = -1
            if besttri < 0:
                for i, score in enumerate(scores):
                    if score > bestscore:
                        besttri = i
                        bestscore = score

        print(
            "%s: %d verts optimized to %d/%d loads for %d entry LRU cache"
            % (self.name, len(self.verts), vertloads, len(vertschedule), MAXVCACHE)
        )
        # print('%s: %d verts scheduled to %d' % (self.name, len(self.verts), len(vertschedule)))
        self.verts = vertschedule
        # print('%s: %d tris scheduled to %d' % (self.name, len(self.tris), len(trischedule)))
        self.tris = trischedule

    def meshData(self, iqm):
        return [
            iqm.addText(self.name),
            iqm.addText(self.material),
            self.firstvert,
            len(self.verts),
            self.firsttri,
            len(self.tris),
        ]
