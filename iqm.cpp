#include "util.h"

struct triangle { uint vert[3]; triangle() {} triangle(uint v0, uint v1, uint v2) { vert[0] = v0; vert[1] = v1; vert[2] = v2; } };
vector<triangle> triangles, neighbors;

struct mesh { uint name, material; uint firstvert, numverts; uint firsttri, numtris; mesh() : name(0), material(0), firstvert(0), numverts(0), firsttri(0), numtris(0) {} };
vector<mesh> meshes;

struct anim { uint name; uint firstframe, numframes; float fps; uint flags; anim() : name(0), firstframe(0), numframes(0), fps(0), flags(0) {} };
vector<anim> anims;

struct joint { uint name; int parent; float pos[3], orient[4], scale[3]; joint() : name(0), parent(-1) { memset(pos, 0, sizeof(pos)); memset(orient, 0, sizeof(orient)); memset(scale, 0, sizeof(scale)); } };
vector<joint> joints;

struct pose { int parent; uint flags; float offset[10], scale[10]; pose() : parent(-1), flags(0) { memset(offset, 0, sizeof(offset)); memset(scale, 0, sizeof(scale)); } };
vector<pose> poses;

struct framebounds { Vec3 bbmin, bbmax; double xyradius, radius; framebounds() : bbmin(0, 0, 0), bbmax(0, 0, 0), xyradius(0), radius(0) {} };
vector<framebounds> bounds;

struct transform
{
    Vec3 pos;
    Quat orient;
    Vec3 scale;

    transform() {}
    transform(const Vec3 &pos, const Quat &orient, const Vec3 &scale = Vec3(1, 1, 1)) : pos(pos), orient(orient), scale(scale) {}
};
vector<transform> frames;

vector<char> stringdata, commentdata;

struct sharedstring
{
    uint offset;
    sharedstring() {}
    sharedstring(const char *s) : offset(stringdata.length()) { stringdata.put(s, strlen(s)+1); }
}; 

static inline bool htcmp(const char *x, const sharedstring &s)
{
    return htcmp(x, &stringdata[s.offset]);
}

hashtable<sharedstring, uint> stringoffsets;

uint sharestring(const char *s)
{
    if(stringdata.empty()) stringoffsets.access("", 0);
    return stringoffsets.access(s ? s : "", stringdata.length());
}

struct blendcombo
{
    int sorted;
    double weights[4];
    uchar bones[4];

    blendcombo() : sorted(0) {}

    void reset() { sorted = 0; }

    void addweight(double weight, int bone)
    {
        if(weight <= 1e-3) return;
        loopk(sorted) if(weight > weights[k])
        {
            for(int l = min(sorted-1, 2); l >= k; l--)
            {
                weights[l+1] = weights[l];
                bones[l+1] = bones[l];
            }
            weights[k] = weight;
            bones[k] = bone;
            if(sorted<4) sorted++;
            return;
        }
        if(sorted>=4) return;
        weights[sorted] = weight;
        bones[sorted] = bone;
        sorted++;
    }

    void finalize()
    {
        loopj(4-sorted) { weights[sorted+j] = 0; bones[sorted+j] = 0; }
        if(sorted <= 0) return;
        double total = 0;
        loopj(sorted) total += weights[j];
        total = 1.0/total;
        loopj(sorted) weights[j] *= total;
    }

    void serialize(uchar *vweights) const
    {
        int total = 0;
        loopk(4) total += (vweights[k] = uchar(weights[k]*255));
        if(sorted <= 0) return;
        while(total > 255)
        {
            loopk(4) if(vweights[k] > 0 && total > 255) { vweights[k]--; total--; }
        }
        while(total < 255)
        {
            loopk(4) if(vweights[k] < 255 && total < 255) { vweights[k]++; total++; }
        }
    }

    bool operator==(const blendcombo &c) { loopi(4) if(weights[i] != c.weights[i] || bones[i] != c.bones[i]) return false; return true; }
    bool operator!=(const blendcombo &c) { loopi(4) if(weights[i] != c.weights[i] || bones[i] != c.bones[i]) return true; return false; }
};
    
static bool parseindex(char *&c, int &val)
{ 
    while(isspace(*c)) c++;
    char *end = NULL;
    int rval = strtol(c, &end, 10);
    if(c == end) return false;
    val = rval;
    c = end;
    return true;
}

static double parseattrib(char *&c, double ival = 0)
{
    while(isspace(*c)) c++;
    char *end = NULL;
    double val = strtod(c, &end);
    if(c == end) val = ival;
    else c = end;
    return val;
}

static bool maybeparseattrib(char *&c, double &result)
{
    while(isspace(*c)) c++;
    char *end = NULL;
    double val = strtod(c, &end);
    if(c == end) return false;
    c = end;
    result = val;
    return true;
}

#if 0
static bool parsename(char *&c, char *buf, int bufsize = sizeof(string))
{
    while(isspace(*c)) c++;
    char *end;
    if(*c == '"')
    {
        c++;
        end = c;
        while(*end && *end != '"') end++;
        copystring(buf, c, min(int(end-c+1), bufsize));
        if(*end == '"') end++;
    }
    else
    {
        end = c;
        while(*end && !isspace(*end)) end++;
        copystring(buf, c, min(int(end-c+1), bufsize));
    }
    if(c == end) return false;
    c = end;
    return true;
}
#endif

static char *trimname(char *&c)
{
    while(isspace(*c)) c++;
    char *start, *end;
    if(*c == '"')
    {
        c++;
        start = end = c;
        while(*end && *end != '"') end++;
        if(*end) { *end = '\0'; end++; }
    }
    else
    {
        start = end = c;
        while(*end && !isspace(*end)) end++;
        if(*end) { *end = '\0'; end++; }
    }
    c = end;
    return start;
}
 
static Vec4 parseattribs4(char *&c, const Vec4 &ival = Vec4(0, 0, 0, 0))
{
    Vec4 val;
    loopk(4) val[k] = parseattrib(c, ival[k]);
    return val;
}

static Vec3 parseattribs3(char *&c, const Vec3 &ival = Vec3(0, 0, 0))
{
    Vec3 val;
    loopk(3) val[k] = parseattrib(c, ival[k]);
    return val;
}

static blendcombo parseblends(char *&c)
{
    blendcombo b;
    int index;
    while(parseindex(c, index))
    {
        double weight = parseattrib(c, 0);
        b.addweight(weight, index);
    }
    b.finalize();
    return b;
}
        
struct ejoint
{
    const char *name;
    int parent;

    ejoint() : name(NULL), parent(-1) {}
};

struct eanim
{
    const char *name;
    int startframe, endframe;
    double fps;
    uint flags;

    eanim() : name(NULL), startframe(0), endframe(INT_MAX), fps(0), flags(0) {}
};

struct emesh
{
    const char *name, *material;
    int firsttri;
    bool used;

    emesh() : name(NULL), material(NULL), firsttri(0), used(false) {}
    emesh(const char *name, const char *material, int firsttri = 0) : name(name), material(material), firsttri(firsttri), used(false) {}
};

struct evarray
{
    string name;
    int type, format, size;

    evarray() : type(IQM_POSITION), format(IQM_FLOAT), size(3) { name[0] = '\0'; }
    evarray(int type, int format, int size, const char *initname = "") : type(type), format(format), size(size) { copystring(name, initname); }
};
    
struct esmoothgroup
{
    enum
    {
        F_USED     = 1<<0,
        F_UVSMOOTH = 1<<1
    };

    int key;
    float angle;
    int flags;

    esmoothgroup() : key(-1), angle(-1), flags(0) {}
};

struct etriangle
{
    int smoothgroup;
    uint vert[3], weld[3];

    etriangle() 
        : smoothgroup(-1) 
    {
    }
    etriangle(int v0, int v1, int v2, int smoothgroup = -1) 
        : smoothgroup(smoothgroup) 
    { 
        vert[0] = v0; 
        vert[1] = v1; 
        vert[2] = v2; 
    }
};

vector<Vec4> mpositions, epositions, etexcoords, etangents, ecolors, ecustom[10];
vector<Vec3> enormals, ebitangents;
vector<blendcombo> mblends, eblends;
vector<etriangle> etriangles;
vector<esmoothgroup> esmoothgroups;
vector<int> esmoothindexes;
vector<uchar> esmoothedges;
vector<ejoint> ejoints;
vector<transform> eposes;
vector<Matrix3x4> mjoints;
vector<int> eframes;
vector<eanim> eanims;
vector<emesh> emeshes;
vector<evarray> evarrays;
hashtable<const char *, char *> enames;

const char *getnamekey(const char *name)
{
    char **exists = enames.access(name);
    if(exists) return *exists;
    char *key = newstring(name);
    enames[key] = key;
    return key;
}

struct weldinfo
{
    int tri, vert;
    weldinfo *next;
};

void weldvert(const vector<Vec3> &norms, const Vec4 &pos, weldinfo *welds, int &numwelds, unionfind<int> &welder)
{
    welder.clear();
    int windex = 0;
    for(weldinfo *w = welds; w; w = w->next, windex++)
    {
        etriangle &wt = etriangles[w->tri];
        esmoothgroup &wg = esmoothgroups[wt.smoothgroup];
        int vindex = windex + 1;
        for(weldinfo *v = w->next; v; v = v->next, vindex++)
        {
            etriangle &vt = etriangles[v->tri];
            esmoothgroup &vg = esmoothgroups[vt.smoothgroup];
            if(wg.key != vg.key) continue;
            if(norms[w->tri].dot(norms[v->tri]) < max(wg.angle, vg.angle)) continue;
            if(((wg.flags | vg.flags) & esmoothgroup::F_UVSMOOTH) &&
               etexcoords[wt.vert[w->vert]] != etexcoords[vt.vert[v->vert]])
                continue;          
            if(esmoothindexes.length() > max(w->vert, v->vert) && esmoothindexes[w->vert] != esmoothindexes[v->vert])
                continue;
            if(esmoothedges.length())
            {
                int w0 = w->vert, w1 = (w->vert+1)%3, w2 = (w->vert+2)%3;
                const Vec4 &wp1 = epositions[wt.vert[w1]],
                           &wp2 = epositions[wt.vert[w2]];
                int v0 = v->vert, v1 = (v->vert+1)%3, v2 = (v->vert+2)%3;
                const Vec4 &vp1 = epositions[vt.vert[v1]],
                           &vp2 = epositions[vt.vert[v2]];
                int wf = esmoothedges[w->tri], vf = esmoothedges[v->tri];
                if((wp1 != vp1 || !(((wf>>w0)|(vf>>v0))&1)) &&
                   (wp1 != vp2 || !(((wf>>w0)|(vf>>v2))&1)) &&
                   (wp2 != vp1 || !(((wf>>w2)|(vf>>v0))&1)) &&
                   (wp2 != vp2 || !(((wf>>w2)|(vf>>v2))&1)))
                    continue;
            }    
            welder.unite(windex, vindex, -1);
        }
    }    
    windex = 0;
    for(weldinfo *w = welds; w; w = w->next, windex++)
    {
        etriangle &wt = etriangles[w->tri];
        wt.weld[w->vert] = welder.find(windex, -1, numwelds);
        if(wt.weld[w->vert] == uint(numwelds)) numwelds++;
    }
}

void smoothverts(bool areaweight = true)
{
    if(etriangles.empty()) return;

    if(enormals.length())
    {
        loopv(etriangles)
        {
            etriangle &t = etriangles[i];
            loopk(3) t.weld[k] = t.vert[k];
        }
        return;
    }

    if(etexcoords.empty()) loopv(esmoothgroups) esmoothgroups[i].flags &= ~esmoothgroup::F_UVSMOOTH;
    if(esmoothedges.length()) while(esmoothedges.length() < etriangles.length()) esmoothedges.add(7);

    vector<Vec3> tarea, tnorms;
    loopv(etriangles)
    {
        etriangle &t = etriangles[i];
        Vec3 v0(epositions[t.vert[0]]),
             v1(epositions[t.vert[1]]),
             v2(epositions[t.vert[2]]);
        tnorms.add(tarea.add((v2 - v0).cross(v1 - v0)).normalize());
    }

    int nextalloc = 0;
    vector<weldinfo *> allocs;
    hashtable<Vec4, weldinfo *> welds(1<<12);

    loopv(etriangles)
    {
        etriangle &t = etriangles[i];
        loopk(3)
        {
            weldinfo **next = &welds.access(epositions[t.vert[k]], NULL);
            if(! (nextalloc % 1024)) allocs.add(new weldinfo[1024]);
            weldinfo &w = allocs[nextalloc/1024][nextalloc%1024];
            nextalloc++;
            w.tri = i;
            w.vert = k;
            w.next = *next;
            *next = &w;
        }
    }

    int numwelds = 0;
    unionfind<int> welder;
    enumerate(welds, Vec4, vpos, weldinfo *, vwelds, weldvert(tnorms, vpos, vwelds, numwelds, welder));

    loopv(allocs) delete[] allocs[i];

    loopi(numwelds) enormals.add(Vec3(0, 0, 0));
    loopv(etriangles)
    {
        etriangle &t = etriangles[i];
        loopk(3) enormals[t.weld[k]]+= areaweight ? tarea[i] : tnorms[i];
    }
    loopv(enormals) if(enormals[i] != Vec3(0, 0, 0)) enormals[i] = enormals[i].normalize();
}

struct sharedvert
{
    int index, weld;

    sharedvert() {}
    sharedvert(int index, int weld) : index(index), weld(weld) {}
};

static inline bool htcmp(const sharedvert &v, const sharedvert &s)
{
    if(epositions[v.index] != epositions[s.index]) return false;
    if(etexcoords.length() && etexcoords[v.index] != etexcoords[s.index]) return false;
    if(enormals.length() && enormals[v.weld] != enormals[s.weld]) return false;
    if(eblends.length() && eblends[v.index] != eblends[s.index]) return false;
    if(ecolors.length() && ecolors[v.index] != ecolors[s.index]) return false;
    loopi(10) if(ecustom[i].length() && ecustom[i][v.index] != ecustom[i][s.index]) return false; 
    return true;
}

static inline uint hthash(const sharedvert &v)
{
    return hthash(epositions[v.index]);
}

const struct vertexarraytype
{
    const char *name;
    int code;
} vatypes[] =
{
    { "position", IQM_POSITION },
    { "texcoord", IQM_TEXCOORD },
    { "normal", IQM_NORMAL  },
    { "tangent", IQM_TANGENT },
    { "blendindexes", IQM_BLENDINDEXES },
    { "blendweights", IQM_BLENDWEIGHTS },
    { "color", IQM_COLOR },
    { "custom0", IQM_CUSTOM + 0 },
    { "custom1", IQM_CUSTOM + 1 },
    { "custom2", IQM_CUSTOM + 2 },
    { "custom3", IQM_CUSTOM + 3 },
    { "custom4", IQM_CUSTOM + 4 },
    { "custom5", IQM_CUSTOM + 5 },
    { "custom6", IQM_CUSTOM + 6 },
    { "custom7", IQM_CUSTOM + 7 },
    { "custom8", IQM_CUSTOM + 8 },
    { "custom9", IQM_CUSTOM + 9 }
};

int findvertexarraytype(const char *name)
{
    loopi(sizeof(vatypes)/sizeof(vatypes[0]))
    {
        if(!strcasecmp(vatypes[i].name, name))
            return vatypes[i].code;
    }
    return -1;
}

const struct vertexarrayformat
{
    const char *name;
    int code;
    int size;
} vaformats[] = 
{
    { "byte", IQM_BYTE, 1 },
    { "ubyte", IQM_UBYTE, 1 },
    { "short", IQM_SHORT, 2 },
    { "ushort", IQM_USHORT, 2 },
    { "int", IQM_INT, 4 },
    { "uint", IQM_UINT, 4 },
    { "half", IQM_HALF, 2 },
    { "float", IQM_FLOAT, 4 },
    { "double", IQM_DOUBLE, 8 }
};

int findvertexarrayformat(const char *name)
{
    loopi(sizeof(vaformats)/sizeof(vaformats[0]))
    {
        if(!strcasecmp(vaformats[i].name, name))
            return vaformats[i].code;
    }
    return -1;
}

struct vertexarray
{
    uint type, flags, format, size, offset;

    vertexarray(uint type, uint format, uint size, uint offset) : type(type), flags(0), format(format), size(size), offset(offset) {}

    int formatsize() const
    {
        return vaformats[format].size;
    }

    int bytesize() const
    {
        return size * vaformats[format].size;
    }
};

vector<sharedvert> vmap;
vector<vertexarray> varrays;
vector<uchar> vdata;

struct halfdata
{
    ushort val;

    halfdata(double d)
    {
        union
        {
            unsigned long long int i;
            double d;
        } conv;
        conv.d = d;
        ushort signbit = ushort((conv.i>>63)&1);
        ushort mantissa = ushort((conv.i>>(52-10))&0x3FF);
        int exponent = int((conv.i>>52)&0x7FF) - 1023 + 15;
        if(exponent <= 0)
        {
            mantissa |= 0x400;
            mantissa >>= min(1-exponent, 10+1);
            exponent = 0;
        } 
        else if(exponent >= 0x1F)
        {
            mantissa = 0;
            exponent = 0x1F;
        }
        val = (signbit<<15) | (ushort(exponent)<<10) | mantissa;
    }
};

template<> inline halfdata endianswap<halfdata>(halfdata n) { n.val = endianswap16(n.val); return n; }

template<int TYPE> static inline int remapindex(int i, const sharedvert &v) { return v.index; }
template<> inline int remapindex<IQM_NORMAL>(int i, const sharedvert &v) { return v.weld; }
template<> inline int remapindex<IQM_TANGENT>(int i, const sharedvert &v) { return i; }

template<class T, class U>
static inline void putattrib(T &out, const U &val) { out = T(val); }

template<class T, class U>
static inline void scaleattrib(T &out, const U &val) { putattrib(out, val); }
template<class U>
static inline void scaleattrib(char &out, const U &val) { out = char(clamp(val*255.0 - 0.5, -128.0, 127.0)); }
template<class U>
static inline void scaleattrib(short &out, const U &val) { out = short(clamp(val*65535.0 - 0.5, -32768.0, 32767.0)); }
template<class U>
static inline void scaleattrib(int &out, const U &val) { out = int(clamp(val*4294967295.0 - 0.5, -2147483648.0, 2147483647.0)); }
template<class U>
static inline void scaleattrib(uchar &out, const U &val) { out = uchar(clamp(val*255.0, 0.0, 255.0)); }
template<class U>
static inline void scaleattrib(ushort &out, const U &val) { out = ushort(clamp(val*65535.0, 0.0, 65535.0)); }
template<class U>
static inline void scaleattrib(uint &out, const U &val) { out = uint(clamp(val*4294967295.0, 0.0, 4294967295.0)); }

template<int T>
static inline bool normalizedattrib() { return true; }

template<int TYPE, int FMT, class T, class U>
static inline void serializeattrib(const vertexarray &va, T *data, const U &attrib)
{
    if(normalizedattrib<TYPE>()) switch(va.size)
    {
    case 4: scaleattrib(data[3], attrib.w);
    case 3: scaleattrib(data[2], attrib.z);
    case 2: scaleattrib(data[1], attrib.y);
    case 1: scaleattrib(data[0], attrib.x);
    }
    else switch(va.size)
    {
    case 4: putattrib(data[3], attrib.w);
    case 3: putattrib(data[2], attrib.z);
    case 2: putattrib(data[1], attrib.y);
    case 1: putattrib(data[0], attrib.x);
    }
    lilswap(data, va.size);
}

template<int TYPE, int FMT, class T>
static inline void serializeattrib(const vertexarray &va, T *data, const Vec3 &attrib)
{
    if(normalizedattrib<TYPE>()) switch(va.size)
    {
    case 3: scaleattrib(data[2], attrib.z);
    case 2: scaleattrib(data[1], attrib.y);
    case 1: scaleattrib(data[0], attrib.x);
    }
    else switch(va.size)
    {
    case 3: putattrib(data[2], attrib.z);
    case 2: putattrib(data[1], attrib.y);
    case 1: putattrib(data[0], attrib.x);
    }
    lilswap(data, va.size);
}

template<int TYPE, int FMT, class T>
static inline void serializeattrib(const vertexarray &va, T *data, const blendcombo &blend)
{
    if(TYPE == IQM_BLENDINDEXES)
    {
        switch(va.size)
        {
        case 4: putattrib(data[3], blend.bones[3]);
        case 3: putattrib(data[2], blend.bones[2]);
        case 2: putattrib(data[1], blend.bones[1]);
        case 1: putattrib(data[0], blend.bones[0]);
        }
    }
    else if(FMT == IQM_UBYTE)
    {
        uchar weights[4];
        blend.serialize(weights);
        switch(va.size)
        {
        case 4: putattrib(data[3], weights[3]);
        case 3: putattrib(data[2], weights[2]);
        case 2: putattrib(data[1], weights[1]);
        case 1: putattrib(data[0], weights[0]);
        }
    }
    else    
    {
        switch(va.size)
        {
        case 4: scaleattrib(data[3], blend.weights[3]);
        case 3: scaleattrib(data[2], blend.weights[2]);
        case 2: scaleattrib(data[1], blend.weights[1]);
        case 1: scaleattrib(data[0], blend.weights[0]);
        }
    }
    lilswap(data, va.size);
}

template<int TYPE, class T>
void setupvertexarray(const vector<T> &attribs, int type, int fmt, int size)
{
    vertexarray &va = varrays.add(vertexarray(type, fmt, size, vdata.length())); 
    const char *name = "";
    loopv(evarrays) if(evarrays[i].type == (int)va.type)
    {
        evarray &info = evarrays[i];
        va.format = info.format;
        va.size = clamp(info.size, 1, 4);
        name = info.name;
        break;
    }
    uint align = max(va.formatsize(), 4);
    if(va.offset%align) { uint pad = align - va.offset%align; va.offset += pad; loopi(pad) vdata.add(0); }
    if(va.type >= IQM_CUSTOM) 
    {
        if(!name[0])
        {
            defformatstring(customname)("custom%d", va.type-IQM_CUSTOM);
            va.type = IQM_CUSTOM + sharestring(customname);
        }
        else va.type = IQM_CUSTOM + sharestring(name);
    }
    int totalsize = va.bytesize() * vmap.length();
    uchar *data = vdata.reserve(totalsize);
    vdata.advance(totalsize);
    loopv(vmap)
    {
        const T &attrib = attribs[remapindex<TYPE>(i, vmap[i])];
        switch(va.format)
        {
        case IQM_BYTE: serializeattrib<TYPE, IQM_BYTE>(va, (char *)data, attrib); break;
        case IQM_UBYTE: serializeattrib<TYPE, IQM_UBYTE>(va, (uchar *)data, attrib); break;
        case IQM_SHORT: serializeattrib<TYPE, IQM_SHORT>(va, (short *)data, attrib); break;
        case IQM_USHORT: serializeattrib<TYPE, IQM_USHORT>(va, (ushort *)data, attrib); break;
        case IQM_INT: serializeattrib<TYPE, IQM_INT>(va, (int *)data, attrib); break;
        case IQM_UINT: serializeattrib<TYPE, IQM_UINT>(va, (uint *)data, attrib); break;
        case IQM_HALF: serializeattrib<TYPE, IQM_HALF>(va, (halfdata *)data, attrib); break;
        case IQM_FLOAT: serializeattrib<TYPE, IQM_FLOAT>(va, (float *)data, attrib); break;
        case IQM_DOUBLE: serializeattrib<TYPE, IQM_DOUBLE>(va, (double *)data, attrib); break;
        }
        data += va.bytesize();
    }
}

// linear speed vertex cache optimization from Tom Forsyth

#define MAXVCACHE 32

struct triangleinfo 
{ 
    bool used;
    float score;
    uint vert[3]; 

    triangleinfo() {} 
    triangleinfo(uint v0, uint v1, uint v2) 
    { 
        vert[0] = v0; 
        vert[1] = v1; 
        vert[2] = v2; 
    } 
};

struct vertexcache : listnode<vertexcache>
{
    int index, rank;
    float score;
    int numuses;
    triangleinfo **uses;

    vertexcache() : index(-1), rank(-1), score(-1.0f), numuses(0), uses(NULL) {}

    void calcscore()
    {
        if(numuses > 0)
        {
            score = 2.0f * powf(numuses, -0.5f);
            if(rank >= 3) score += powf(1.0f - (rank - 3)/float(MAXVCACHE - 3), 1.5f);
            else if(rank >= 0) score += 0.75f;
        }
        else score = -1.0f;
    }

    void removeuse(triangleinfo *t)
    {
        loopi(numuses) if(uses[i] == t)
        {
            uses[i] = uses[--numuses];
            return;
        }
    }
};

void maketriangles(vector<triangleinfo> &tris, const vector<sharedvert> &mmap)
{
    triangleinfo **uses = new triangleinfo *[3*tris.length()];
    vertexcache *verts = new vertexcache[mmap.length()];
    list<vertexcache> vcache;

    loopv(tris)
    {
        triangleinfo &t = tris[i];
        t.used = t.vert[0] == t.vert[1] || t.vert[1] == t.vert[2] || t.vert[2] == t.vert[0];
        if(t.used) continue;
        loopk(3) verts[t.vert[k]].numuses++;
    }
    triangleinfo **curuse = uses;
    loopvrev(tris)
    {
        triangleinfo &t = tris[i];
        if(t.used) continue;
        loopk(3)
        {
            vertexcache &v = verts[t.vert[k]];
            if(!v.uses) { curuse += v.numuses; v.uses = curuse; }
            *--v.uses = &t;
        }
    }
    loopv(mmap) verts[i].calcscore();
    triangleinfo *besttri = NULL;
    float bestscore = -1e16f;
    loopv(tris)
    {
        triangleinfo &t = tris[i];
        if(t.used) continue;
        t.score = verts[t.vert[0]].score + verts[t.vert[1]].score + verts[t.vert[2]].score;
        if(t.score > bestscore) { besttri = &t; bestscore = t.score; }
    }

    //int reloads = 0, n = 0;
    while(besttri)
    {
        besttri->used = true;
        triangle &t = triangles.add();
        loopk(3) 
        {
            vertexcache &v = verts[besttri->vert[k]];
            if(v.index < 0) { v.index = vmap.length(); vmap.add(mmap[besttri->vert[k]]); }
            t.vert[k] = v.index;
            v.removeuse(besttri);
            if(v.rank >= 0) vcache.remove(&v)->rank = -1;
            //else reloads++;
            if(v.numuses <= 0) continue;
            vcache.insertfirst(&v);
            v.rank = 0;
        }
        int rank = 0;
        for(vertexcache *v = vcache.first(); v != vcache.end(); v = v->next)
        {
            v->rank = rank++;
            v->calcscore();
        }
        besttri = NULL;
        bestscore = -1e16f;
        for(vertexcache *v = vcache.first(); v != vcache.end(); v = v->next)
        {
            loopi(v->numuses)
            {
                triangleinfo &t = *v->uses[i];
                t.score = verts[t.vert[0]].score + verts[t.vert[1]].score + verts[t.vert[2]].score;
                if(t.score > bestscore) { besttri = &t; bestscore = t.score; }
            }
        }
        while(vcache.size > MAXVCACHE) vcache.removelast()->rank = -1;
        if(!besttri) loopv(tris)
        {
            triangleinfo &t = tris[i];
            if(!t.used && t.score > bestscore) { besttri = &t; bestscore = t.score; }
        }
    }
    //printf("reloads: %d, worst: %d, best: %d\n", reloads, tris.length()*3, mmap.length());
        
    delete[] uses;
    delete[] verts;
}

void calctangents(bool areaweight = true)
{
    Vec3 *tangent = new Vec3[2*vmap.length()], *bitangent = tangent+vmap.length();
    memset(tangent, 0, 2*vmap.length()*sizeof(Vec3));
    loopv(triangles)
    {
        const triangle &t = triangles[i];
        sharedvert &i0 = vmap[t.vert[0]],
                   &i1 = vmap[t.vert[1]],
                   &i2 = vmap[t.vert[2]];

        Vec3 v0(epositions[i0.index]), e1 = Vec3(epositions[i1.index]) - v0, e2 = Vec3(epositions[i2.index]) - v0;

        double u1 = etexcoords[i1.index].x - etexcoords[i0.index].x, v1 = etexcoords[i1.index].y - etexcoords[i0.index].y,
               u2 = etexcoords[i2.index].x - etexcoords[i0.index].x, v2 = etexcoords[i2.index].y - etexcoords[i0.index].y;
        Vec3 u = e2*v1 - e1*v2,
             v = e2*u1 - e1*u2;

        if(e2.cross(e1).dot(v.cross(u)) < 0) 
        { 
            u = -u; 
            v = -v; 
        }

        if(!areaweight)
        {
            u = u.normalize();
            v = v.normalize();
        }
        
        loopj(3)
        {
            tangent[t.vert[j]] += u;
            bitangent[t.vert[j]] += v;
        }
    }
    loopv(vmap)
    {
        const Vec3 &n = enormals[vmap[i].weld],
                   &t = tangent[i],
                   &bt = bitangent[i];
        etangents.add(Vec4((t - n*n.dot(t)).normalize(), n.cross(t).dot(bt) < 0 ? -1 : 1));
    }
    delete[] tangent;
}

struct neighborkey
{
    uint e0, e1;

    neighborkey() {}
    neighborkey(uint e0, uint e1) : e0(min(e0, e1)), e1(max(e0, e1)) {}

    uint hash() const { return e0 + e1; }
    bool operator==(const neighborkey &n) const { return e0 == n.e0 && e1 == n.e1; }
};

static inline uint hthash(const neighborkey &n) { return n.hash(); }
static inline bool htcmp(const neighborkey &x, const neighborkey &y) { return x == y; }

struct neighborval
{
    uint tris[2];

    neighborval() {}
    neighborval(uint i) { tris[0] = i; tris[1] = 0xFFFFFFFFU; }

    void add(uint i)
    {
        if(tris[1] != 0xFFFFFFFFU) tris[0] = tris[1] = 0xFFFFFFFFU;
        else if(tris[0] != 0xFFFFFFFFU) tris[1] = i;
    } 
    
    int opposite(uint i) const
    {
        return tris[0] == i ? tris[1] : tris[0];
    }
};

void makeneighbors()
{
    hashtable<neighborkey, neighborval> nhash;

    loopv(triangles)
    {
        triangle &t = triangles[i];
        for(int j = 0, p = 2; j < 3; p = j, j++)
        {
            neighborkey key(t.vert[p], t.vert[j]);
            neighborval *val = nhash.access(key);
            if(val) val->add(i);
            else nhash[key] = neighborval(i);
        }
    }

    loopv(triangles)
    {
        triangle &t = triangles[i];
        triangle &n = neighbors.add();
        for(int j = 0, p = 2; j < 3; p = j, j++)
            n.vert[p] = nhash[neighborkey(t.vert[p], t.vert[j])].opposite(i);
    }
}

double escale = 1;
    
void makemeshes()
{
    meshes.setsize(0);
    triangles.setsize(0);
    neighbors.setsize(0);
    vmap.setsize(0);
    varrays.setsize(0);
    vdata.setsize(0);

    hashtable<sharedvert, uint> mshare(1<<12);
    vector<sharedvert> mmap;
    vector<triangleinfo> tinfo;

    loopv(emeshes)
    {
        emesh &em1 = emeshes[i];
        if(em1.used) continue;
        for(int j = i; j < emeshes.length(); j++) 
        {
            emesh &em = emeshes[j];
            if(em.name != em1.name || em.material != em1.material) continue;
            int lasttri = emeshes.inrange(i+1) ? emeshes[i+1].firsttri : etriangles.length();
            for(int k = em.firsttri; k < lasttri; k++)
            {
                etriangle &et = etriangles[k];
                triangleinfo &t = tinfo.add();    
                loopl(3)
                {
                    sharedvert v(et.vert[l], et.weld[l]);
                    t.vert[l] = mshare.access(v, mmap.length());
                    if(!mmap.inrange(t.vert[l])) mmap.add(v);
                }
            }
            em.used = true;
        }
        if(tinfo.empty()) continue;

        mesh &m = meshes.add();
        m.name = sharestring(em1.name);
        m.material = sharestring(em1.material);
        m.firsttri = triangles.length();
        m.firstvert = vmap.length();
        maketriangles(tinfo, mmap);
        m.numtris = triangles.length() - m.firsttri;
        m.numverts = vmap.length() - m.firstvert;

        mshare.clear();
        mmap.setsize(0);
        tinfo.setsize(0);
    }

    if(triangles.length()) makeneighbors();

    if(escale != 1) loopv(epositions) epositions[i] *= escale;
    if(epositions.length()) setupvertexarray<IQM_POSITION>(epositions, IQM_POSITION, IQM_FLOAT, 3);
    if(etexcoords.length()) setupvertexarray<IQM_TEXCOORD>(etexcoords, IQM_TEXCOORD, IQM_FLOAT, 2);
    if(enormals.length()) setupvertexarray<IQM_NORMAL>(enormals, IQM_NORMAL, IQM_FLOAT, 3);
    if(etangents.length())
    {
        if(ebitangents.length() && enormals.length())
        {
            loopv(etangents) if(ebitangents.inrange(i) && enormals.inrange(i))
                etangents[i].w = enormals[i].cross(Vec3(etangents[i])).dot(ebitangents[i]) < 0 ? -1 : 1;
        }
        setupvertexarray<IQM_TANGENT>(etangents, IQM_TANGENT, IQM_FLOAT, 4);
    }    
    else if(enormals.length() && etexcoords.length())
    {
        calctangents();
        setupvertexarray<IQM_TANGENT>(etangents, IQM_TANGENT, IQM_FLOAT, 4);
    }
    if(eblends.length())
    {
        setupvertexarray<IQM_BLENDINDEXES>(eblends, IQM_BLENDINDEXES, IQM_UBYTE, 4);
        setupvertexarray<IQM_BLENDWEIGHTS>(eblends, IQM_BLENDWEIGHTS, IQM_UBYTE, 4);
    }
    if(ecolors.length()) setupvertexarray<IQM_COLOR>(ecolors, IQM_COLOR, IQM_UBYTE, 4);
    loopi(10) if(ecustom[i].length()) setupvertexarray<IQM_CUSTOM>(ecustom[i], IQM_CUSTOM + i, IQM_FLOAT, 4);

    if(epositions.length())
    {
        mpositions.setsize(0);
        mpositions.swap(epositions);
    }
    if(eblends.length())
    {
        mblends.setsize(0);
        mblends.swap(eblends);
    }
}

void makebounds(framebounds &bb, Matrix3x4 *buf, Matrix3x4 *invbase, transform *frame)
{
    loopv(ejoints)
    {
        ejoint &j = ejoints[i];
        if(j.parent >= 0) buf[i] = buf[j.parent] * Matrix3x4(frame[i].orient, frame[i].pos, frame[i].scale);
        else buf[i] = Matrix3x4(frame[i].orient, frame[i].pos, frame[i].scale);
    }
    loopv(ejoints) buf[i] *= invbase[i];
    loopv(mpositions)
    {
        const blendcombo &c = mblends[i]; 
        Matrix3x4 m(Vec4(0, 0, 0, 0), Vec4(0, 0, 0, 0), Vec4(0, 0, 0, 0));
        loopk(4) if(c.weights[k] > 0)
            m += buf[c.bones[k]] * c.weights[k];
        Vec3 p = m.transform(Vec3(mpositions[i]));
        
        if(!i) bb.bbmin = bb.bbmax = p;
        else
        {
            bb.bbmin.x = min(bb.bbmin.x, p.x);
            bb.bbmin.y = min(bb.bbmin.y, p.y);
            bb.bbmin.z = min(bb.bbmin.z, p.z);
            bb.bbmax.x = max(bb.bbmax.x, p.x);
            bb.bbmax.y = max(bb.bbmax.y, p.y);
            bb.bbmax.z = max(bb.bbmax.z, p.z);
        }
        double xyradius = p.x*p.x + p.y*p.y;
        bb.xyradius = max(bb.xyradius, xyradius);
        bb.radius = max(bb.radius, xyradius + p.z*p.z);
    }
    if(bb.xyradius > 0) bb.xyradius = sqrt(bb.xyradius);
    if(bb.radius > 0) bb.radius = sqrt(bb.radius);
}
     
void makerelativebasepose()
{
    int numbasejoints = min(ejoints.length(), eframes.length() ? eframes[0] : eposes.length());
    for(int i = numbasejoints-1; i >= 0; i--)
    {
        ejoint &ej = ejoints[i];
        if(ej.parent < 0) continue; 
        transform &parent = eposes[ej.parent], &child = eposes[i];
        child.pos = (-parent.orient).transform(child.pos - parent.pos);   
        child.orient = (-parent.orient)*child.orient;
        if(child.orient.w > 0) child.orient.flip();
    }
}

void makeanims()
{
    if(escale != 1) loopv(eposes) eposes[i].pos *= escale;
    int numbasejoints = eframes.length() ? eframes[0] : eposes.length();
    if(meshes.length() && joints.empty()) 
    {
        mjoints.setsize(0);
        loopv(ejoints)
        {
            ejoint &ej = ejoints[i];
            joint &j = joints.add();
            j.name = sharestring(ej.name);
            j.parent = ej.parent;
            if(i < numbasejoints) 
            {
                mjoints.add().invert(Matrix3x4(eposes[i].orient, eposes[i].pos, eposes[i].scale));
                loopk(3) j.pos[k] = eposes[i].pos[k];
                loopk(4) j.orient[k] = eposes[i].orient[k];
                loopk(3) j.scale[k] = eposes[i].scale[k];
            }
            else mjoints.add().invert(Matrix3x4(Quat(0, 0, 0, 1), Vec3(0, 0, 0), Vec3(1, 1, 1)));
            if(ej.parent >= 0) mjoints[i] *= mjoints[ej.parent];
        }
    }
    if(eanims.empty()) return;
    if(poses.empty()) loopv(ejoints)
    {
        ejoint &ej = ejoints[i];
        pose &p = poses.add();
        p.parent = ej.parent;
    }    
    if(poses.empty()) return;
    int totalframes = frames.length()/poses.length();
    Matrix3x4 *mbuf = mpositions.length() && mblends.length() && mjoints.length() ? new Matrix3x4[poses.length()] : NULL;
    loopv(eanims)
    {
        eanim &ea = eanims[i];
        anim &a = anims.add();
        a.name = sharestring(ea.name);
        a.firstframe = totalframes;
        a.numframes = 0;
        a.fps = ea.fps;
        a.flags = ea.flags;
        for(int j = ea.startframe, end = eanims.inrange(i+1) ? eanims[i+1].startframe : eframes.length(); j < end && j <= ea.endframe; j++)
        {
            int offset = eframes[j], range = (eframes.inrange(j+1) ? eframes[j+1] : eposes.length()) - offset;
            if(range <= 0) continue;
            loopk(min(range, poses.length())) frames.add(eposes[offset + k]); 
            loopk(max(poses.length() - range, 0)) frames.add(transform(Vec3(0, 0, 0), Quat(0, 0, 0, 1), Vec3(1, 1, 1)));
            if(mbuf) makebounds(bounds.add(), mbuf, mjoints.getbuf(), &frames[frames.length() - poses.length()]);
            a.numframes++;
        }
        totalframes += a.numframes;
    }
    if(mbuf) delete[] mbuf;
}

void resetimporter()
{
    epositions.setsize(0);
    etexcoords.setsize(0);
    etangents.setsize(0);
    ebitangents.setsize(0);
    ecolors.setsize(0);
    loopi(10) ecustom[i].setsize(0);
    eblends.setsize(0);
    etriangles.setsize(0);
    esmoothindexes.setsize(0);
    esmoothedges.setsize(0);
    esmoothgroups.setsize(0);
    esmoothgroups.add();
    ejoints.setsize(0);
    eposes.setsize(0);
    eframes.setsize(0);
    eanims.setsize(0);
    emeshes.setsize(0);
    evarrays.setsize(0);
}

struct filespec
{
    const char *file;
    const char *name;
    double fps;
    uint flags;
    int startframe;
    int endframe;

    filespec() { reset(); }

    void reset()
    {
        file = NULL;
        name = NULL;
        fps = 0;
        flags = 0;
        startframe = 0;
        endframe = -1;
    }
};
 
bool loadiqe(const char *filename, const filespec &spec)
{
    stream *f = openfile(filename, "r");
    if(!f) return false;

    resetimporter();

    const char *curmesh = getnamekey(""), *curmaterial = getnamekey("");
    bool needmesh = true;
    int fmoffset = 0;
    char buf[512];
    if(!f->getline(buf, sizeof(buf))) goto error;
    if(!strchr(buf, '#') || strstr(buf, "# Inter-Quake Export") != strchr(buf, '#')) goto error;
    while(f->getline(buf, sizeof(buf)))
    {
        char *c = buf;
        while(isspace(*c)) ++c;
        if(isalpha(c[0]) && isalnum(c[1]) && (!c[2] || isspace(c[2]))) switch(*c++)
        {
        case 'v':
            switch(*c++)
            {
            case 'p': epositions.add(parseattribs4(c, Vec4(0, 0, 0, 1))); continue;
            case 't': etexcoords.add(parseattribs4(c)); continue;
            case 'n': enormals.add(parseattribs3(c)); continue;
            case 'x':
            {
                Vec4 tangent(parseattribs3(c), 0);
                Vec3 bitangent(0, 0, 0);
                bitangent.x = parseattrib(c);
                if(maybeparseattrib(c, bitangent.y))
                {
                    bitangent.z = parseattrib(c);
                    ebitangents.add(bitangent);
                }    
                else tangent.w = bitangent.x;
                etangents.add(tangent);
                continue;
            }
            case 'b': eblends.add(parseblends(c)); continue;
            case 'c': ecolors.add(parseattribs4(c, Vec4(0, 0, 0, 1))); continue;
            case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
            {
                int n = c[-1] - '0';
                ecustom[n].add(parseattribs4(c));
                continue;
            }
            case 's':
                parseindex(c, esmoothindexes.add());
                continue;
            }
            break;
        case 'p':
        {
            transform t;
            switch(*c++)
            {
            case 'q':
                {
                    t.pos = parseattribs3(c);
                    loopk(3) t.orient[k] = parseattrib(c);
                    t.orient.restorew();
                    double w = parseattrib(c, t.orient.w);
                    if(w != t.orient.w)
                    {
                        t.orient.w = w;
                        t.orient.normalize();
//        double x2 = f.orient.x*f.orient.x, y2 = f.orient.y*f.orient.y, z2 = f.orient.z*f.orient.z, w2 = f.orient.w*f.orient.w, s2 = x2 + y2 + z2 + w2;
//        f.orient.x = keepsign(f.orient.x, sqrt(max(1.0 - (w2 + y2 + z2) / s2, 0.0)));
//        f.orient.y = keepsign(f.orient.y, sqrt(max(1.0 - (w2 + x2 + z2) / s2, 0.0)));
//        f.orient.z = keepsign(f.orient.z, sqrt(max(1.0 - (w2 + x2 + y2) / s2, 0.0)));
//        f.orient.w = keepsign(f.orient.w, sqrt(max(1.0 - (x2 + y2 + z2) / s2, 0.0)));
                    }
                    if(t.orient.w > 0) t.orient.flip();
                    t.scale = parseattribs3(c, Vec3(1, 1, 1));
                    eposes.add(t);
                    continue;
                }
            case 'm':
                {
                    t.pos = parseattribs3(c);
                    Matrix3x3 m;
                    m.a = parseattribs3(c);
                    m.b = parseattribs3(c);
                    m.c = parseattribs3(c);
                    Vec3 mscale(Vec3(m.a.x, m.b.x, m.c.x).magnitude(), Vec3(m.a.y, m.b.y, m.c.y).magnitude(), Vec3(m.a.z, m.b.z, m.c.z).magnitude());
                    // check determinant for sign of scaling
                    if(m.determinant() < 0) mscale = -mscale;
                    m.a /= mscale;
                    m.b /= mscale;
                    m.c /= mscale;
                    t.orient = Quat(m);
                    if(t.orient.w > 0) t.orient.flip();
                    t.scale = parseattribs3(c, Vec3(1, 1, 1)) * mscale;
                    eposes.add(t);
                    continue;
                }
            case 'a':
                {
                    t.pos = parseattribs3(c);
                    Vec3 rot = parseattribs3(c);
                    double cx = cos(rot.x/2), sx = sin(rot.x/2),
                           cy = cos(rot.y/2), sy = sin(rot.y/2),
                           cz = cos(rot.z/2), sz = sin(rot.z/2);
                    t.orient = Quat(sx*cy*cz - cx*sy*sz,
                                    cx*sy*cz + sx*cy*sz,
                                    cx*cy*sz - sx*sy*cz,
                                    cx*cy*cz + sx*sy*sz);
                    if(t.orient.w > 0) t.orient.flip();
                    t.scale = parseattribs3(c, Vec3(1, 1, 1));
                    eposes.add(t);
                    continue;
                } 
            }
            break;
        }
        case 'f':
            switch(*c++)
            {
            case 'a': 
                {
                    int i1 = 0, i2 = 0, i3 = 0;
                    if(!parseindex(c, i1) || !parseindex(c, i2)) continue;
                    if(needmesh)
                    {
                        emeshes.add(emesh(curmesh, curmaterial, etriangles.length()));
                        needmesh = false;
                    }
                    if(i1 < 0) i1 = max(epositions.length() + i1, 0);
                    if(i2 < 0) i2 = max(epositions.length() + i2, 0);
                    while(parseindex(c, i3))
                    {
                        if(i3 < 0) i3 = max(epositions.length() + i3, 0);
                        esmoothgroups.last().flags |= esmoothgroup::F_USED;
                        etriangles.add(etriangle(i1, i2, i3, esmoothgroups.length()-1));
                        i2 = i3;
                    }         
                    continue;
                }
            case 'm': 
                {
                    int i1 = 0, i2 = 0, i3 = 0;
                    if(!parseindex(c, i1) || !parseindex(c, i2)) continue;
                    if(needmesh)
                    {
                        emeshes.add(emesh(curmesh, curmaterial, etriangles.length()));
                        needmesh = false;
                    }
                    i1 = i1 < 0 ? max(epositions.length() + i1, 0) : (fmoffset + i1);
                    i2 = i2 < 0 ? max(epositions.length() + i2, 0) : (fmoffset + i2);
                    while(parseindex(c, i3))
                    {
                        i3 = i3 < 0 ? max(epositions.length() + i3, 0) : (fmoffset + i3);
                        esmoothgroups.last().flags |= esmoothgroup::F_USED;
                        etriangles.add(etriangle(i1, i2, i3, esmoothgroups.length()-1));
                        i2 = i3;
                    }
                    continue;
                }
            case 's':
                {
                    int i1 = 0, i2 = 0, i3 = 0;
                    uchar flags = 0;
                    if(!parseindex(c, i1) || !parseindex(c, i2) || !parseindex(c, i3)) continue;
                    flags |= clamp(i1, 0, 1);
                    flags |= clamp(i2, 0, 1)<<1;
                    flags |= clamp(i3, 0, 1)<<2;
                    esmoothgroups.last().flags |= esmoothgroup::F_USED;
                    while(parseindex(c, i3))
                    {
                        esmoothedges.add(flags | 4);
                        flags = 1 | ((flags & 4) >> 1) | (clamp(i3, 0, 1)<<2);
                    } 
                    esmoothedges.add(flags);
                    continue;
                }
            }
            break;
        }    
        char *args = c;
        while(*args && !isspace(*args)) args++;
        if(!strncmp(c, "smoothgroup", max(int(args-c), 11)))
        {
            if(esmoothgroups.last().flags & esmoothgroup::F_USED) esmoothgroups.dup();
            parseindex(args, esmoothgroups.last().key);
        }
        else if(!strncmp(c, "smoothangle", max(int(args-c), 11)))
        {
            if(esmoothgroups.last().flags & esmoothgroup::F_USED) esmoothgroups.dup();
            double angle = parseattrib(args, 0);
            esmoothgroups.last().angle = fabs(cos(clamp(angle, -180.0, 180.0) * M_PI/180));
        }
        else if(!strncmp(c, "smoothuv", max(int(args-c), 8)))
        {
            if(esmoothgroups.last().flags & esmoothgroup::F_USED) esmoothgroups.dup();
            int val = 1;
            if(parseindex(args, val) && val <= 0) esmoothgroups.last().flags &= ~esmoothgroup::F_UVSMOOTH;
            else esmoothgroups.last().flags |= esmoothgroup::F_UVSMOOTH;
        }
        else if(!strncmp(c, "mesh", max(int(args-c), 4)))
        { 
            curmesh = getnamekey(trimname(args));
            if(emeshes.empty() || emeshes.last().name != curmesh) needmesh = true;
            fmoffset = epositions.length();

#if 0
            emesh &m = emeshes.add();
            m.firsttri = etriangles.length();
            fmoffset = epositions.length();
            parsename(args, m.name);
#endif
        }
        else if(!strncmp(c, "material", max(int(args-c), 8)))
        {
            curmaterial = getnamekey(trimname(args));
            if(emeshes.empty() || emeshes.last().material != curmaterial) needmesh = true;
//            if(emeshes.length()) parsename(c, emeshes.last().material);
        }
        else if(!strncmp(c, "joint", max(int(args-c), 5)))
        {
            ejoint &j = ejoints.add();
            j.name = getnamekey(trimname(args));
            parseindex(args, j.parent);
        }
        else if(!strncmp(c, "vertexarray", max(int(args-c), 11)))
        {
            evarray &va = evarrays.add();
            va.type = findvertexarraytype(trimname(args));
            va.format = findvertexarrayformat(trimname(args)); 
            va.size = strtol(args, &args, 10);
            copystring(va.name, trimname(args));
        }
        else if(!strncmp(c, "animation", max(int(args-c), 9)))
        {
            eanim &a = eanims.add();
            a.name = getnamekey(trimname(args));
            a.startframe = eframes.length();
            if(!eframes.length() || eframes.last() != eposes.length()) eframes.add(eposes.length());
        }
        else if(!strncmp(c, "frame", max(int(args-c), 5)))
        {
            if(eanims.length() && eframes.length() && eframes.last() != eposes.length()) eframes.add(eposes.length()); 
        }
        else if(!strncmp(c, "framerate", max(int(args-c), 9)))
        {
            if(eanims.length())
            {
                double fps = parseattrib(args);
                eanims.last().fps = max(fps, 0.0);
            }
        }
        else if(!strncmp(c, "loop", max(int(args-c), 4)))
        {
            if(eanims.length()) eanims.last().flags |= IQM_LOOP;
        }
        else if(!strncmp(c, "comment", max(int(args-c), 7)))
        {
            if(commentdata.length()) break;
            for(;;)
            {
                int len = f->read(commentdata.reserve(1024), 1024);
                commentdata.advance(len);
                if(len < 1024) { commentdata.add('\0'); break; }
            }
        }
    }

    delete f;

    if(eanims.length() == 1)
    {
        eanim &a = eanims.last();
        if(spec.name) a.name = spec.name;
        if(spec.fps > 0) a.fps = spec.fps;
        a.flags |= spec.flags;
        if(spec.endframe >= 0) a.endframe = a.startframe + spec.endframe;
        a.startframe += spec.startframe;
    }

    smoothverts();
    makemeshes();
    makeanims();
    
    return true;

error:
    delete f;
    return false;
}

struct md5weight
{
    int joint;
    double bias;
    Vec3 pos;
};

struct md5vert
{
    double u, v;
    uint start, count;
};

struct md5hierarchy
{
    const char *name;
    int parent, flags, start;
};

vector<md5weight> weightinfo;
vector<md5vert> vertinfo;

void buildmd5verts()
{
    loopv(vertinfo)
    {
        md5vert &v = vertinfo[i];
        Vec3 pos(0, 0, 0);
        loopk(v.count)
        {
            md5weight &w = weightinfo[v.start+k];
            transform &j = eposes[w.joint];
            pos += (j.orient.transform(w.pos) + j.pos)*w.bias;
        }
        epositions.add(Vec4(pos, 1));
        etexcoords.add(Vec4(v.u, v.v, 0, 0));

        blendcombo &c = eblends.add();
        loopj(v.count)
        {
            md5weight &w = weightinfo[v.start+j];
            c.addweight(w.bias, w.joint);
        }
        c.finalize();
    }
}

void parsemd5mesh(stream *f, char *buf, size_t bufsize)
{
    md5weight w;
    md5vert v;
    etriangle t(0, 0, 0, 0);
    int index, firsttri = etriangles.length(), firstvert = vertinfo.length(), firstweight = weightinfo.length(), numtris = 0, numverts = 0, numweights = 0;
    emesh m;

    while(f->getline(buf, bufsize) && buf[0]!='}')
    {
        if(strstr(buf, "// meshes:"))
        {
            char *start = strchr(buf, ':')+1;
            if(*start==' ') start++;
            char *end = start + strlen(start)-1;
            while(end >= start && isspace(*end)) end--;
            end[1] = '\0';
            m.name = getnamekey(start);
        }
        else if(strstr(buf, "shader"))
        {
            char *start = strchr(buf, '"'), *end = start ? strchr(start+1, '"') : NULL;
            if(start && end)
            {
                *end = '\0';
                m.material = getnamekey(start+1);
            }
        }
        else if(sscanf(buf, " numverts %d", &numverts)==1)
        {
            numverts = max(numverts, 0);
            if(numverts)
            {
                vertinfo.reserve(numverts);
                vertinfo.advance(numverts);
            }
        }
        else if(sscanf(buf, " numtris %d", &numtris)==1)
        {
            numtris = max(numtris, 0);
            if(numtris)
            {
                etriangles.reserve(numtris);
                etriangles.advance(numtris);
            }
            m.firsttri = firsttri;
        }
        else if(sscanf(buf, " numweights %d", &numweights)==1)
        {
            numweights = max(numweights, 0);
            if(numweights)
            {
                weightinfo.reserve(numweights);
                weightinfo.advance(numweights);
            }
        }
        else if(sscanf(buf, " vert %d ( %lf %lf ) %u %u", &index, &v.u, &v.v, &v.start, &v.count)==5)
        {
            if(index>=0 && index<numverts)
            {
                v.start += firstweight;
                vertinfo[firstvert + index] = v;
            }
        }
        else if(sscanf(buf, " tri %d %u %u %u", &index, &t.vert[0], &t.vert[1], &t.vert[2])==4)
        {
            if(index>=0 && index<numtris)
            {
                loopk(3) t.vert[k] += firstvert;
                etriangles[firsttri + index] = t;
            }
        }
        else if(sscanf(buf, " weight %d %d %lf ( %lf %lf %lf ) ", &index, &w.joint, &w.bias, &w.pos.x, &w.pos.y, &w.pos.z)==6)
        {
            if(index>=0 && index<numweights) weightinfo[firstweight + index] = w;
        }
    }

    if(numtris && numverts) emeshes.add(m);
}

bool loadmd5mesh(const char *filename, const filespec &spec)
{
    stream *f = openfile(filename, "r");
    if(!f) return false;

    resetimporter();
    esmoothgroups.add().flags |= esmoothgroup::F_UVSMOOTH;

    char buf[512];
    while(f->getline(buf, sizeof(buf)))
    {
        int tmp;
        if(sscanf(buf, " MD5Version %d", &tmp)==1)
        {
            if(tmp!=10) { delete f; return false; }
        }
        else if(sscanf(buf, " numJoints %d", &tmp)==1)
        {
            if(tmp<1 || (joints.length() && tmp != joints.length())) { delete f; return false; }
        }
        else if(sscanf(buf, " numMeshes %d", &tmp)==1)
        {
            if(tmp<1) { delete f; return false; }
        }
        else if(strstr(buf, "joints {"))
        {
            ejoint j;
            transform p;
            while(f->getline(buf, sizeof(buf)) && buf[0]!='}')
            {
                char *c = buf;
                j.name = getnamekey(trimname(c));
                if(sscanf(c, " %d ( %lf %lf %lf ) ( %lf %lf %lf )",
                    &j.parent, &p.pos.x, &p.pos.y, &p.pos.z,
                    &p.orient.x, &p.orient.y, &p.orient.z)==7)
                {
                    p.orient.restorew();
                    p.scale = Vec3(1, 1, 1);
                    ejoints.add(j);
                    eposes.add(p);
                }
            }
        }
        else if(strstr(buf, "mesh {"))
        {
            parsemd5mesh(f, buf, sizeof(buf));
        }
    }

    delete f;

    buildmd5verts();
    smoothverts();
    makemeshes();
    makerelativebasepose();
    makeanims();

    return true;
}

bool loadmd5anim(const char *filename, const filespec &spec)
{
    stream *f = openfile(filename, "r");
    if(!f) return false;

    resetimporter();

    vector<md5hierarchy> hierarchy;
    vector<transform> baseframe;
    int animdatalen = 0, animframes = 0, frameoffset = eposes.length(), firstframe = eframes.length();
    double framerate = 0;
    double *animdata = NULL;
    char buf[512];
    while(f->getline(buf, sizeof(buf)))
    {
        int tmp;
        if(sscanf(buf, " MD5Version %d", &tmp)==1)
        {
            if(tmp!=10) { delete f; return false; }
        }
        else if(sscanf(buf, " numJoints %d", &tmp)==1)
        {
            if(tmp<1) { delete f; return false; }
        }
        else if(sscanf(buf, " numFrames %d", &animframes)==1)
        {
            if(animframes<1) { delete f; return false; }
        }
        else if(sscanf(buf, " frameRate %lf", &framerate)==1);
        else if(sscanf(buf, " numAnimatedComponents %d", &animdatalen)==1)
        {
            if(animdatalen>0) animdata = new double[animdatalen];
        }
        else if(strstr(buf, "bounds {"))
        {
            while(f->getline(buf, sizeof(buf)) && buf[0]!='}');
        }
        else if(strstr(buf, "hierarchy {"))
        {
            while(f->getline(buf, sizeof(buf)) && buf[0]!='}')
            {
                char *c = buf;
                md5hierarchy h;
                h.name = getnamekey(trimname(c));
                if(sscanf(c, " %d %d %d", &h.parent, &h.flags, &h.start)==3)
                    hierarchy.add(h);
            }
            if(hierarchy.empty() || (poses.length() && hierarchy.length()!=poses.length())) { delete f; return false; }
            loopv(hierarchy)
            {
                md5hierarchy &h = hierarchy[i];
                ejoint &j = ejoints.add();
                j.name = h.name;
                j.parent = h.parent;
            }
        }
        else if(strstr(buf, "baseframe {"))
        {
            while(f->getline(buf, sizeof(buf)) && buf[0]!='}')
            {
                transform j;
                if(sscanf(buf, " ( %lf %lf %lf ) ( %lf %lf %lf )", &j.pos.x, &j.pos.y, &j.pos.z, &j.orient.x, &j.orient.y, &j.orient.z)==6)
                {
                    j.orient.restorew();
                    j.scale = Vec3(1, 1, 1);
                    baseframe.add(j);
                }
            }
            if(baseframe.length()!=hierarchy.length()) { delete f; return false; }
            eposes.reserve(animframes*baseframe.length());
            eposes.advance(animframes*baseframe.length());
        }
        else if(sscanf(buf, " frame %d", &tmp)==1)
        {
            for(int numdata = 0; f->getline(buf, sizeof(buf)) && buf[0]!='}';)
            {
                for(char *src = buf, *next = src; numdata < animdatalen; numdata++, src = next)
                {
                    animdata[numdata] = strtod(src, &next);
                    if(next <= src) break;
                }
            }
            int offset = frameoffset + tmp*baseframe.length();
            eframes.add(offset);
            loopv(baseframe)
            {
                md5hierarchy &h = hierarchy[i];
                transform j = baseframe[i];
                if(h.start < animdatalen && h.flags)
                {
                    double *jdata = &animdata[h.start];
                    if(h.flags&1) j.pos.x = *jdata++;
                    if(h.flags&2) j.pos.y = *jdata++;
                    if(h.flags&4) j.pos.z = *jdata++;
                    if(h.flags&8) j.orient.x = *jdata++;
                    if(h.flags&16) j.orient.y = *jdata++;
                    if(h.flags&32) j.orient.z = *jdata++;
                    j.orient.restorew();
                }
                eposes[offset + i] = j;
            }
        }
    }

    if(animdata) delete[] animdata;
    delete f;

    eanim &a = eanims.add();
    if(spec.name) a.name = getnamekey(spec.name);
    else
    {
        string name;
        copystring(name, filename);
        char *end = strrchr(name, '.');
        if(end) *end = '\0';
        a.name = getnamekey(name);
    }
    a.startframe = firstframe;
    a.fps = spec.fps > 0 ? spec.fps : framerate;
    a.flags = spec.flags;
    if(spec.endframe >= 0) a.endframe = a.startframe + spec.endframe;
    a.startframe += spec.startframe;

    makeanims();

    return true;
}

namespace smd
{

bool skipcomment(char *&curbuf)
{
    while(*curbuf && isspace(*curbuf)) curbuf++;
    switch(*curbuf)
    {
        case '#':
        case ';':
        case '\r':
        case '\n':
        case '\0':
            return true;
        case '/':
            if(curbuf[1] == '/') return true;
            break;
    }
    return false;
}

void skipsection(stream *f, char *buf, size_t bufsize)
{
    while(f->getline(buf, bufsize))
    {
        char *curbuf = buf;
        if(skipcomment(curbuf)) continue;
        if(!strncmp(curbuf, "end", 3)) break;
    }
}

void readname(char *&curbuf, char *name, size_t namesize)
{
    char *curname = name;
    while(*curbuf && isspace(*curbuf)) curbuf++;
    bool allowspace = false;
    if(*curbuf == '"') { curbuf++; allowspace = true; }
    while(*curbuf)
    {
        char c = *curbuf++;
        if(c == '"') break;
        if(isspace(c) && !allowspace) break;
        if(curname < &name[namesize-1]) *curname++ = c;
    }
    *curname = '\0';
}

void readnodes(stream *f, char *buf, size_t bufsize)
{
    while(f->getline(buf, bufsize))
    {
        char *curbuf = buf;
        if(skipcomment(curbuf)) continue;
        if(!strncmp(curbuf, "end", 3)) break;
        int id = strtol(curbuf, &curbuf, 10);
        string name;
        readname(curbuf, name, sizeof(name));
        int parent = strtol(curbuf, &curbuf, 10);
        if(id < 0 || id > 255 || parent > 255 || !name[0] || (ejoints.inrange(id) && ejoints[id].name)) continue;
        ejoint j;
        j.name = getnamekey(name);
        j.parent = parent;
        while(ejoints.length() <= id) ejoints.add();
        ejoints[id] = j;
    }
}

void readmaterial(char *&curbuf, char *mat, char *name, size_t matsize)
{
    char *curmat = mat;
    while(*curbuf && isspace(*curbuf)) curbuf++;
    char *ext = NULL;
    while(*curbuf)
    {
        char c = *curbuf++;
        if(isspace(c)) break;
        if(c == '.' && !ext) ext = curmat;
        if(curmat < &mat[matsize-1]) *curmat++ = c;
    }
    *curmat = '\0';
    if(!ext) ext = curmat;
    memcpy(name, mat, ext - mat);
    name[ext - mat] = '\0';
}

void readskeleton(stream *f, char *buf, size_t bufsize)
{
    int frame = -1, firstpose = -1;
    while(f->getline(buf, bufsize))
    {
        char *curbuf = buf;
        if(skipcomment(curbuf)) continue;
        if(sscanf(curbuf, " time %d", &frame) == 1) continue;
        else if(!strncmp(curbuf, "end", 3)) break;
        else if(frame != 0) continue;
        int bone;
        Vec3 pos, rot;
        if(sscanf(curbuf, " %d %lf %lf %lf %lf %lf %lf", &bone, &pos.x, &pos.y, &pos.z, &rot.x, &rot.y, &rot.z) != 7)
            continue;
        if(!ejoints.inrange(bone))
            continue;
        if(firstpose < 0)
        {
            firstpose = eposes.length();
            eposes.reserve(ejoints.length());
            eposes.advance(ejoints.length());
        }
        double cx = cos(rot.x/2), sx = sin(rot.x/2),
               cy = cos(rot.y/2), sy = sin(rot.y/2),
               cz = cos(rot.z/2), sz = sin(rot.z/2);
        transform p(pos, Quat(sx*cy*cz - cx*sy*sz,
                              cx*sy*cz + sx*cy*sz,
                              cx*cy*sz - sx*sy*cz,
                              cx*cy*cz + sx*sy*sz));
        if(p.orient.w > 0) p.orient.flip();
        eposes[firstpose + bone] = p;
    }
}

void readtriangles(stream *f, char *buf, size_t bufsize)
{
    emesh m;
    while(f->getline(buf, bufsize))
    {
        char *curbuf = buf;
        if(skipcomment(curbuf)) continue;
        if(!strncmp(curbuf, "end", 3)) break;
        string name, material;
        readmaterial(curbuf, material, name, sizeof(material));
        if(!m.name || strcmp(m.name, name))
        {
            if(m.name && etriangles.length() > m.firsttri) emeshes.add(m);
            m.name = getnamekey(name);
            m.material = getnamekey(material);
            m.firsttri = etriangles.length();
        }
        Vec4 *pos = epositions.reserve(3) + 2, *tc = etexcoords.reserve(3) + 2;
        Vec3 *norm = enormals.reserve(3) + 2;
        blendcombo *c = eblends.reserve(3) + 2;
        loopi(3)
        {
            char *curbuf;
            do
            {
                if(!f->getline(buf, bufsize)) goto endsection;
                curbuf = buf;
            } while(skipcomment(curbuf));
            int parent = -1, numlinks = 0, len = 0;
            if(sscanf(curbuf, " %d %lf %lf %lf %lf %lf %lf %lf %lf %d%n", &parent, &pos->x, &pos->y, &pos->z, &norm->x, &norm->y, &norm->z, &tc->x, &tc->y, &numlinks, &len) < 9) goto endsection;
            curbuf += len;
            pos->w = 1;
            tc->y = 1 - tc->y;
            tc->z = tc->w = 0;
            c->reset();
            double pweight = 0, tweight = 0;
            for(; numlinks > 0; numlinks--)
            {
                int bone = -1, len = 0;
                double weight = 0;
                if(sscanf(curbuf, " %d %lf%n", &bone, &weight, &len) < 2) break;
                curbuf += len;
                tweight += weight;
                if(bone == parent) pweight += weight;
                else c->addweight(weight, bone);
            }
            if(tweight < 1) pweight += 1 - tweight;
            if(pweight > 0) c->addweight(pweight, parent);
            c->finalize();
            --pos;
            --tc;
            --norm;
            --c;
        }
        etriangle &t = etriangles.add();
        loopi(3) t.vert[i] = epositions.length() + i;
        t.smoothgroup = 0;
        epositions.advance(3);
        enormals.advance(3);
        etexcoords.advance(3);
        eblends.advance(3);
    }
endsection:
    if(m.name && etriangles.length () > m.firsttri) emeshes.add(m);
}

int readframes(stream *f, char *buf, size_t bufsize)
{
    int frame = -1, numframes = 0, lastbone = ejoints.length(), frameoffset = eposes.length();
    while(f->getline(buf, bufsize))
    {
        char *curbuf = buf;
        if(skipcomment(curbuf)) continue;
        int nextframe = -1;
        if(sscanf(curbuf, " time %d", &nextframe) == 1)
        {
            for(; lastbone < ejoints.length(); lastbone++) eposes[frameoffset + frame*ejoints.length() + lastbone] = eposes[frameoffset + lastbone];
            if(nextframe >= numframes)
            {
                eposes.reserve(ejoints.length() * (nextframe + 1 - numframes));
                loopi(nextframe - numframes) 
                {
                    eframes.add(eposes.length());
                    eposes.put(&eposes[frameoffset], ejoints.length());
                }
                eframes.add(eposes.length());
                eposes.advance(ejoints.length());
                numframes = nextframe + 1;
            }
            frame = nextframe;
            lastbone = 0;
            continue;
        }
        else if(!strncmp(curbuf, "end", 3)) break;
        int bone;
        Vec3 pos, rot;
        if(sscanf(curbuf, " %d %lf %lf %lf %lf %lf %lf", &bone, &pos.x, &pos.y, &pos.z, &rot.x, &rot.y, &rot.z) != 7)
            continue;
        if(bone < 0 || bone >= ejoints.length())
            continue;
        for(; lastbone < bone; lastbone++) eposes[frameoffset + frame*ejoints.length() + lastbone] = eposes[frameoffset + lastbone];
        lastbone++;
        double cx = cos(rot.x/2), sx = sin(rot.x/2),
               cy = cos(rot.y/2), sy = sin(rot.y/2),
               cz = cos(rot.z/2), sz = sin(rot.z/2);
        transform p(pos,
                    Quat(sx*cy*cz - cx*sy*sz,
                         cx*sy*cz + sx*cy*sz,
                         cx*cy*sz - sx*sy*cz,
                         cx*cy*cz + sx*sy*sz));
        if(p.orient.w > 0) p.orient.flip();
        eposes[frameoffset + frame*ejoints.length() + bone] = p;
    }
    for(; lastbone < ejoints.length(); lastbone++) eposes[frameoffset + frame*ejoints.length() + lastbone] = eposes[frameoffset + lastbone];
    return numframes;
}

}

bool loadsmd(const char *filename, const filespec &spec)
{
    stream *f = openfile(filename, "r");
    if(!f) return false;

    resetimporter();

    char buf[512];
    int version = -1, firstframe = eframes.length();
    bool hastriangles = false;
    while(f->getline(buf, sizeof(buf)))
    {
        char *curbuf = buf;
        if(smd::skipcomment(curbuf)) continue;
        if(sscanf(curbuf, " version %d", &version) == 1)
        {
            if(version != 1) { delete f; return false; }
        }
        else if(!strncmp(curbuf, "nodes", 5))
            smd::readnodes(f, buf, sizeof(buf));
        else if(!strncmp(curbuf, "triangles", 9))
        {
            smd::readtriangles(f, buf, sizeof(buf));
            hastriangles = true;
        }
        else if(!strncmp(curbuf, "skeleton", 8))
            smd::readframes(f, buf, sizeof(buf));
        else if(!strncmp(curbuf, "vertexanimation", 15))
            smd::skipsection(f, buf, sizeof(buf));
    }

    delete f;

    if(poses.length() && ejoints.length() && poses.length() != ejoints.length()) return false;

    if(hastriangles)
    {
        eframes.setsize(firstframe);
        smoothverts();
        makemeshes();
        makeanims();
    }
    else
    {
        eanim &a = eanims.add();
        if(spec.name) a.name = getnamekey(spec.name);
        else
        {
            string name;
            copystring(name, filename);
            char *end = strrchr(name, '.');
            if(end) *end = '\0';
            a.name = getnamekey(name);
        }
        a.startframe = firstframe;
        a.fps = spec.fps;
        a.flags = spec.flags;
        if(spec.endframe >= 0) a.endframe = a.startframe + spec.endframe;
        a.startframe += spec.startframe;
        makeanims();
    }

    return true;
}

int framesize = 0;
vector<ushort> animdata;

void calcanimdata()
{
    if(frames.length()) loopv(poses)
    {
        pose &j = poses[i];
        loopk(10) { j.offset[k] = 1e16f; j.scale[k] = -1e16f; }
    }
    loopv(frames)
    {
        pose &j = poses[i%poses.length()];
        transform &f = frames[i];
        loopk(3) 
        {
            j.offset[k] = min(j.offset[k], float(f.pos[k]));
            j.scale[k] = max(j.scale[k], float(f.pos[k]));
        }
        loopk(4)
        {
            j.offset[3+k] = min(j.offset[3+k], float(f.orient[k]));
            j.scale[3+k] = max(j.scale[3+k], float(f.orient[k]));
        }
        loopk(3)
        {
            j.offset[7+k] = min(j.offset[7+k], float(f.scale[k]));
            j.scale[7+k] = max(j.scale[7+k], float(f.scale[k]));
        }
    }
    loopv(poses)
    {
        pose &j = poses[i];
        loopk(10) 
        {
            j.scale[k] -= j.offset[k];
            if(j.scale[k] >= 1e-10f) { framesize++; j.scale[k] /= 0xFFFF; j.flags |= 1<<k; } 
            else j.scale[k] = 0.0f;
        }
    }
#if 0
    int runlength = 0, blocksize = 0, blocks = 0;
    #define FLUSHVAL(val) \
        if(!blocksize || (animdata.last() == val ? runlength >= 0xFF : runlength || blocksize > 0xFF)) \
        { \
            animdata.add(0); \
            animdata.add(val); \
            blocksize = 1; \
            runlength = 0; \
            blocks++; \
        } \
        else if(animdata.last() == val) \
        { \
            animdata[animdata.length()-blocksize-1] += 0x10; \
            runlength++; \
        } \
        else \
        { \
            animdata[animdata.length()-blocksize-1]++; \
            animdata.add(val); \
            blocksize++; \
        }
    loopv(joints)
    {
        joint &j = joints[i];
        loopk(3) if(j.flags & (0x01<<k))
        {
            for(int l = i; l < frames.length(); l += poses.length())
            {
                transform &f = frames[l];
                ushort val = ushort((f.pos[k] - j.offset[k]) / j.scale[k]);
                FLUSHVAL(val);
            }
        }
        loopk(4) if(j.flags & (0x08<<k))
        {
            for(int l = i; l < frames.length(); l += poses.length())
            {
                transform &f = frames[l];
                ushort val = ushort((f.orient[k] - j.offset[3+k]) / j.scale[3+k]);
                FLUSHVAL(val);
            }
        }
        loopk(3) if(j.flags & (0x80<<k))
        {
            for(int l = i; l < frames.length(); l += poses.length())
            {
                transform &f = frames[l];
                ushort val = ushort((f.scale[k] - j.offset[7+k]) / j.scale[7+k]);
                FLUSHVAL(val);
            }
        }
    }                    
    printf("%d frames of size %d/%d compressed from %d/%d to %d in %d blocks", frames.length()/poses.length(), framesize, poses.length()*9, framesize*frames.length()/poses.length(), poses.length()*9*frames.length()/poses.length(), animdata.length(), blocks);
#else
    loopv(frames)
    {
        pose &j = poses[i%poses.length()];
        transform &f = frames[i];
        loopk(3) if(j.flags & (0x01<<k)) animdata.add(ushort((float(f.pos[k]) - j.offset[k]) / j.scale[k]));
        loopk(4) if(j.flags & (0x08<<k)) animdata.add(ushort((float(f.orient[k]) - j.offset[3+k]) / j.scale[3+k]));
        loopk(3) if(j.flags & (0x80<<k)) animdata.add(ushort((float(f.scale[k]) - j.offset[7+k]) / j.scale[7+k]));
    }
#endif
    while(vdata.length()%4) vdata.add(0);
    while(stringdata.length()%4) stringdata.add('\0');
    while(commentdata.length()%4) commentdata.add('\0');
    while(animdata.length()%2) animdata.add(0);
}

bool writeiqm(const char *filename)
{
    stream *f = openfile(filename, "wb");
    if(!f) return false;

    iqmheader hdr;
    memset(&hdr, 0, sizeof(hdr));
    copystring(hdr.magic, IQM_MAGIC, sizeof(hdr.magic)); 
    hdr.filesize = sizeof(hdr);
    hdr.version = IQM_VERSION;
    if(stringdata.length()) hdr.ofs_text = hdr.filesize; hdr.num_text = stringdata.length(); hdr.filesize += hdr.num_text;
    hdr.num_meshes = meshes.length(); if(meshes.length()) hdr.ofs_meshes = hdr.filesize; hdr.filesize += meshes.length() * sizeof(mesh);
    uint voffset = hdr.filesize + varrays.length() * sizeof(vertexarray);
    hdr.num_vertexarrays = varrays.length(); if(varrays.length()) hdr.ofs_vertexarrays = hdr.filesize; hdr.filesize += varrays.length() * sizeof(vertexarray);
    uint valign = (8 - (hdr.filesize%8))%8;
    voffset += valign;
    hdr.filesize += valign + vdata.length();
    hdr.num_vertexes = vmap.length();
    hdr.num_triangles = triangles.length(); if(triangles.length()) hdr.ofs_triangles = hdr.filesize; hdr.filesize += triangles.length() * sizeof(triangle);
    if(neighbors.length()) hdr.ofs_adjacency = hdr.filesize; hdr.filesize += neighbors.length() * sizeof(triangle);
    hdr.num_joints = joints.length(); if(joints.length()) hdr.ofs_joints = hdr.filesize; hdr.filesize += joints.length() * sizeof(joint);
    hdr.num_poses = poses.length(); if(poses.length()) hdr.ofs_poses = hdr.filesize; hdr.filesize += poses.length() * sizeof(pose);
    hdr.num_anims = anims.length(); if(anims.length()) hdr.ofs_anims = hdr.filesize; hdr.filesize += anims.length() * sizeof(anim);
    hdr.num_frames = poses.length() ? frames.length()/poses.length() : 0; hdr.num_framechannels = framesize; 
    if(animdata.length()) hdr.ofs_frames = hdr.filesize; hdr.filesize += animdata.length() * sizeof(ushort); 
    if(bounds.length()) hdr.ofs_bounds = hdr.filesize; hdr.filesize += bounds.length() * sizeof(float[8]);
    if(commentdata.length()) hdr.ofs_comment = hdr.filesize; hdr.num_comment = commentdata.length(); hdr.filesize += hdr.num_comment;

    lilswap(&hdr.version, (sizeof(hdr) - sizeof(hdr.magic))/sizeof(uint));
    f->write(&hdr, sizeof(hdr));

    if(stringdata.length()) f->write(stringdata.getbuf(), stringdata.length());

    loopv(meshes)
    {
        mesh &m = meshes[i];
        f->putlil(m.name);
        f->putlil(m.material);
        f->putlil(m.firstvert);
        f->putlil(m.numverts);
        f->putlil(m.firsttri);
        f->putlil(m.numtris);
    }

    loopv(varrays)
    {
        vertexarray &v = varrays[i];
        f->putlil(v.type); 
        f->putlil(v.flags);
        f->putlil(v.format);
        f->putlil(v.size);
        f->putlil(voffset + v.offset);
    }

    loopi(valign) f->putchar(0);
    f->write(vdata.getbuf(), vdata.length());

    loopv(triangles)
    {
        triangle &t = triangles[i];
        loopk(3) f->putlil(t.vert[k]);
    }

    loopv(neighbors)
    {
        triangle &t = neighbors[i];
        loopk(3) f->putlil(t.vert[k]);
    }

    loopv(joints)
    {
        joint &j = joints[i];
        f->putlil(j.name);
        f->putlil(j.parent);
        loopk(3) f->putlil(float(j.pos[k]));
        loopk(4) f->putlil(float(j.orient[k]));
        loopk(3) f->putlil(float(j.scale[k]));
    }

    loopv(poses)
    {
        pose &p = poses[i];
        f->putlil(p.parent);
        f->putlil(p.flags);
        loopk(10) f->putlil(p.offset[k]);
        loopk(10) f->putlil(p.scale[k]);
    }

    loopv(anims)
    {
        anim &a = anims[i];
        f->putlil(a.name);
        f->putlil(a.firstframe);
        f->putlil(a.numframes);
        f->putlil(a.fps);
        f->putlil(a.flags);
    }

    loopv(animdata) f->putlil(animdata[i]);

    loopv(bounds)
    {
        framebounds &b = bounds[i];
        loopk(3) f->putlil(float(b.bbmin[k]));
        loopk(3) f->putlil(float(b.bbmax[k]));
        f->putlil(float(b.xyradius));
        f->putlil(float(b.radius));
    }

    if(commentdata.length()) f->write(commentdata.getbuf(), commentdata.length());

    delete f;
    return true;
}

int main(int argc, char **argv)
{
    if(argc < 1)
        return EXIT_FAILURE;

    vector<filespec> infiles;
    filespec inspec;
    const char *outfile = NULL;
    for(int i = 1; i < argc; i++)
    {
        if(argv[i][0] == '-') 
        {
            if(argv[i][1] == '-')
            {
                if(!strcasecmp(&argv[i][2], "fps")) { if(i + 1 < argc) inspec.fps = atof(argv[++i]); }
                else if(!strcasecmp(&argv[i][2], "name")) { if(i + 1 < argc) inspec.name = argv[++i]; }
                else if(!strcasecmp(&argv[i][2], "loop")) { inspec.flags |= IQM_LOOP; }
                else if(!strcasecmp(&argv[i][2], "start")) { if(i + 1 < argc) inspec.startframe = max(atoi(argv[++i]), 0); }
                else if(!strcasecmp(&argv[i][2], "end")) { if(i + 1 < argc) inspec.endframe = max(atoi(argv[++i]), 0); }
                else if(!strcasecmp(&argv[i][2], "scale")) { if(i + 1 < argc) escale = clamp(atof(argv[++i]), 1e-8, 1e8); }
            }
            else switch(argv[i][1])
            {
            case 's':
                if(i + 1 < argc) escale = clamp(atof(argv[++i]), 1e-8, 1e8);
                break;
            }
        }
        else if(!outfile) outfile = argv[i];
        else 
        {
            infiles.add(inspec).file = argv[i];
            inspec.reset();
        }
    }

    if(!outfile) fatal("no output file specified");
    else if(infiles.empty()) fatal("no input files specified");

    if(escale != 1) printf("scale: %f\n", escale);

    loopv(infiles)
    {
        const filespec &inspec = infiles[i];
        const char *infile = inspec.file, *type = strrchr(infile, '.');
        if(!type) fatal("no file type: %s", infile);
		else if(!strcasecmp(type, ".md5mesh"))
        {
			if(loadmd5mesh(infile, inspec)) conoutf("imported: %s", infile);
			else fatal("failed reading: %s", infile);
		}
		else if(!strcasecmp(type, ".md5anim"))
		{
			if(loadmd5anim(infile, inspec)) conoutf("imported: %s", infile);
			else fatal("failed reading: %s", infile);
		}
        else if(!strcasecmp(type, ".iqe"))
        {
            if(loadiqe(infile, inspec)) conoutf("imported: %s", infile);
            else fatal("failed reading: %s", infile);
        }
        else if(!strcasecmp(type, ".smd"))
        {
            if(loadsmd(infile, inspec)) conoutf("imported: %s", infile);
            else fatal("failed reading: %s", infile);
        }
		else fatal("unknown file type: %s", type);	 
	}

	calcanimdata();

    conoutf("");

	if(writeiqm(outfile)) conoutf("exported: %s", outfile);
	else fatal("failed writing: %s", outfile);

	return EXIT_SUCCESS;
}

