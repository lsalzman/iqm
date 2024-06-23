#include "util.h"

iqmheader hdr;
vector<iqmvertexarray> vertexarrays;
vector<char> str, comment;
vector<iqmmesh> meshes;
vector<iqmjoint> joints;
vector<iqmpose> poses;
vector<iqmanim> anims;
vector<iqmtriangle> triangles, adjacency;
vector<iqmbounds> bounds;
vector<ushort> frames;
vector<uchar> vdata;

const int fmtsize[] = { 1, 1, 2, 2, 4, 4, 2, 4, 8 };

bool fixtangents()
{
    float *pos = NULL, *tc = NULL, *norm = NULL, *tan = NULL;
    loopv(vertexarrays)
    {
        iqmvertexarray &va = vertexarrays[i];
        switch(va.type)
        {
        case IQM_POSITION: if(va.format != IQM_FLOAT || va.size != 3) return false; pos = (float *)&vdata[va.offset]; break;
        case IQM_NORMAL: if(va.format != IQM_FLOAT || va.size != 3) return false; norm = (float *)&vdata[va.offset]; break;
        case IQM_TANGENT: if(va.format != IQM_FLOAT || va.size != 4) return false; tan = (float *)&vdata[va.offset]; break;
        case IQM_TEXCOORD: if(va.format != IQM_FLOAT || va.size != 2) return false; tc = (float *)&vdata[va.offset]; break;
        }
    }
    if(!pos || !tc || !norm || !tan) return false;
    float *svec = new float[3*hdr.num_vertexes], *tvec = new float[3*hdr.num_vertexes];
    memset(svec, 0, 3*hdr.num_vertexes*sizeof(float));
    memset(tvec, 0, 3*hdr.num_vertexes*sizeof(float));
    #define loadf(out, in, comps) { memcpy(out, in, comps*sizeof(float)); lilswap(out, comps); }
    #define add3f(v, n) { v[0] += n[0], v[1] += n[1]; v[2] += n[2]; }
    #define sub3f(v, n) { v[0] -= n[0], v[1] -= n[1]; v[2] -= n[2]; }
    #define sub2f(v, n) { v[0] -= n[0], v[1] -= n[1]; }
    #define neg3f(v) { v[0] = -v[0]; v[1] = -v[1]; v[2] = -v[2]; } 
    #define dot3f(v, o) (v[0]*o[0] + v[1]*o[1] + v[2]*o[2])
    #define cross3f(a, b) { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] }
    #define norm3f(v, n) { float invlen = 1.0f/sqrtf(dot3f(n, n)); v[0] = n[0]*invlen; v[1] = n[1]*invlen; v[2] = n[2]*invlen; }
    #define project3f(v, a, b) { float d = dot3f(a, b); v[0] = a[0] - b[0]*d; v[1] = a[1] - b[1]*d; v[2] = a[2] - b[2]*d; }
    loopv(triangles)
    {
        const iqmtriangle &tri = triangles[i];
        float e0[3], e1[3], e2[3];
        loadf(e0, &pos[3*tri.vertex[0]], 3);
        loadf(e1, &pos[3*tri.vertex[1]], 3);
        loadf(e2, &pos[3*tri.vertex[2]], 3);
        sub3f(e1, e0);
        sub3f(e2, e0);
        float tc0[2], tc1[2], tc2[2];
        loadf(tc0, &tc[2*tri.vertex[0]], 2);
        loadf(tc1, &tc[2*tri.vertex[1]], 2);
        loadf(tc2, &tc[2*tri.vertex[2]], 2);
        sub2f(tc1, tc0);
        sub2f(tc2, tc0);
        float s[3] = { e2[0]*tc1[1] - e1[0]*tc2[1], e2[1]*tc1[1] - e1[1]*tc2[1], e2[2]*tc1[1] - e1[2]*tc2[1] },
              t[3] = { e2[0]*tc1[0] - e1[0]*tc2[0], e2[1]*tc1[0] - e1[1]*tc2[0], e2[2]*tc1[0] - e1[2]*tc2[0] },
              n[3] = cross3f(e2, e1),
              c[3] = cross3f(t, s);
        if(dot3f(c, n) < 0) 
        { 
            neg3f(s); 
            neg3f(t); 
        }
        loopk(3)
        {
            float *sk = &svec[3*tri.vertex[k]], *tk = &tvec[3*tri.vertex[k]];
            add3f(sk, s);
            add3f(tk, t);
        }
    }
    loopi(hdr.num_vertexes)
    {
        float n[3], *s = &svec[3*i], *t = &tvec[3*i], *out = &tan[4*i];
        loadf(n, &norm[3*i], 3);
        project3f(out, s, n);
        norm3f(out, out);
        float c[3] = cross3f(n, s);
        out[3] = dot3f(c, t) < 0 ? -1 : 1;
    }
    lilswap(tan, 4*hdr.num_vertexes);
    delete[] svec;
    delete[] tvec;
    return true;
}

bool loadiqmmeshes(const char *infile, uchar *buf)
{
    lilswap((uint *)&buf[hdr.ofs_vertexarrays], hdr.num_vertexarrays*sizeof(iqmvertexarray)/sizeof(uint));
    vertexarrays.put((iqmvertexarray *)&buf[hdr.ofs_vertexarrays], hdr.num_vertexarrays);

    loopv(vertexarrays)
    {
        iqmvertexarray &va = vertexarrays[i];
        uint offset = vdata.length();
        uint align = max(fmtsize[va.format], 4);
        if(offset%align) { uint pad = align - offset%align; offset += pad; loopi(pad) vdata.add(0); }
        vdata.put(&buf[va.offset], fmtsize[va.format] * va.size * hdr.num_vertexes);
        va.offset = offset;
    }

    lilswap((uint *)&buf[hdr.ofs_triangles], hdr.num_triangles*sizeof(iqmtriangle)/sizeof(uint));
    triangles.put((iqmtriangle *)&buf[hdr.ofs_triangles], hdr.num_triangles);

    lilswap((uint *)&buf[hdr.ofs_meshes], hdr.num_meshes*sizeof(iqmmesh)/sizeof(uint));
    meshes.put((iqmmesh *)&buf[hdr.ofs_meshes], hdr.num_meshes);

    if(hdr.version == 1)
    {
        iqmjointv1 *j1 = (iqmjointv1 *)&buf[hdr.ofs_joints];
        lilswap((uint *)j1, hdr.num_joints*sizeof(iqmjointv1)/sizeof(uint));
        loopi(hdr.num_joints)
        {
            iqmjoint &j = joints.add();
            j.name = j1->name;
            j.parent = j1->parent;
            loopk(3) j.translate[k] = j1->translate[k];
            loopk(3) j.rotate[k] = j1->rotate[k];
            Quat q(Vec3(j.rotate[0], j.rotate[1], j.rotate[2]));
            j.rotate[3] = q.w;
            loopk(3) j.scale[k] = j1->scale[k];
            j1++;
        }
    }
    else
    {
        lilswap((uint *)&buf[hdr.ofs_joints], hdr.num_joints*sizeof(iqmjoint)/sizeof(uint));
        joints.put((iqmjoint *)&buf[hdr.ofs_joints], hdr.num_joints);
    }

    if(hdr.ofs_adjacency) 
    {
        lilswap((uint *)&buf[hdr.ofs_adjacency], hdr.num_triangles*sizeof(iqmtriangle)/sizeof(uint));
        adjacency.put((iqmtriangle *)&buf[hdr.ofs_adjacency], hdr.num_triangles);
    }

    if(hdr.version <= 2) fixtangents();

    return true;
}

bool loadiqmanims(const char *infile, uchar *buf)
{
    if(hdr.version == 1)
    {
        iqmposev1 *p1 = (iqmposev1 *)&buf[hdr.ofs_poses];
        lilswap((uint *)p1, hdr.num_poses*sizeof(iqmposev1)/sizeof(uint));
        loopi(hdr.num_poses)
        {
            iqmpose &p = poses.add();
            p.parent = p1->parent;
            p.mask = 0;
            loopk(6) 
            {
                p.mask |= p1->mask&(1<<k);
                p.channeloffset[k] = p1->channeloffset[k];
                p.channelscale[k] = p1->channelscale[k];
            }
            p.channeloffset[6] = 1e16f;
            p.channelscale[6] = -1e16f;
            loopk(3) 
            {
                if(p1->mask&(1<<(k+6))) p.mask |= 1<<(k+7);
                p.channeloffset[k+7] = p1->channeloffset[k+6];
                p.channelscale[k+7] = p1->channelscale[k+6];
            }
            p1++;
        }
    }
    else
    {
        lilswap((uint *)&buf[hdr.ofs_poses], hdr.num_poses*sizeof(iqmpose)/sizeof(uint));
        poses.put((iqmpose *)&buf[hdr.ofs_poses], hdr.num_poses);
    }

    lilswap((uint *)&buf[hdr.ofs_anims], hdr.num_anims*sizeof(iqmanim)/sizeof(uint));
    anims.put((iqmanim *)&buf[hdr.ofs_anims], hdr.num_anims);

    lilswap((ushort *)&buf[hdr.ofs_frames], hdr.num_frames*hdr.num_framechannels);
    ushort *src = (ushort *)&buf[hdr.ofs_frames];

    if(hdr.ofs_bounds) 
    {
        lilswap((uint *)&buf[hdr.ofs_bounds], hdr.num_frames*sizeof(iqmbounds)/sizeof(uint));
        bounds.put((iqmbounds *)&buf[hdr.ofs_bounds], hdr.num_frames);
    }

    if(hdr.version == 1)
    {
        iqmposev1 *poses1 = (iqmposev1 *)&buf[hdr.ofs_poses];
        ushort *src1 = src;
        vector<Quat> quats;
        loopi(hdr.num_frames)
        {
            loopj(hdr.num_poses)
            {
                iqmposev1 &p1 = poses1[j];
                iqmpose &p = poses[j];
                if(p1.mask&0x01) src1++;
                if(p1.mask&0x02) src1++;
                if(p1.mask&0x04) src1++;
                Quat &q = quats.add();
                q.x = p1.channeloffset[3]; if(p1.mask&0x08) q.x += *src1++ * p1.channelscale[3];
                q.y = p1.channeloffset[4]; if(p1.mask&0x10) q.y += *src1++ * p1.channelscale[4];
                q.z = p1.channeloffset[5]; if(p1.mask&0x20) q.z += *src1++ * p1.channelscale[5];
                q.restorew();
                p.channeloffset[6] = min(p.channeloffset[6], float(q.w));
                p.channelscale[6] = max(p.channelscale[6], float(q.w)); 
                if(p1.mask&0x40) src1++;
                if(p1.mask&0x80) src1++;
                if(p1.mask&0x100) src1++;
            }
        }

        loopv(poses)
        {
            iqmpose &p = poses[i];
            p.channelscale[6] -= p.channeloffset[6];
            if(p.channelscale[6] >= 1e-10f) { p.channelscale[6] /= 0xFFFF; p.mask |= 1<<6; }
            else p.channelscale[6] = 0.0f;
        }

        src1 = src;
        loopi(hdr.num_frames)
        {
            loopj(hdr.num_poses)
            {
                iqmposev1 &p1 = poses1[j];
                iqmpose &p = poses[j];
                if(p1.mask&0x01) frames.add(*src1++);
                if(p1.mask&0x02) frames.add(*src1++);
                if(p1.mask&0x04) frames.add(*src1++);
                if(p1.mask&0x08) frames.add(*src1++);
                if(p1.mask&0x10) frames.add(*src1++);
                if(p1.mask&0x20) frames.add(*src1++);
                
                if(p.mask&0x40) 
                {
                    Quat &q = quats[i*hdr.num_poses + j];
                    frames.add(ushort(0.5f + (float(q.w) - p.channeloffset[6]) / p.channelscale[6]));
                }

                if(p1.mask&0x40) frames.add(*src1++);
                if(p1.mask&0x80) frames.add(*src1++);
                if(p1.mask&0x100) frames.add(*src1++);
            }
        }
        hdr.num_framechannels = frames.length()/hdr.num_frames;
    }
    else
    {
        frames.put(src, hdr.num_frames*hdr.num_framechannels);
    }

    return true;
}

bool loadiqm(const char *infile)
{
    stream *f = openfile(infile, "rb");
    if(!f) return false;

    uchar *buf = NULL;
    if(f->read(&hdr, sizeof(hdr)) != sizeof(hdr) || memcmp(hdr.magic, IQM_MAGIC, sizeof(hdr.magic)))
        goto error;
    lilswap(&hdr.version, (sizeof(hdr) - sizeof(hdr.magic))/sizeof(uint));
    if(hdr.version != 1 && hdr.version != 2)
        goto error;
    if(hdr.filesize > (16<<20))
        goto error; // sanity check... don't load files bigger than 16 MB
    buf = new uchar[hdr.filesize];
    if(f->read(buf + sizeof(hdr), hdr.filesize - sizeof(hdr)) != int(hdr.filesize - sizeof(hdr)))
        goto error;

    if(hdr.ofs_text) str.put((char *)&buf[hdr.ofs_text], hdr.num_text);
    if(hdr.ofs_comment) comment.put((char *)&buf[hdr.ofs_comment], hdr.num_comment);

    if(hdr.num_meshes > 0 && !loadiqmmeshes(infile, buf)) goto error;
    if(hdr.num_anims > 0 && !loadiqmanims(infile, buf)) goto error;

    delete[] buf;
    delete f;
    return true;

error:
    delete[] buf;
    delete f;
    return false;
}

bool writeiqm(const char *outfile)
{
    stream *f = openfile(outfile, "wb");
    if(!f) return false;

    while(vdata.length()%4) vdata.add(0);
    while(str.length()%4) str.add(0);
    while(comment.length()%4) comment.add(0);
    while(frames.length()%2) frames.add(0);

    iqmheader ohdr;
    memset(&ohdr, 0, sizeof(ohdr));
    copystring(ohdr.magic, IQM_MAGIC, sizeof(ohdr.magic));
    ohdr.filesize = sizeof(ohdr);
    ohdr.version = IQM_VERSION;

    if(str.length()) { ohdr.ofs_text = ohdr.filesize; ohdr.num_text = str.length(); ohdr.filesize += str.length(); }
    ohdr.num_meshes = meshes.length(); if(meshes.length()) { ohdr.ofs_meshes = ohdr.filesize; ohdr.filesize += meshes.length() * sizeof(iqmmesh); }

    uint voffset = ohdr.filesize + vertexarrays.length() * sizeof(iqmvertexarray);
    ohdr.num_vertexarrays = vertexarrays.length(); if(vertexarrays.length()) { ohdr.ofs_vertexarrays = ohdr.filesize; ohdr.filesize += vertexarrays.length() * sizeof(iqmvertexarray); }
    uint valign = (8 - (ohdr.filesize%8))%8;
    voffset += valign;
    ohdr.filesize += valign + vdata.length();
    ohdr.num_vertexes = hdr.num_vertexes;

    ohdr.num_triangles = triangles.length(); if(triangles.length()) { ohdr.ofs_triangles = ohdr.filesize; ohdr.filesize += triangles.length() * sizeof(iqmtriangle); }
    if(adjacency.length()) { ohdr.ofs_adjacency = ohdr.filesize; ohdr.filesize += adjacency.length() * sizeof(iqmtriangle); }

    ohdr.num_joints = joints.length(); if(joints.length()) { ohdr.ofs_joints = ohdr.filesize; ohdr.filesize += joints.length() * sizeof(iqmjoint); }

    ohdr.num_poses = poses.length(); if(poses.length()) { ohdr.ofs_poses = ohdr.filesize; ohdr.filesize += poses.length() * sizeof(iqmpose); }

    ohdr.num_anims = anims.length(); if(anims.length()) { ohdr.ofs_anims = ohdr.filesize; ohdr.filesize += anims.length() * sizeof(iqmanim); }

    ohdr.num_frames = poses.length() ? hdr.num_frames : 0; ohdr.num_framechannels = poses.length() ? hdr.num_framechannels : 0; if(frames.length()) { ohdr.ofs_frames = ohdr.filesize; ohdr.filesize += frames.length() * sizeof(ushort); }

    if(bounds.length()) { ohdr.ofs_bounds = ohdr.filesize; ohdr.filesize += bounds.length() * sizeof(iqmbounds); }

    if(comment.length()) { ohdr.ofs_comment = ohdr.filesize; ohdr.num_comment = comment.length(); ohdr.filesize += comment.length(); }

    lilswap(&ohdr.version, (sizeof(ohdr) - sizeof(ohdr.magic))/sizeof(uint));
    f->write(&ohdr, sizeof(ohdr));

    if(str.length()) f->write(str.getbuf(), str.length());

    loopv(meshes)
    {
        iqmmesh &m = meshes[i];
        f->putlil(m.name);
        f->putlil(m.material);
        f->putlil(m.first_vertex);
        f->putlil(m.num_vertexes);
        f->putlil(m.first_triangle);
        f->putlil(m.num_triangles);
    }

    loopv(vertexarrays)
    {
        iqmvertexarray &v = vertexarrays[i];
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
        iqmtriangle &t = triangles[i];
        loopk(3) f->putlil(t.vertex[k]);
    }

    loopv(adjacency)
    {
        iqmtriangle &t = adjacency[i];
        loopk(3) f->putlil(t.vertex[k]);
    }

    loopv(joints)
    {
        iqmjoint &j = joints[i];
        f->putlil(j.name);
        f->putlil(j.parent);
        loopk(3) f->putlil(j.translate[k]);
        loopk(4) f->putlil(j.rotate[k]);
        loopk(3) f->putlil(j.scale[k]);
    }

    loopv(poses)
    {
        iqmpose &p = poses[i];
        f->putlil(p.parent);
        f->putlil(p.mask);
        loopk(10) f->putlil(p.channeloffset[k]);
        loopk(10) f->putlil(p.channelscale[k]);
    }

    loopv(anims)
    {
        iqmanim &a = anims[i];
        f->putlil(a.name);
        f->putlil(a.first_frame);
        f->putlil(a.num_frames);
        f->putlil(a.framerate);
        f->putlil(a.flags);
    }

    loopv(frames) f->putlil(frames[i]);

    loopv(bounds)
    {
        iqmbounds &b = bounds[i];
        loopk(3) f->putlil(b.bbmin[k]);
        loopk(3) f->putlil(b.bbmax[k]);
        f->putlil(b.xyradius);
        f->putlil(b.radius);
    }

    if(comment.length()) f->write(comment.getbuf(), comment.length());

    delete f;
    return true;
}

int main(int argc, char **argv)
{
    if(argc < 3) fatal("Usage: %s outfile infile", argv[0]);
    const char *outfile = argv[1];
    const char *infile = argv[2];
    if(!loadiqm(infile)) fatal("failed reading: %s", infile);
    if(!writeiqm(outfile)) fatal("failed writing: %s", outfile);
    conoutf("converted %s @ v%d to %s @ v%d", infile, hdr.version, outfile, IQM_VERSION);
    return EXIT_SUCCESS;
}

