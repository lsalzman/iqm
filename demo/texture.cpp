#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <GL/gl.h>
#include <GL/glext.h>

#include "util.h"

#define FUNCNAME(name) name##1
#define DEFPIXEL uint OP(r, 0);
#define PIXELOP OP(r, 0);
#define BPP 1
#include "scale.h"

#define FUNCNAME(name) name##2
#define DEFPIXEL uint OP(r, 0), OP(g, 1);
#define PIXELOP OP(r, 0); OP(g, 1);
#define BPP 2
#include "scale.h"

#define FUNCNAME(name) name##3
#define DEFPIXEL uint OP(r, 0), OP(g, 1), OP(b, 2);
#define PIXELOP OP(r, 0); OP(g, 1); OP(b, 2);
#define BPP 3
#include "scale.h"

#define FUNCNAME(name) name##4
#define DEFPIXEL uint OP(r, 0), OP(g, 1), OP(b, 2), OP(a, 3);
#define PIXELOP OP(r, 0); OP(g, 1); OP(b, 2); OP(a, 3);
#define BPP 4
#include "scale.h"

static void scaletexture(uchar *src, uint sw, uint sh, uint bpp, uint pitch, uchar *dst, uint dw, uint dh)
{
    if(sw == dw*2 && sh == dh*2)
    {
        switch(bpp)
        {
            case 1: return halvetexture1(src, sw, sh, pitch, dst);
            case 2: return halvetexture2(src, sw, sh, pitch, dst);
            case 3: return halvetexture3(src, sw, sh, pitch, dst);
            case 4: return halvetexture4(src, sw, sh, pitch, dst);
        }
    }
    else if(sw < dw || sh < dh || sw&(sw-1) || sh&(sh-1))
    {
        switch(bpp)
        {
            case 1: return scaletexture1(src, sw, sh, pitch, dst, dw, dh);
            case 2: return scaletexture2(src, sw, sh, pitch, dst, dw, dh);
            case 3: return scaletexture3(src, sw, sh, pitch, dst, dw, dh);
            case 4: return scaletexture4(src, sw, sh, pitch, dst, dw, dh);
        }
    }
    else
    {
        switch(bpp)
        {
            case 1: return shifttexture1(src, sw, sh, pitch, dst, dw, dh);
            case 2: return shifttexture2(src, sw, sh, pitch, dst, dw, dh);
            case 3: return shifttexture3(src, sw, sh, pitch, dst, dw, dh);
            case 4: return shifttexture4(src, sw, sh, pitch, dst, dw, dh);
        }
    }
}

static inline void bgr2rgb(uchar *data, int len, int bpp)
{
    for(uchar *end = &data[len]; data < end; data += bpp)
        swap(data[0], data[2]);
}

struct TGAHeader
{
    uchar  identsize;
    uchar  cmaptype;
    uchar  imagetype;
    uchar  cmaporigin[2]; 
    uchar  cmapsize[2];
    uchar  cmapentrysize;
    uchar  xorigin[2];
    uchar  yorigin[2];
    uchar  width[2];
    uchar  height[2];
    uchar  pixelsize;
    uchar  descbyte;
};

static uchar *loadtga(const char *fname, int &w, int &h, int &bpp)
{
    FILE *f = fopen(fname, "rb");
    if(!f) return NULL;

    uchar *data = NULL, *cmap = NULL;
    TGAHeader hdr;
    if(fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) goto error;
    if(fseek(f, hdr.identsize, SEEK_CUR) < 0) goto error;
    if(hdr.pixelsize != 8 && hdr.pixelsize != 24 && hdr.pixelsize != 32) goto error;

    bpp = hdr.pixelsize/8;
    w = hdr.width[0] + (hdr.width[1]<<8);
    h = hdr.height[0] + (hdr.height[1]<<8);

    if(hdr.imagetype==1)
    {
        int cmapsize = hdr.cmapsize[0] + (hdr.cmapsize[1]<<8);
        if(hdr.cmapentrysize!=8 || hdr.cmapentrysize!=24 || hdr.cmapentrysize!=32) goto error;
        bpp = hdr.cmapentrysize/8;
        cmap = new uchar[bpp*cmapsize];
        if((int)fread(cmap, 1, bpp*cmapsize, f) != bpp*cmapsize) goto error;
        if(bpp>=3) bgr2rgb(cmap, bpp*cmapsize, bpp);
        data = new uchar[bpp*w*h];
        uchar *idxs = &data[(bpp-1)*w*h];
        if((int)fread(idxs, 1, w*h, f) != w*h) goto error;
        uchar *src = idxs, *dst = &data[bpp*w*h];    
        for(int i = 0; i < h; i++)
        {
            dst -= bpp*w;
            uchar *row = dst;
            for(int j = 0; j < w; j++)
            {
                memcpy(row, &cmap[*src++ * bpp], bpp);
                row += bpp;
            }
        }
    }
    else if(hdr.imagetype==2)
    { 
        data = new uchar[bpp*w*h];
        uchar *dst = &data[bpp*w*h];
        for(int i = 0; i < h; i++)
        {
            dst -= bpp*w;
            if((int)fread(dst, 1, bpp*w, f) != bpp*w) goto error;
        }
        if(bpp>=3) bgr2rgb(data, bpp*w*h, bpp);
    }
    else if(hdr.imagetype==9)
    {
        int cmapsize = hdr.cmapsize[0] + (hdr.cmapsize[1]<<8);
        if(hdr.cmapentrysize!=8 || hdr.cmapentrysize!=24 || hdr.cmapentrysize!=32) goto error;
        bpp = hdr.cmapentrysize/8;
        cmap = new uchar[bpp*cmapsize];
        if((int)fread(cmap, 1, bpp*cmapsize, f) != bpp*cmapsize) goto error;
        if(bpp>=3) bgr2rgb(cmap, bpp*cmapsize, bpp);
        data = new uchar[bpp*w*h];
        uchar buf[128];
        for(uchar *end = &data[bpp*w*h], *dst = end - bpp*w; dst >= data;)
        {
            int c = fgetc(f);
            if(c==EOF) goto error;
            if(c&0x80)
            {
                int idx = fgetc(f);
                if(idx==EOF) goto error;
                const uchar *col = &cmap[idx*bpp];
                c -= 0x7F;
                c *= bpp;
                while(c > 0 && dst >= data)
                {
                    int n = min(c, int(end-dst));
                    for(uchar *run = dst+n; dst < run; dst += bpp) memcpy(dst, col, bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; }
                }
            }
            else
            {
                c += 1;
                while(c > 0 && dst >= data)
                {
                    int n = min(c, int(end-dst)/bpp);
                    if((int)fread(buf, 1, n, f) != n) goto error;
                    for(uchar *src = buf; src < &buf[n]; dst += bpp) memcpy(dst, &cmap[*src++ * bpp], bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; }
                }
            }
        }
    }
    else if(hdr.imagetype==10)
    {
        data = new uchar[bpp*w*h];
        uchar buf[4];
        for(uchar *end = &data[bpp*w*h], *dst = end - bpp*w; dst >= data;)
        {
            int c = fgetc(f);
            if(c==EOF) goto error;
            if(c&0x80)
            {
                if((int)fread(buf, 1, bpp, f) != bpp) goto error;
                c -= 0x7F;
                if(bpp>=3) swap(buf[0], buf[2]);
                c *= bpp;
                while(c > 0)
                {
                    int n = min(c, int(end-dst));
                    for(uchar *run = dst+n; dst < run; dst += bpp) memcpy(dst, buf, bpp);
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; if(dst < data) break; }
                }
            }
            else
            {
                c += 1;
                c *= bpp;
                while(c > 0)
                {
                    int n = min(c, int(end-dst));
                    if((int)fread(dst, 1, n, f) != n) goto error;
                    if(bpp>=3) bgr2rgb(dst, n, bpp);
                    dst += n;
                    c -= n;
                    if(dst >= end) { end -= bpp*w; dst = end - bpp*w; if(dst < data) break; }
                }
            }
        }
    }
    else goto error;

    if(cmap) delete[] cmap;
    fclose(f);
    return data;
    
error:
    if(data) delete[] data;
    if(cmap) delete[] cmap;
    fclose(f);
    return NULL;
}

static GLenum texformat(int bpp)
{
    switch(bpp)
    {
        case 8: return GL_LUMINANCE;
        case 16: return GL_LUMINANCE_ALPHA;
        case 24: return GL_RGB;
        case 32: return GL_RGBA;
        default: return 0;
    }
}

int formatsize(GLenum format)
{
    switch(format)
    {
        case GL_LUMINANCE:
        case GL_ALPHA: return 1;
        case GL_LUMINANCE_ALPHA: return 2;
        case GL_RGB: return 3;
        case GL_RGBA: return 4;
        default: return 4;
    }
}

void resizetexture(int w, int h, bool mipmap, GLenum target, int &tw, int &th)
{
    GLint sizelimit = 4096;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &sizelimit);
    w = min(w, sizelimit);
    h = min(h, sizelimit);
    if(mipmap || w&(w-1) || h&(h-1))
    {
        tw = th = 1;
        while(tw < w) tw *= 2;
        while(th < h) th *= 2;
        if(w < tw - tw/4) tw /= 2;
        if(h < th - th/4) th /= 2;
    }
    else
    {
        tw = w;
        th = h;
    }
}

void uploadtexture(GLenum target, GLenum internal, int tw, int th, GLenum format, GLenum type, void *pixels, int pw, int ph, bool mipmap)
{
    int bpp = formatsize(format);
    uchar *buf = NULL;
    if(pw!=tw || ph!=th)
    {
        buf = new uchar[tw*th*bpp];
        scaletexture((uchar *)pixels, pw, ph, bpp, pw*bpp, buf, tw, th);
    }
    for(int level = 0;; level++)
    {
        uchar *src = buf ? buf : (uchar *)pixels;
        if(target==GL_TEXTURE_1D) glTexImage1D(target, level, internal, tw, 0, format, type, src);
        else glTexImage2D(target, level, internal, tw, th, 0, format, type, src);
        if(!mipmap || max(tw, th) <= 1) break;
        int srcw = tw, srch = th;
        if(tw > 1) tw /= 2;
        if(th > 1) th /= 2;
        if(!buf) buf = new uchar[tw*th*bpp];
        scaletexture(src, srcw, srch, bpp, srcw*bpp, buf, tw, th);
    }
    if(buf) delete[] buf;
}

void createtexture(int tnum, int w, int h, void *pixels, int clamp, int filter, GLenum component = GL_RGB, GLenum target = GL_TEXTURE_2D, int pw = 0, int ph = 0)
{
    glBindTexture(target, tnum);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(target, GL_TEXTURE_WRAP_S, clamp&1 ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    if(target!=GL_TEXTURE_1D) glTexParameteri(target, GL_TEXTURE_WRAP_T, clamp&2 ? GL_CLAMP_TO_EDGE : GL_REPEAT);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, filter ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, filter > 1 ? GL_LINEAR_MIPMAP_LINEAR : (filter ? GL_LINEAR : GL_NEAREST));

    GLenum format = component, type = GL_UNSIGNED_BYTE;
    switch(component)
    {
        case GL_RGB5:
        case GL_RGB8:
        case GL_RGB16:
            format = GL_RGB;
            break;

        case GL_RGBA8:
        case GL_RGBA16:
            format = GL_RGBA;
            break;
    }

    if(!pw) pw = w;
    if(!ph) ph = h;
    int tw = w, th = h;
    bool mipmap = filter > 1;
    if(pixels) resizetexture(w, h, mipmap, target, tw, th);
    uploadtexture(target, component, tw, th, format, type, pixels, pw, ph, mipmap && pixels);
}

GLuint loadtexture(const char *name, int clamp)
{
    int w, h, b;
    uchar *data = loadtga(name, w, h, b); 
    if(!data) { printf("%s: failed loading\n", name); return NULL; }
    GLenum format = texformat(b*8);
    if(!format) { printf("%s: failed loading\n", name); delete[] data; return NULL; }

    GLuint tex;
    glGenTextures(1, &tex);
    createtexture(tex, w, h, data, clamp, 2, format);

    delete[] data;
    return tex;
}
 
