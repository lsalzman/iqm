This demo contains sample code for loading the IQM format. It requires GLUT or freeglut to build and run, and should compile on any platform for which they are available with the included Makefile. On Windows, you can just run the pre-built demo.exe and gpu-demo.exe. 

The GPU Skinning demo (gpu-demo) shows an example of how one can animate meshes stored on the GPU in vertex buffer objects using GLSL shaders.

The demos are run as follows:

./demo [options] [file.iqm] [anim.iqm]
./gpu-demo [options] [file.iqm] [anim.iqm]

If no "file.iqm" is specified, then "mrfixit.iqm" is loaded by default. If "file.iqm" contains animations, these animations will be used. If an extra "anim.iqm" is specified containing animations, these animations will be used instead along with the meshes in "file.iqm".

Options can be any of the following command-line switches:

-s N
  Displays the model with scale N.

-r N
  Rotates the model by N degrees.

-t Z
-t X,Y,Z
  Translates model by X,Y,Z units.


*** LICENSE INFO ***

The files "demo.cpp", "gpu-demo.cpp", "iqm.h", "geom.h", "util.h", "scale.h", and "texture.cpp" are licensed as specified by the LICENSE file included with this distribution.

For GL/ and freeglut file licenses, please refer to the licenses included within the files. 

The "mrfixit.iqm" model (and "Head.tga"/"Body.tga" skins) is subject to the following license:

Mr. Fixit conceptualized, modeled and animated by geartrooper a.k.a. John Siar ironantknight@gmail.com
Mr. Fixit textures, normal maps and displacement by Acord a.k.a Anthony Cord tonycord@gmail.com
mask textures by Nieb

Mr. Fixit is released under the CC-BY-NC (Creative Commons Attribution Non-Commercial) license. See http://creativecommons.org/licenses/by-nc/3.0/legalcode for more info. Please contact Geartrooper for permission for uses not within the scope of this license.


