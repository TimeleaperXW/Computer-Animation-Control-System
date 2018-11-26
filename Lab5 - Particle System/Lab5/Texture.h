
#include <GL/glut.h>
//====================================
// define the structure for texture 
//====================================

//create a structure
struct TextureTga            
{
	GLubyte *imageData;      // pointer
	GLuint bpp;				 // color depth on each pixel
	GLuint width;            // image width
	GLuint height;           // image height
	GLuint texID;            //  texture index
};
// loading file: BMP/JPG/GIF
bool LoadTexture(char *szPathName, GLuint &texid);

