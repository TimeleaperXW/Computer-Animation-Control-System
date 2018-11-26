#include "stdafx.h"
#include <olectl.h>              
#include <math.h>              
#include <GL/glut.h>
#include "Texture.h"

//========================================================
// Loading an image (BMP/JPG/GIF) and turn it into texture
//========================================================

bool LoadTexture(char *szPathName, GLuint &texid)     
{
	HDC   hdcTemp;            // DC for saving bitmap
	HBITMAP  hbmpTemp;            // saving temporate bitmap
	IPicture *pPicture;            // identify IPicture Interface
	OLECHAR  wszPath[MAX_PATH + 1];         // full path of the image
	char  szPath[MAX_PATH + 1];          // full path of the image
	long  lWidth;             // width of the image
	long  lHeight;            // height of the image
	long  lWidthPixels;           // bandwidth of the image
	long  lHeightPixels;           // bandheight of the image
	GLint  glMaxTexDim;           // largest size for texture

	// get current path
	GetCurrentDirectory(MAX_PATH, szPath);      
	// adding"\"
	strcat(szPath, "\\");          
	// adding relative path
	strcat(szPath, szPathName);          
	
	// change from ASCII to Unicode
	MultiByteToWideChar(CP_ACP, 0, szPath, -1, wszPath, MAX_PATH);  // change from ASCII to Unicode
	HRESULT hr = OleLoadPicturePath(wszPath, 0, 0, 0, IID_IPicture, (void**)&pPicture);

	//if loading image failed
	if (FAILED(hr))              
	{
		// Display the error message
		MessageBox(HWND_DESKTOP, "ImageLoad Failed!", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;         
	}

	//if building DC failed
	hdcTemp = CreateCompatibleDC(GetDC(0));        
	if (!hdcTemp)              
	{
		//release IPicture
		pPicture->Release();           
		// Display the error message
		MessageBox(HWND_DESKTOP, "TextureLoad Failed!", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;        
	}

	//get the supported largest size for texture
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);    

	//get width of IPicture turn into Pixels
	pPicture->get_Width(&lWidth);          
	lWidthPixels = MulDiv(lWidth, GetDeviceCaps(hdcTemp, LOGPIXELSX), 2540);
	//get height of IPicture turn into Pixels
	pPicture->get_Height(&lHeight);          
	lHeightPixels = MulDiv(lHeight, GetDeviceCaps(hdcTemp, LOGPIXELSY), 2540);

	// Adjusting the image according to supported largest size
	if (lWidthPixels <= glMaxTexDim)         
		lWidthPixels = 1 << (int)floor((log((double)lWidthPixels) / log(2.0f)) + 0.5f);
	else                
		lWidthPixels = glMaxTexDim;

	if (lHeightPixels <= glMaxTexDim)         
		lHeightPixels = 1 << (int)floor((log((double)lHeightPixels) / log(2.0f)) + 0.5f);
	else                
		lHeightPixels = glMaxTexDim;

	// Create a temporary bitmap
	BITMAPINFO bi = { 0 };            
	DWORD  *pBits = 0;         // pointer   

	bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);   
	bi.bmiHeader.biBitCount = 32;        
	bi.bmiHeader.biWidth = lWidthPixels;       
	bi.bmiHeader.biHeight = lHeightPixels;      
	bi.bmiHeader.biCompression = BI_RGB;        
	bi.bmiHeader.biPlanes = 1;         

	// Create a bitmap for visiting color, depth and values
	hbmpTemp = CreateDIBSection(hdcTemp, &bi, DIB_RGB_COLORS, (void**)&pBits, 0, 0);

	// if failed
	if (!hbmpTemp)              
	{
		DeleteDC(hdcTemp);            
		pPicture->Release();           
		// Display the error message
		MessageBox(HWND_DESKTOP, "TextureLoad Failed!", "Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;            
	}

	//Choose temporary DC handle and bitmap object
	SelectObject(hdcTemp, hbmpTemp);         

	// Draw IPicture on Bitmap
	pPicture->Render(hdcTemp, 0, 0, lWidthPixels, lHeightPixels, 0, lHeight, lWidth, -lHeight, 0);

	// Turn BGR to RGB and set ALPHA 255
	for (long i = 0; i < lWidthPixels * lHeightPixels; i++)    
	{
		BYTE* pPixel = (BYTE*)(&pBits[i]);      
		BYTE  temp = pPixel[0];         
		pPixel[0] = pPixel[2];        
		pPixel[2] = temp;           
		pPixel[3] = 255;           
	}

	// Create the texture
	glGenTextures(1, &texid);          

	// Generate the texture using data from the bitmap
	glBindTexture(GL_TEXTURE_2D, texid);       
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // Filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // Filter
	glTexImage2D(GL_TEXTURE_2D, 0, 3, lWidthPixels, lHeightPixels, 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);


	DeleteObject(hbmpTemp);            
	DeleteDC(hdcTemp);             
	pPicture->Release();            

	return true;              
}