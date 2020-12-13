#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void MakePalette(int whichPalette, int paletteSize, unsigned char (*palette)[3]);
void SetPalette(int whichPalette, int paletteSize, unsigned char (*palette)[3]);
int SetPixel(int x, int y, int c);
void ClearPixmap(int c);
    /* for circles, X will be scaled to aspect ratio */
void DrawFilledCircle(int cx, int cy, int r, int c, int aspX, int aspY);
void DrawLine(int x0, int y0, int x1, int y1, int c);

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif /* _GRAPHICS_H_ */
