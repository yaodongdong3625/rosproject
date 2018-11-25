#ifndef __BMP_WRITER_H__
#define __BMP_WRITER_H__

#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include <memory>

struct tagBITMAPFILEHEADER {
    const uint16_t    bfType;
    const uint32_t    bfSize;
    const uint16_t    bfReserved1;
    const uint16_t    bfReserved2;
    const uint32_t    bfOffBits;
    tagBITMAPFILEHEADER(uint32_t Size = 0, uint32_t OffBits = 0);
} __attribute__((packed));

inline tagBITMAPFILEHEADER::tagBITMAPFILEHEADER(uint32_t Size, uint32_t OffBits):
    bfType('B' + 'M' * 256),
    bfSize(Size),
    bfReserved1(0), bfReserved2(0),
    bfOffBits(OffBits)
{}

struct tagBITMAPINFOHEADER {
    const uint32_t    Size;
    const int32_t     Width;
    const int32_t     Height;
    const int16_t     Planes;
    const int16_t     BitCount;
    const uint32_t    V4Compression;
    const uint32_t    SizeImage;
    const int32_t     XPelsPerMeter;
    const int32_t     YPelsPerMeter;
    const uint32_t    ClrUsed;
    const uint32_t    ClrImportant;
    tagBITMAPINFOHEADER(int32_t width = 1, int32_t height = 1, int16_t bpp = 8);
    static uint32_t CalcRowSizeOfImage(int32_t width, int16_t bpp);
    static uint32_t CalcSizeImage(int32_t width, int32_t height, int16_t bpp);

} __attribute__((packed));

inline tagBITMAPINFOHEADER::tagBITMAPINFOHEADER(int32_t width, int32_t height, int16_t bpp):
    Size(sizeof (*this)),
    Width(width),Height(-height),
    Planes(1), BitCount(bpp),
    V4Compression(0),
    SizeImage(CalcSizeImage(width, height, bpp)),
    XPelsPerMeter(2835), YPelsPerMeter(2835),
    ClrUsed(0), ClrImportant(0)
{}

inline uint32_t tagBITMAPINFOHEADER::CalcRowSizeOfImage(int32_t width, int16_t bpp) {
    uint32_t rowSize = (((width *bpp) - 1) / 32 + 1)* 32 / 8; // align to 32 bit, in byte
    return rowSize;
}

inline uint32_t tagBITMAPINFOHEADER::CalcSizeImage(int32_t width, int32_t height, int16_t bpp) {
    uint32_t rowSize = CalcRowSizeOfImage(width , bpp);
    return rowSize *height;
}

template<const uint32_t Bpp>
struct tagGreyBmpFileHead {
    static const size_t ColorTableCount =  Bpp > 8 ? 0 : (1<<Bpp);

    struct tagBITMAPFILEHEADER fileHead;
    struct tagBITMAPINFOHEADER bmpInfo;
    struct tagQUAD { uint8_t r, g, b, a;} ColorTable[ColorTableCount];

    tagGreyBmpFileHead(int32_t width, int32_t height);
}__attribute__((packed));

template<const uint32_t Bpp>
inline tagGreyBmpFileHead<Bpp>::tagGreyBmpFileHead(int32_t width, int32_t height):
    fileHead(tagBITMAPINFOHEADER::CalcSizeImage(width, height, Bpp) + sizeof (*this), sizeof (*this)),
    bmpInfo(width, height, Bpp)
{
    for (size_t i = 0; i < ColorTableCount; i++) {
        uint8_t grey = 255 / (ColorTableCount - 1) * i;
        ColorTable[i].r = grey;
        ColorTable[i].g = grey;
        ColorTable[i].b = grey;
        ColorTable[i].a = 0;
    }
}

class TBmpWriter8BppGrey
{
public:
    static const uint32_t Bpp = 8;
    TBmpWriter8BppGrey(int32_t width = 1, int32_t height = 1);
    bool  SetPixel(uint32_t w, uint32_t h, uint32_t grey);
    void _SetPixel(uint32_t w, uint32_t h, uint32_t grey);
    uint8_t *rowPtr(uint32_t h);
    void GetDataPtr(const void *&_head, size_t &head_size, const void * &body, size_t &body_size);
    virtual ~TBmpWriter8BppGrey();

    const uint32_t Width, Height;
    const uint32_t rowSizeInByte;

private:
    const tagGreyBmpFileHead<Bpp> head;
    std::vector<uint8_t>pixs;
};

inline TBmpWriter8BppGrey::TBmpWriter8BppGrey(int32_t width, int32_t height):
    Width(width), Height(height),
    rowSizeInByte(tagBITMAPINFOHEADER::CalcRowSizeOfImage(width, Bpp)),
    head(width, height),
    pixs(tagBITMAPINFOHEADER::CalcSizeImage(width, height, Bpp))
{
}

inline void TBmpWriter8BppGrey::_SetPixel(uint32_t w, uint32_t h, uint32_t grey)
{
    pixs[h * rowSizeInByte + w] = grey;
}

inline uint8_t *TBmpWriter8BppGrey::rowPtr(uint32_t h)
{
    if (h >= Height) {
        return 0;
    }
    return &pixs[h * rowSizeInByte];
}

inline void TBmpWriter8BppGrey::GetDataPtr(const void *&_head, size_t &head_size, const void *&body, size_t &body_size)
{
    _head = &head;
    head_size = sizeof head;
    body = pixs.data();
    body_size = pixs.size();
}

inline bool TBmpWriter8BppGrey::SetPixel(uint32_t w, uint32_t h, uint32_t grey)
{
    if (w >= Width || h >= Height) {
        return false;
    }
    _SetPixel(w, h, grey);
    return true;
}

inline TBmpWriter8BppGrey::~TBmpWriter8BppGrey() {}

#endif // __BMP_WRITER_H__
