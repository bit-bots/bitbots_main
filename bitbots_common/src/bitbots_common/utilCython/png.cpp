#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <png.h>

#include "png.hpp"

#define abort_(...) \
{ \
    fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n"); \
    abort(); \
}

png_bytep read_png_file(const char* file_name, int* w, int* h, int* ct)
{
    unsigned char header[8];    // 8 is the maximum size that can be checked

    /* open file and test for it being a png */
    FILE *fp = fopen(file_name, "rb");
    if (!fp)
        abort_("[read_png_file] File %s could not be opened for reading", file_name);
    fread(header, 1, 8, fp);
    if (png_sig_cmp(header, 0, 8))
        abort_("[read_png_file] File %s is not recognized as a PNG file", file_name);


    /* initialize stuff */
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr)
        abort_("[read_png_file] png_create_read_struct failed");

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        abort_("[read_png_file] png_create_info_struct failed");

    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[read_png_file] Error during init_io");

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);

    png_read_info(png_ptr, info_ptr);

    int width = png_get_image_width(png_ptr, info_ptr);*w=width;
    int height = png_get_image_height(png_ptr, info_ptr);*h=height;
    int color_type = png_get_color_type(png_ptr, info_ptr);*ct=color_type;
    //int bit_depth = png_get_bit_depth(png_ptr, info_ptr);

    //int number_of_passes = png_set_interlace_handling(png_ptr);
    png_read_update_info(png_ptr, info_ptr);


    /* read file */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[read_png_file] Error during read_image");

    int rowbytes = png_get_rowbytes(png_ptr,info_ptr);
    png_bytep png_raw_bytes = new png_byte[height * rowbytes];
    png_bytep row_pointers[height];
    //row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    for (int y=0; y < height; y++)
        row_pointers[y] = png_raw_bytes + y * rowbytes;
    assert(&(png_raw_bytes[height * rowbytes]) == &(row_pointers[height - 1][rowbytes]));
    //row_pointers[y] = (png_byte*) malloc(png_get_rowbytes(png_ptr,info_ptr));

    png_read_image(png_ptr, row_pointers);

    fclose(fp);

    //png_ptr info_ptr
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    return png_raw_bytes;
}

inline int get_pixel_size(int png_color_type) {
    switch(png_color_type) {
        case(PNG_COLOR_TYPE_GRAY):
            return 1;
        case(PNG_COLOR_TYPE_RGB):
            return 3;
        case(PNG_COLOR_TYPE_RGB_ALPHA):
            return 3;
        default:
            throw std::runtime_error("Color Type unknown");
    }
}

void write_png(const char* file_name, const uint8_t* c_raw_data, int width, int height, int color_type, int bit_depth) {
    png_bytep raw_data = const_cast<png_bytep>(c_raw_data);
    /* create file */
    FILE *fp = fopen(file_name, "wb");
    if (!fp)
        abort_("[write_png_file] File %s could not be opened for writing", file_name);


    /* initialize stuff */
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr)
        abort_("[write_png_file] png_create_write_struct failed");

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        abort_("[write_png_file] png_create_info_struct failed");

    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during init_io");

    png_init_io(png_ptr, fp);


    /* write header */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during writing header");

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 bit_depth, color_type, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);


    /* write bytes */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during writing bytes");

    unsigned pixel_size = get_pixel_size(color_type);
    png_bytep row_pointers[height];
    for(int i = 0; i < height; ++i) {
        row_pointers[i] = raw_data + i * width * pixel_size;
    }
    png_write_image(png_ptr, row_pointers);


    /* end write */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during end of write");

    png_write_end(png_ptr, NULL);

    /* cleanup heap allocation */
    fclose(fp);

    png_destroy_write_struct(&png_ptr, &info_ptr);
}
