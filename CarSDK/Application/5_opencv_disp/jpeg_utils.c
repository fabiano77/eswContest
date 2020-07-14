
#include <stdio.h>
#include <jpeglib.h>
#include <stdlib.h>
#include <string.h>

#include "jpeg_utils.h"
#include "util.h"

/**
  * @brief  Save rgb24 image data as jpeg file.
  * @param  rgbbuf: pointer to rgb24 image buffer
		     w: width value of source buffer
		     h : height value of source buffer
		     quality : quality of jpeg
		     filename : file name to save as jpeg
  * @retval if success, return 0 value
  */
int compress_rgb24_to_jpeg(unsigned char* rgbbuf, int w, int h, int quality, char* filename)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];

    FILE *outfile = fopen( filename, "wb" );

    if (!outfile) {
        ERROR("Can't open file!\n");
        return -1;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = w;
    cinfo.image_height = h;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);

    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        
        unsigned offset = cinfo.next_scanline * cinfo.image_width * 3;
        row_pointer[0] = &rgbbuf[offset];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    fclose(outfile);

    return 0;
}

