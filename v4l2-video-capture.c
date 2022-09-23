/**
 * @file v4l2_video_capture.c
 *
 * This is small utility making use of basic v4l2 api.
 * It was written only for my educational reasons but now as it works I thought
 * I can upload it to github, so it also could serve its educational goals for others.
 *
 * @author Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 */

/*===========================================================================*\
 * system header files
\*===========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

/*===========================================================================*\
 * project header files
\*===========================================================================*/

/*===========================================================================*\
 * preprocessor #define constants and macros
\*===========================================================================*/
#define V4l2_SELECT_TIMEOUT_SEC 10

/*===========================================================================*\
 * local type definitions
\*===========================================================================*/
struct v4l2_buffer_descriptor
{
    int index;
    void* addr;
    size_t size;
    uint32_t offset;
};

enum v4l2_buffer_sharing_mode
{
    V4L2_BUFFER_SHARING_MODE_MMAP,
    V4L2_BUFFER_SHARING_MODE_DMA
};

/*===========================================================================*\
 * global object definitions
\*===========================================================================*/

/*===========================================================================*\
 * local function declarations
\*===========================================================================*/
static void v4l2_print_usage(const char* progname);
static const char* v4l2_capabilities_to_string(char* buf, size_t size, uint32_t capabilities);
static const char* v4l2_buf_type_to_string(enum v4l2_buf_type buf_type);
static const char* v4l2_frmsizetype_to_string(enum v4l2_frmsizetypes type);
static const char* v4l2_frmivaltype_to_string(enum v4l2_frmivaltypes type);
static void v4l2_print_capabilities(const struct v4l2_capability* caps);
static void v4l2_print_fmtdesc(const struct v4l2_fmtdesc* fmtdesc);
static void v4l2_print_frmsizeenum(const struct v4l2_frmsizeenum* frmsizeenum);
static void v4l2_print_frmivalenum(const struct v4l2_frmivalenum* frmivalenum);
static void v4l2_print_cropping_capabilities(const struct v4l2_cropcap* cropcap);
static void v4l2_print_format(const struct v4l2_format* format);
static void v4l2_print_buffer(const struct v4l2_buffer* buffer);
static void v4l2_print_control(int fd, const struct v4l2_query_ext_ctrl* qextctrl);

static uint32_t v4l2_query_capabilities(int fd, uint32_t flags);
static void v4l2_query_controls(int fd);
static int v4l2_query_buffers(int fd, int number_of_buffers);
static int v4l2_queue_buffers(int fd, int number_of_buffers);
static int v4l2_capture_frame(int fd, void** frame_buffer, size_t* frame_size);
static void v4l2_store_frame(const uint8_t* image, uint32_t fourcc, size_t size, int counter);
static int v4l2_video_capture(int fd, int number_of_frames);

/*===========================================================================*\
 * local object definitions
\*===========================================================================*/
static const char* output_directory;
static struct v4l2_format selected_format;
static struct v4l2_buffer_descriptor* buffer_descriptors;
static enum v4l2_buffer_sharing_mode buffer_sharing_mode = V4L2_BUFFER_SHARING_MODE_DMA;

/*===========================================================================*\
 * inline function definitions
\*===========================================================================*/

/*===========================================================================*\
 * public function definitions
\*===========================================================================*/
int main(int argc, char *argv[])
{
    int fd;
    uint32_t capabilities;
    int number_of_frames = 1;
    int number_of_buffers = 1;
    bool use_compressed_formats = false;

    static struct option long_options[] = {
        {"number-of-frames",       required_argument, 0, 'n'},
        {"number-of-buffers",      required_argument, 0, 'b'},
        {"use-compressed-formats", no_argument,       0, 'c'},
        {"output-directory",       no_argument,       0, 'o'},
        {0, 0, 0, 0}
    };

    for (;;) {
        int c = getopt_long(argc, argv, "n:b:co:", long_options, 0);
        if (-1 == c)
            break;

        switch (c) {
            case 'n':
                number_of_frames = atoi(optarg);
                break;

            case 'b':
                number_of_buffers = atoi(optarg);
                break;

            case 'c':
                use_compressed_formats = true;
                break;

            case 'o':
                output_directory = optarg;
                break;

            default:
                /* do nothing */
                break;
        }
    }

    if (number_of_frames < 1)
        number_of_frames = 1;

    if (number_of_buffers < 1)
        number_of_buffers = 1;

    if (output_directory == NULL)
        output_directory = ".";

    const char* filename = argv[optind];
    if (!filename) {
        fprintf(stderr, "device filename is not provided\n");
        v4l2_print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    fd = open(filename, O_RDWR);
    if (-1 == fd) {
        fprintf(stderr, "cannot open '%s': %s\n", filename, strerror(errno));
        v4l2_print_usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    memset(&selected_format, 0, sizeof(selected_format));

    capabilities = v4l2_query_capabilities(fd, use_compressed_formats ? V4L2_FMT_FLAG_COMPRESSED : 0);
    if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s do not support video capture or streaming\n", filename);
        exit(EXIT_FAILURE);
    }

    v4l2_query_controls(fd);

    if (selected_format.type == 0) {
        fprintf(stderr, "No frame format is selected for capturing\n");
        exit(EXIT_FAILURE);
    }

    v4l2_print_format(&selected_format);

    if (-1 == ioctl(fd, VIDIOC_S_FMT, &selected_format)) {
        fprintf(stderr, "VIDIOC_S_FMT failed: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    number_of_buffers = v4l2_query_buffers(fd, number_of_buffers);
    if (number_of_buffers < 0) {
        fprintf(stderr, "v4l2_query_buffers() failed\n");
        exit(EXIT_FAILURE);
    }

    if (v4l2_queue_buffers(fd, number_of_buffers)) {
        fprintf(stderr, "v4l2_queue_buffers() failed\n");
        exit(EXIT_FAILURE);
    }

    if (v4l2_video_capture(fd, number_of_frames)) {
        fprintf(stderr, "v4l2_capture_image() failed\n");
        exit(EXIT_FAILURE);
    }

    close(fd);
    return 0;
}

/*===========================================================================*\
 * local function definitions
\*===========================================================================*/
static void v4l2_print_usage(const char* progname)
{
    fprintf(stdout, "usage: %s [-n <frames>] [-b <buffers>] [-c] [-o <output-directory>] <filename>\n", progname);
    fprintf(stdout, " options:\n");
    fprintf(stdout, "  -n <frames>  --number-of-frames=<frames>   : number of frames to be captured (default: 1)\n");
    fprintf(stdout, "  -b <buffers> --number-of-buffers=<buffers> : number of buffers to be allocated for capturing (default: 1)\n");
    fprintf(stdout, "  -c --use-compressed-formats                : if set, capturing will search for compressed formats\n");
    fprintf(stdout, "  -o --output-directory                      : if set, specifies directory for captured frames\n");
    fprintf(stdout, "  <filename>                                 : capturing device (e.g. /dev/video0)\n");
}

static const char* v4l2_capabilities_to_string(char* buf, size_t size, uint32_t capabilities)
{
    size_t i;
    int n;
    char *p = buf;

    static const char* caps[] = {
        "V4L2_CAP_VIDEO_CAPTURE",
        "V4L2_CAP_VIDEO_OUTPUT",
        "V4L2_CAP_VIDEO_OVERLAY",
        "UNKNOWN_0x00000008",
        "V4L2_CAP_VBI_CAPTURE",
        "V4L2_CAP_VBI_OUTPUT",
        "V4L2_CAP_SLICED_VBI_CAPTURE",
        "V4L2_CAP_SLICED_VBI_OUTPUT",
        "V4L2_CAP_RDS_CAPTURE",
        "V4L2_CAP_VIDEO_OUTPUT_OVERLAY",
        "V4L2_CAP_HW_FREQ_SEEK",
        "V4L2_CAP_RDS_OUTPUT",
        "V4L2_CAP_VIDEO_CAPTURE_MPLANE",
        "V4L2_CAP_VIDEO_OUTPUT_MPLANE",
        "V4L2_CAP_VIDEO_M2M_MPLANE",
        "V4L2_CAP_VIDEO_M2M",
        "V4L2_CAP_TUNER",
        "V4L2_CAP_AUDIO",
        "V4L2_CAP_RADIO",
        "V4L2_CAP_MODULATOR",
        "V4L2_CAP_SDR_CAPTURE",
        "V4L2_CAP_EXT_PIX_FORMAT",
        "V4L2_CAP_SDR_OUTPUT",
        "V4L2_CAP_META_CAPTURE",
        "V4L2_CAP_READWRITE",
        "V4L2_CAP_ASYNCIO",
        "V4L2_CAP_STREAMING",
        "V4L2_CAP_META_OUTPUT",
        "V4L2_CAP_TOUCH",
        "UNKNOWN_0x20000000",
        "UNKNOWN_0x40000000",
        "V4L2_CAP_DEVICE_CAPS",
    };

    memset(p, 0, size);

    for (i = 0; i < (sizeof(caps) / sizeof(caps[0])); ++i)
        if (capabilities & (1U << i)) {
            n = snprintf(p, size, "\t\t%s\n", caps[i]);
            if (n < 0)
                return NULL;
            if ((size_t)n >= size)
                break;
            p += n;
            size -= n;
        }

    return buf;
}

static const char* v4l2_buf_type_to_string(enum v4l2_buf_type buf_type)
{
    static const char* buf_types[] = {
        [0]                                  = "0",
        [V4L2_BUF_TYPE_VIDEO_CAPTURE]        = "V4L2_BUF_TYPE_VIDEO_CAPTURE",
        [V4L2_BUF_TYPE_VIDEO_OVERLAY]        = "V4L2_BUF_TYPE_VIDEO_OVERLAY",
        [V4L2_BUF_TYPE_VIDEO_OUTPUT]         = "V4L2_BUF_TYPE_VIDEO_OUTPUT",
        [V4L2_BUF_TYPE_VBI_CAPTURE]          = "V4L2_BUF_TYPE_VBI_CAPTURE",
        [V4L2_BUF_TYPE_VBI_OUTPUT]           = "V4L2_BUF_TYPE_VBI_OUTPUT",
        [V4L2_BUF_TYPE_SLICED_VBI_CAPTURE]   = "V4L2_BUF_TYPE_SLICED_VBI_CAPTURE",
        [V4L2_BUF_TYPE_SLICED_VBI_OUTPUT]    = "V4L2_BUF_TYPE_SLICED_VBI_OUTPUT",
        [V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY] = "V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY",
        [V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE] = "V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE",
        [V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE]  = "V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE",
        [V4L2_BUF_TYPE_SDR_CAPTURE]          = "V4L2_BUF_TYPE_SDR_CAPTURE",
        [V4L2_BUF_TYPE_SDR_OUTPUT]           = "V4L2_BUF_TYPE_SDR_OUTPUT",
    };

    if (buf_type >= (sizeof(buf_types) / sizeof(buf_types[0])))
        buf_type = 0;

    return buf_types[buf_type];
}

static const char* v4l2_memory_to_string(enum v4l2_memory memory)
{
	static const char* memories[] = {
		[0]                    = "0",
		[V4L2_MEMORY_MMAP]     = "V4L2_MEMORY_MMAP",
		[V4L2_MEMORY_USERPTR]  = "V4L2_MEMORY_USERPTR",
		[V4L2_MEMORY_OVERLAY]  = "V4L2_MEMORY_OVERLAY",
		[V4L2_MEMORY_DMABUF]   = "V4L2_MEMORY_DMABUF",
	};

	if (memory >= (sizeof(memories) / sizeof(memories[0])))
		memory = 0;

	return memories[memory];
}

static const char* v4l2_frmsizetype_to_string(enum v4l2_frmsizetypes type)
{
    static const char* types[] = {
        [0]                              = "0",
        [V4L2_FRMSIZE_TYPE_DISCRETE]     = "V4L2_FRMSIZE_TYPE_DISCRETE",
        [V4L2_FRMSIZE_TYPE_CONTINUOUS]   = "V4L2_FRMSIZE_TYPE_CONTINUOUS",
        [V4L2_FRMSIZE_TYPE_STEPWISE]     = "V4L2_FRMSIZE_TYPE_STEPWISE",
    };

    if (type >= (sizeof(types) / sizeof(types[0])))
        type = 0;

    return types[type];
}

static const char* v4l2_frmivaltype_to_string(enum v4l2_frmivaltypes type)
{
    static const char* types[] = {
        [0]                              = "0",
        [V4L2_FRMIVAL_TYPE_DISCRETE]     = "V4L2_FRMIVAL_TYPE_DISCRETE",
        [V4L2_FRMIVAL_TYPE_CONTINUOUS]   = "V4L2_FRMIVAL_TYPE_CONTINUOUS",
        [V4L2_FRMIVAL_TYPE_STEPWISE]     = "V4L2_FRMIVAL_TYPE_STEPWISE",
    };

    if (type >= (sizeof(types) / sizeof(types[0])))
        type = 0;

    return types[type];
}

static void v4l2_print_capabilities(const struct v4l2_capability* caps)
{
    char buf1[1024];
    char buf2[1024];

    fprintf(stdout,
        "v4l2_capability:\n"
        "\tdriver       : %s\n"
        "\tcard         : %s\n"
        "\tbus_info     : %s\n"
        "\tversion      : 0x%08x\n"
        "\tcapabilities : 0x%08x\n"
        "%s"
        "\tdevice_caps  : 0x%08x\n"
        "%s",
        caps->driver,
        caps->card,
        caps->bus_info,
        caps->version,
        caps->capabilities,
        v4l2_capabilities_to_string(buf1, sizeof(buf1), caps->capabilities),
        caps->device_caps,
        v4l2_capabilities_to_string(buf2, sizeof(buf2), caps->device_caps)
        );
}

static void v4l2_print_fmtdesc(const struct v4l2_fmtdesc* fmtdesc)
{
    fprintf(stdout,
        "v4l2_fmtdesc:\n"
        "\tindex       : %u\n"
        "\ttype        : %s\n"
        "\tflags       : 0x%08x\n"
        "\tdescription : %s\n"
        "\tpixelformat : '%c%c%c%c'\n",
        fmtdesc->index,
        v4l2_buf_type_to_string(fmtdesc->type),
        fmtdesc->flags,
        fmtdesc->description,
        (fmtdesc->pixelformat >>  0) & 0xff,
        (fmtdesc->pixelformat >>  8) & 0xff,
        (fmtdesc->pixelformat >> 16) & 0xff,
        (fmtdesc->pixelformat >> 24) & 0xff
        );
}

static void v4l2_print_frmsizeenum(const struct v4l2_frmsizeenum* frmsizeenum)
{
    fprintf(stdout,
        "\tv4l2_frmsizeenum:\n"
        "\t\tindex       : %u\n"
        "\t\tpixelformat : '%c%c%c%c'\n"
        "\t\ttype        : %s\n",
        frmsizeenum->index,
        (frmsizeenum->pixel_format >>  0) & 0xff,
        (frmsizeenum->pixel_format >>  8) & 0xff,
        (frmsizeenum->pixel_format >> 16) & 0xff,
        (frmsizeenum->pixel_format >> 24) & 0xff,
        v4l2_frmsizetype_to_string(frmsizeenum->type)
        );

    if (frmsizeenum->type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        fprintf(stdout,
            "\t\tdiscrete    : width: %u, height: %u\n",
            frmsizeenum->discrete.width,
            frmsizeenum->discrete.height
            );
    } else
    if (frmsizeenum->type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        fprintf(stdout,
            "\t\tstepwise    : width: [%u:%u:%u], height: [%u:%u:%u]\n",
            frmsizeenum->stepwise.min_width,
            frmsizeenum->stepwise.max_width,
            frmsizeenum->stepwise.step_width,
            frmsizeenum->stepwise.min_height,
            frmsizeenum->stepwise.max_height,
            frmsizeenum->stepwise.step_height
            );
    } else {
        /* do nothing */
    }
}

static void v4l2_print_frmivalenum(const struct v4l2_frmivalenum* frmivalenum)
{
    fprintf(stdout,
        "\t\tv4l2_frmivalenum:\n"
        "\t\t\tindex       : %u\n"
        "\t\t\tpixelformat : '%c%c%c%c'\n"
        "\t\t\twidth       : %u\n"
        "\t\t\theight      : %u\n"
        "\t\t\ttype        : %s\n",
        frmivalenum->index,
        (frmivalenum->pixel_format >>  0) & 0xff,
        (frmivalenum->pixel_format >>  8) & 0xff,
        (frmivalenum->pixel_format >> 16) & 0xff,
        (frmivalenum->pixel_format >> 24) & 0xff,
        frmivalenum->width,
        frmivalenum->height,
        v4l2_frmivaltype_to_string(frmivalenum->type)
        );

    if (frmivalenum->type == V4L2_FRMIVAL_TYPE_DISCRETE) {
        fprintf(stdout,
            "\t\t\tdiscrete    : %u/%u\n",
            frmivalenum->discrete.numerator,
            frmivalenum->discrete.denominator
            );
    } else
    if (frmivalenum->type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        fprintf(stdout,
            "\t\t\tstepwise    : min: %u/%u, max: %u/%u, step: %u/%u\n",
            frmivalenum->stepwise.min.numerator,
            frmivalenum->stepwise.min.denominator,
            frmivalenum->stepwise.max.numerator,
            frmivalenum->stepwise.max.denominator,
            frmivalenum->stepwise.step.numerator,
            frmivalenum->stepwise.step.denominator
            );
    } else {
        /* do nothing */
    }
}

static void v4l2_print_cropping_capabilities(const struct v4l2_cropcap* cropcap)
{
    fprintf(stdout,
        "v4l2_cropcap:\n"
        "\tbounds      : left: %d, top: %d, width: %u, height: %u\n"
        "\tdefrect     : left: %d, top: %d, width: %u, height: %u\n"
        "\tpixelaspect : numerator: %u, denominator: %u\n",
        cropcap->bounds.left, cropcap->bounds.top, cropcap->bounds.width, cropcap->bounds.height,
        cropcap->defrect.left, cropcap->defrect.top, cropcap->defrect.width, cropcap->defrect.height,
        cropcap->pixelaspect.numerator, cropcap->pixelaspect.denominator
        );
}

static void v4l2_print_format(const struct v4l2_format* format)
{
    fprintf(stdout,
        "v4l2_format:\n"
        "\ttype        : %s\n",
        v4l2_buf_type_to_string(format->type)
        );
    if (V4L2_TYPE_IS_MULTIPLANAR(format->type)) {
        fprintf(stdout,
            "\twidth       : %u\n"
            "\theight      : %u\n"
            "\tpixelformat : '%c%c%c%c'\n"
            "\tfield       : %u\n"
            "\tcolorspace  : %u\n"
            "\tnum_planes  : %u\n",
            format->fmt.pix_mp.width,
            format->fmt.pix_mp.height,
            (format->fmt.pix_mp.pixelformat >>  0) & 0xff,
            (format->fmt.pix_mp.pixelformat >>  8) & 0xff,
            (format->fmt.pix_mp.pixelformat >> 16) & 0xff,
            (format->fmt.pix_mp.pixelformat >> 24) & 0xff,
            format->fmt.pix_mp.field,
            format->fmt.pix_mp.colorspace,
            format->fmt.pix_mp.num_planes
        );
    }
    else {
        fprintf(stdout,
            "\twidth       : %u\n"
            "\theight      : %u\n"
            "\tpixelformat : '%c%c%c%c'\n"
            "\tfield       : %u\n"
            "\tbytesperline: %u\n"
            "\tsizeimage   : %u\n"
            "\tcolorspace  : %u\n",
            format->fmt.pix.width,
            format->fmt.pix.height,
            (format->fmt.pix.pixelformat >>  0) & 0xff,
            (format->fmt.pix.pixelformat >>  8) & 0xff,
            (format->fmt.pix.pixelformat >> 16) & 0xff,
            (format->fmt.pix.pixelformat >> 24) & 0xff,
            format->fmt.pix.field,
            format->fmt.pix.bytesperline,
            format->fmt.pix.sizeimage,
            format->fmt.pix.colorspace
        );
    }
}

static void v4l2_print_buffer(const struct v4l2_buffer* buffer)
{
    fprintf(stdout,
        "v4l2_buffer:\n"
        "\tindex       : %u\n"
        "\ttype        : %s\n"
        "\tbyteused    : %u\n"
        "\tflags       : 0x%08x\n"
        "\tfield       : %u\n"
        "\tsequence    : %u\n"
        "\tmemory      : %s\n"
        "\tlength      : %u\n",
        buffer->index,
        v4l2_buf_type_to_string(buffer->type),
        buffer->bytesused,
        buffer->flags,
        buffer->field,
        buffer->sequence,
        v4l2_memory_to_string(buffer->memory),
        buffer->length
    );
    if (V4L2_TYPE_IS_MULTIPLANAR(buffer->type)) {
        unsigned i;
        for (i = 0; i < buffer->length; ++i) {
            fprintf(stdout,
                "\t\tbytesused : %u\n"
                "\t\tlength    : %u\n",
                buffer->m.planes[i].bytesused,
                buffer->m.planes[i].length
            );
            if (buffer->memory == V4L2_MEMORY_MMAP) {
                fprintf(stdout,
                    "\t\tmem_offset  : %u\n",
                    buffer->m.planes[i].m.mem_offset
                );
            } else
            if (buffer->memory == V4L2_MEMORY_USERPTR) {
                fprintf(stdout,
                    "\t\tuserptr     : %lu\n",
                    buffer->m.planes[i].m.userptr
                );
            } else
            if (buffer->memory == V4L2_MEMORY_DMABUF) {
                fprintf(stdout,
                    "\t\tfd          : %d\n",
                    buffer->m.planes[i].m.fd
                );
            } else {
                /* do nothing */
            }
        }
    }
    else {
        if (buffer->memory == V4L2_MEMORY_MMAP) {
            fprintf(stdout,
                "\toffset      : %u\n",
                buffer->m.offset
            );
        } else
        if (buffer->memory == V4L2_MEMORY_USERPTR) {
            fprintf(stdout,
                "\tuserptr     : %lu\n",
                buffer->m.userptr
            );
        } else
        if (buffer->memory == V4L2_MEMORY_DMABUF) {
            fprintf(stdout,
                "\tfd          : %d\n",
                buffer->m.fd
            );
        } else {
            /* do nothing */
        }
    }
}

static const char* v4l2_ctrl_type_to_string(enum v4l2_ctrl_type ctrl_type)
{
    const char* str = "Unknown control type";

    switch (ctrl_type) {
        case V4L2_CTRL_TYPE_INTEGER: str = "V4L2_CTRL_TYPE_INTEGER"; break;
        case V4L2_CTRL_TYPE_BOOLEAN: str = "V4L2_CTRL_TYPE_BOOLEAN"; break;
        case V4L2_CTRL_TYPE_MENU: str = "V4L2_CTRL_TYPE_MENU"; break;
        case V4L2_CTRL_TYPE_BUTTON: str = "V4L2_CTRL_TYPE_BUTTON"; break;
        case V4L2_CTRL_TYPE_INTEGER64: str = "V4L2_CTRL_TYPE_INTEGER64"; break;
        case V4L2_CTRL_TYPE_CTRL_CLASS: str = "V4L2_CTRL_TYPE_CTRL_CLASS"; break;
        case V4L2_CTRL_TYPE_STRING: str = "V4L2_CTRL_TYPE_STRING"; break;
        case V4L2_CTRL_TYPE_BITMASK: str = "V4L2_CTRL_TYPE_BITMASK"; break;
        case V4L2_CTRL_TYPE_INTEGER_MENU: str = "V4L2_CTRL_TYPE_INTEGER_MENU"; break;
        case V4L2_CTRL_TYPE_U8: str = "V4L2_CTRL_TYPE_U8"; break;
        case V4L2_CTRL_TYPE_U16: str = "V4L2_CTRL_TYPE_U16"; break;
        case V4L2_CTRL_TYPE_U32: str = "V4L2_CTRL_TYPE_U32"; break;

        default:
            break;
    }

    return str;
}

static void v4l2_print_control(int fd, const struct v4l2_query_ext_ctrl* qextctrl)
{
    do {
        int status;
        struct v4l2_ext_control extctrl;
        struct v4l2_ext_controls extctrls;

        fprintf(stdout,
            "\tid: 0x%08x, type: %s (%u), name: %s\n"
            "\t\tmin/max/step : %lld/%lld/%llu\n"
            "\t\tdefault      : %lld\n"
            "\t\tflags        : 0x%08x\n",
            qextctrl->id,
            v4l2_ctrl_type_to_string(qextctrl->type),
            qextctrl->type,
            qextctrl->name,
            qextctrl->minimum,
            qextctrl->maximum,
            qextctrl->step,
            qextctrl->default_value,
            qextctrl->flags
            );

        if ((qextctrl->flags & V4L2_CTRL_FLAG_WRITE_ONLY) || (qextctrl->type == V4L2_CTRL_TYPE_BUTTON))
            break;

        memset(&extctrl, 0, sizeof(extctrl));
        memset(&extctrls, 0, sizeof(extctrls));

        extctrl.id = qextctrl->id;
        if (qextctrl->flags & V4L2_CTRL_FLAG_HAS_PAYLOAD) {
            extctrl.size = qextctrl->elems * qextctrl->elem_size;
            extctrl.ptr = alloca(extctrl.size);
        }

        extctrls.which = V4L2_CTRL_ID2WHICH(qextctrl->id);
        extctrls.count = 1;
        extctrls.controls = &extctrl;

        status = ioctl(fd, VIDIOC_G_EXT_CTRLS, &extctrls);
        if (-1 == status) {
            fprintf(stderr, "VIDIOC_G_EXT_CTRLS failed: %s\n", strerror(errno));
            break;
        }

        if (qextctrl->nr_of_dims == 0) {
            switch (qextctrl->type) {
                case V4L2_CTRL_TYPE_U8:
                    fprintf(stdout, "\t\tvalue        : %u\n", *extctrl.p_u8);
                    break;
                case V4L2_CTRL_TYPE_U16:
                    fprintf(stdout, "\t\tvalue        : %u\n", *extctrl.p_u16);
                    break;
                case V4L2_CTRL_TYPE_U32:
                    fprintf(stdout, "\t\tvalue        : %u\n", *extctrl.p_u32);
                    break;
                case V4L2_CTRL_TYPE_STRING:
                    fprintf(stdout, "\t\tvalue        : '%s'\n", extctrl.string);
                    break;
                case V4L2_CTRL_TYPE_INTEGER64:
                    fprintf(stdout, "\t\tvalue        : %lld\n", extctrl.value64);
                    break;
                default:
                    fprintf(stdout, "\t\tvalue        : %d\n", extctrl.value);
                    break;
            }
        } else {
            unsigned i;
            fprintf(stdout, "\t\tdims: ");
            for (i = 0; i < qextctrl->nr_of_dims; ++i)
                fprintf(stdout, "[%u]", qextctrl->dims[i]);
            fprintf(stdout, "\n");
        }

    } while (0);
}

static uint32_t v4l2_query_capabilities(int fd, uint32_t flags)
{
    uint32_t capabilities = 0;

    do {
        int status;
        struct v4l2_capability caps;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_cropcap cropcap;
        struct v4l2_frmsizeenum frmsizeenum;
        struct v4l2_frmivalenum frmivalenum;

        memset(&caps, 0, sizeof(caps));
        status = ioctl(fd, VIDIOC_QUERYCAP, &caps);
        if (-1 == status) {
            fprintf(stderr, "VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
            break;
        }

        capabilities = caps.capabilities;
        v4l2_print_capabilities(&caps);

        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.index = 0;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (; 0 == (status = ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)); fmtdesc.index++) {
            v4l2_print_fmtdesc(&fmtdesc);

            memset(&frmsizeenum, 0, sizeof(frmsizeenum));
            frmsizeenum.index = 0;
            frmsizeenum.pixel_format = fmtdesc.pixelformat;
            for (; 0 == (status = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum)); frmsizeenum.index++) {
                v4l2_print_frmsizeenum(&frmsizeenum);

                if (V4L2_FRMSIZE_TYPE_DISCRETE == frmsizeenum.type) {
                    memset(&frmivalenum, 0, sizeof(frmivalenum));
                    frmivalenum.index = 0;
                    frmivalenum.pixel_format = frmsizeenum.pixel_format;
                    frmivalenum.width = frmsizeenum.discrete.width;
                    frmivalenum.height = frmsizeenum.discrete.height;
                    for (; 0 == (status = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum)); frmivalenum.index++)
                        v4l2_print_frmivalenum(&frmivalenum);
                    if (-1 == status && errno != EINVAL)
                        fprintf(stderr, "VIDIOC_ENUM_FRAMEINTERVALS failed: %s\n", strerror(errno));
                }
                else
                if (V4L2_FRMSIZE_TYPE_STEPWISE == frmsizeenum.type) {
                    memset(&frmivalenum, 0, sizeof(frmivalenum));
                    frmivalenum.index = 0;
                    frmivalenum.pixel_format = frmsizeenum.pixel_format;
                    frmivalenum.width = frmsizeenum.stepwise.max_width;
                    frmivalenum.height = frmsizeenum.stepwise.max_height;
                    for (; 0 == (status = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum)); frmivalenum.index++)
                        v4l2_print_frmivalenum(&frmivalenum);
                    if (-1 == status && errno != EINVAL)
                        fprintf(stderr, "VIDIOC_ENUM_FRAMEINTERVALS failed: %s\n", strerror(errno));
                }
                else
                    continue;

                if (selected_format.type == 0) {
                    if ((flags ^ fmtdesc.flags) == 0) {
                        selected_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        selected_format.fmt.pix.pixelformat = frmsizeenum.pixel_format;

                        if (V4L2_FRMSIZE_TYPE_DISCRETE == frmsizeenum.type) {
                            selected_format.fmt.pix.width = frmsizeenum.discrete.width;
                            selected_format.fmt.pix.height = frmsizeenum.discrete.height;
                        }
                        else
                        if (V4L2_FRMSIZE_TYPE_STEPWISE == frmsizeenum.type) {
                            selected_format.fmt.pix.width = frmsizeenum.stepwise.min_width;
                            selected_format.fmt.pix.height = frmsizeenum.stepwise.min_height;
                        }
                        else {
                            selected_format.fmt.pix.width = 0;
                            selected_format.fmt.pix.height = 0;
                        }
                    }
                }
            }
            if (-1 == status && errno != EINVAL)
                fprintf(stderr, "VIDIOC_ENUM_FRAMESIZES failed: %s\n", strerror(errno));
        }
        if (-1 == status && errno != EINVAL)
            fprintf(stderr, "VIDIOC_ENUM_FMT failed: %s\n", strerror(errno));

        memset(&cropcap, 0, sizeof(cropcap));
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        status = ioctl(fd, VIDIOC_CROPCAP, &cropcap);
        if (-1 == status) {
            fprintf(stderr, "VIDIOC_CROPCAP failed: %s\n", strerror(errno));
            break;
        }

        v4l2_print_cropping_capabilities(&cropcap);
    } while (0);

    return capabilities;
}

static void v4l2_query_controls(int fd)
{
    int status;
    struct v4l2_query_ext_ctrl qextctrl;

    fprintf(stdout, "VIDIOC_QUERY_EXT_CTRL:\n");

    memset(&qextctrl, 0, sizeof(qextctrl));
    qextctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;

    do {
        status = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qextctrl);
        if (-1 == status) {
            if (errno != EINVAL)
                fprintf(stderr, "VIDIOC_QUERY_EXT_CTRL failed: %s\n", strerror(errno));
            break;
        }

        if (qextctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            continue;

        v4l2_print_control(fd, &qextctrl);
        qextctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    } while (1);

    memset(&qextctrl, 0, sizeof(qextctrl));
    qextctrl.id = V4L2_CID_PRIVATE_BASE;

    do {
        status = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qextctrl);
        if (-1 == status) {
            if (errno != EINVAL)
                fprintf(stderr, "VIDIOC_QUERY_EXT_CTRL failed: %s\n", strerror(errno));
            break;
        }

        if (qextctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            continue;

        v4l2_print_control(fd, &qextctrl);
        qextctrl.id++;
    } while (1);


    memset(&qextctrl, 0, sizeof(qextctrl));
    qextctrl.id = V4L2_CID_USER_BASE;

    do {
        status = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qextctrl);
        if (-1 == status) {
            if (errno != EINVAL)
                fprintf(stderr, "VIDIOC_QUERY_EXT_CTRL failed: %s\n", strerror(errno));
            break;
        }

        if (qextctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            continue;

        v4l2_print_control(fd, &qextctrl);
        qextctrl.id++;
    } while (qextctrl.id < V4L2_CID_LASTP1);
}

static int v4l2_query_buffers(int fd, int number_of_buffers)
{
    int retval = -1;

    do {
        struct v4l2_requestbuffers requestbuffers;
        uint32_t i;

        memset(&requestbuffers, 0, sizeof(requestbuffers));
        requestbuffers.count = number_of_buffers;
        requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requestbuffers.memory = V4L2_MEMORY_MMAP;

        if (-1 == ioctl(fd, VIDIOC_REQBUFS, &requestbuffers)) {
            fprintf(stderr, "VIDIOC_REQBUFS failed: %s\n", strerror(errno));
            break;
        }

        fprintf(stdout,
            "VIDIOC_REQBUFS:\n"
            "\trequested count: %u, commited count: %u, caps: 0x%08x\n",
            number_of_buffers, requestbuffers.count, requestbuffers.capabilities
            );

        buffer_descriptors = calloc(requestbuffers.count, sizeof(*buffer_descriptors));
        if (NULL == buffer_descriptors) {
            fprintf(stderr, "calloc(%u, %zu) failed\n",
                requestbuffers.count, sizeof(*buffer_descriptors));
            break;
        }

        for (i = 0; i < requestbuffers.count; ++i) {
            struct v4l2_buffer buffer;
            struct v4l2_buffer_descriptor* bd;
            void* addr;

            memset(&buffer, 0, sizeof(buffer));
            buffer.index = i;
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            if(-1 == ioctl(fd, VIDIOC_QUERYBUF, &buffer)) {
                fprintf(stderr, "VIDIOC_QUERYBUF[%d] failed: %s\n", i, strerror(errno));
                break;
            }

            fprintf(stdout, "VIDIOC_QUERYBUF[%u]:\n", i);
            v4l2_print_buffer(&buffer);

            addr = mmap(
                NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
            if (MAP_FAILED == addr) {
                fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
                break;
            }

            bd = buffer_descriptors + i;
            bd->index = i;
            bd->addr = addr;
            bd->size = buffer.length;
            bd->offset = buffer.m.offset;
        }

        if (i < requestbuffers.count)
            break;

        retval = requestbuffers.count;
    } while (0);

    return retval;
}

static int v4l2_queue_buffers(int fd, int number_of_buffers)
{
    int retval = -1;

    do {
        int i;

        for (i = 0; i < number_of_buffers; ++i) {
            struct v4l2_buffer buffer;

            memset(&buffer, 0, sizeof(buffer));
            buffer.index = i;
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            if(-1 == ioctl(fd, VIDIOC_QBUF, &buffer)) {
                fprintf(stderr, "VIDIOC_QBUF[%d] failed: %s\n", i, strerror(errno));
                break;
            }
        }

        if (i < number_of_buffers)
            break;

        retval = 0;
    } while (0);

    return retval;
}

static int v4l2_capture_frame(int fd, void** frame_buffer, size_t* frame_size)
{
    int retval = -1;

    do {
        int status;
        struct v4l2_buffer buffer;
        fd_set fds;
        struct timespec ts;
        uint32_t index;
        uint32_t flags;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        ts.tv_sec = V4l2_SELECT_TIMEOUT_SEC;
        ts.tv_nsec = 0;
        status = pselect(fd+1, &fds, NULL, NULL, &ts, NULL);
        if (-1 == status) {
            fprintf(stderr, "pselect() failed: %s\n", strerror(errno));
            break;
        } else
        if (0 == status) {
            fprintf(stderr, "no data within %d seconds, timeout expired\n", V4l2_SELECT_TIMEOUT_SEC);
            break;
        }

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        if(-1 == ioctl(fd, VIDIOC_DQBUF, &buffer)) {
            fprintf(stderr, "VIDIOC_DQBUF failed: %s\n", strerror(errno));
            break;
        }

        fprintf(stdout, "VIDIOC_DQBUF:\n");
        v4l2_print_buffer(&buffer);

        index = buffer.index;
        flags = buffer.flags;

        if (0 == (flags & V4L2_BUF_FLAG_ERROR)) {
            if (frame_buffer)
                *frame_buffer = buffer_descriptors[index].addr;
            if (frame_size)
                *frame_size = buffer.bytesused;
        }

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;

        if(-1 == ioctl(fd, VIDIOC_QBUF, &buffer)) {
            fprintf(stderr, "VIDIOC_QBUF failed: %s\n", strerror(errno));
            break;
        }

        if (flags & V4L2_BUF_FLAG_ERROR) {
            fprintf(stderr, "Received erroneous frame for buffer[%u]\n", index);
            break;
        }

        retval = 0;
    } while (0);

    return retval;
}

static void v4l2_store_frame(const uint8_t* image, uint32_t fourcc, size_t size, int counter)
{
    char image_filename[256];
    int fd = -1;

    do {
        int n;

        n = snprintf(image_filename, sizeof(image_filename), "%s/image%04d.%c%c%c%c",
            output_directory,
            counter,
            (fourcc >>  0) & 0xff,
            (fourcc >>  8) & 0xff,
            (fourcc >> 16) & 0xff,
            (fourcc >> 24) & 0xff
        );

        if (n < 0)
            break;
        if ((size_t)n >= sizeof(image_filename))
            break;

        fd = open(image_filename, O_WRONLY | O_CREAT | O_TRUNC, 0664);
        if (-1 == fd){
            fprintf(stderr, "cannot open '%s': %s\n", image_filename, strerror(errno));
            break;
        }

        if (-1 == write(fd, image, size)) {
            fprintf(stderr, "write() failed: %s\n", strerror(errno));
            break;
        }
    } while (0);

    if (fd != -1)
        close(fd);
}

static int v4l2_video_capture(int fd, int number_of_frames)
{
    uint32_t type;
    void* frame_buffer;
    size_t frame_size;
    int i;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == ioctl(fd, VIDIOC_STREAMON, &type)) {
        fprintf(stderr, "VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return -1;
    }

    i = 0;
    while (i < number_of_frames) {
        if (0 == v4l2_capture_frame(fd, &frame_buffer, &frame_size)) {
            v4l2_store_frame(frame_buffer, selected_format.fmt.pix.pixelformat, frame_size, i + 1);
            i++;
        }
    }

    if(-1 == ioctl(fd, VIDIOC_STREAMOFF, &type)) {
        fprintf(stderr, "VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}
