/* SPDX-License-Identifier: MIT */
/**
 * @file v4l2-video-capture.c
 *
 * This is small utility making use of basic v4l2 api.
 * It was written only for my educational reasons but now as it works I thought
 * I can upload it to github, so it also could serve its educational goals for others.
 *
 * @author Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>
 */

/*===========================================================================*\
 * system header files
\*===========================================================================*/
#define _GNU_SOURCE

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
#include <linux/udmabuf.h>

/*===========================================================================*\
 * project header files
\*===========================================================================*/

/*===========================================================================*\
 * preprocessor #define constants and macros
\*===========================================================================*/
#define SELECT_TIMEOUT_SEC 10
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#define MEMFD_FILE_NAME "dmabuf"
#define UDMABUF_DEVICE_NAME "/dev/udmabuf"
#define IS_POWER_OF_TWO(x) (((x) & ((x) - 1)) == 0)
#define ALIGN(x, a) __ALIGN(x, (a) - 1)
#define __ALIGN(x, mask) (((x) + (mask)) & ~(mask))

/*===========================================================================*\
 * local type definitions
\*===========================================================================*/
struct v4l2_buffer_descriptor
{
    unsigned index;
    unsigned nplanes;
    struct {
        void* addr;
        size_t size;
        int fd;
    } planes[VIDEO_MAX_PLANES];
};

enum v4l2_buffer_sharing_mode
{
    V4L2_BUFFER_SHARING_MODE_MMAP,
    V4L2_BUFFER_SHARING_MODE_DMA
};

struct v4l2_iovec {
    void  *iov_base;
    size_t iov_len;
};

struct v4l2_selected_format {
    uint32_t pixelformat;
    uint32_t width;
    uint32_t height;
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

static int v4l2_create_memory_fd(size_t size);
static int v4l2_create_dmabuf_fd(int memfd, size_t size);
static int v4l2_dma_alloc(size_t size, void **addr);

static uint32_t v4l2_query_capabilities(int fd, uint32_t flags);
static void v4l2_query_controls(int fd);
static int v4l2_query_mmap_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type);
static int v4l2_query_userptr_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type);
static int v4l2_query_dma_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type);
static int v4l2_query_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type, enum v4l2_memory memory);
static int v4l2_queue_buffer(int fd, int index, enum v4l2_buf_type buf_type, enum v4l2_memory memory, int verbosity);
static int v4l2_queue_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type, enum v4l2_memory memory);
static int v4l2_capture_frame(int fd, struct v4l2_iovec *iov, size_t iovcnt, enum v4l2_buf_type buf_type, enum v4l2_memory memory);
static void v4l2_store_frame(uint32_t fourcc, const struct v4l2_iovec *iov, size_t iovcnt, int counter);
static int v4l2_video_capture(int fd, int number_of_frames, enum v4l2_buf_type buf_type, enum v4l2_memory memory);

/*===========================================================================*\
 * local object definitions
\*===========================================================================*/
static const char* output_directory;
static struct v4l2_selected_format selected_format;
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
    enum v4l2_memory memory = V4L2_MEMORY_MMAP;
    bool use_compressed_formats = false;
    enum v4l2_buf_type buf_type;
    struct v4l2_format format;

    static struct option long_options[] = {
        {"number-of-frames",       required_argument, 0, 'n'},
        {"number-of-buffers",      required_argument, 0, 'b'},
        {"memory",                 required_argument, 0, 'm'},
        {"use-compressed-formats", no_argument,       0, 'c'},
        {"output-directory",       required_argument, 0, 'o'},
        {0, 0, 0, 0}
    };

    for (;;) {
        int c = getopt_long(argc, argv, "n:b:m:co:", long_options, 0);
        if (-1 == c)
            break;

        switch (c) {
            case 'n':
                number_of_frames = atoi(optarg);
                break;

            case 'b':
                number_of_buffers = atoi(optarg);
                break;

            case 'm':
                if (strcmp(optarg, "mmap") == 0) {
                    memory = V4L2_MEMORY_MMAP;
                } else
                if (strcmp(optarg, "userptr") == 0) {
                    memory = V4L2_MEMORY_USERPTR;
                } else
                if (strcmp(optarg, "dmabuf") == 0) {
                    memory = V4L2_MEMORY_DMABUF;
                } else {
                    /* use default value */
                    memory = V4L2_MEMORY_MMAP;
                }
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
    if (!(capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) ||
        !(capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s doesn't support video capture or streaming\n", filename);
        exit(EXIT_FAILURE);
    }

    /*
     * If driver supports multiplanar format (V4L2_CAP_VIDEO_CAPTURE_MPLANE),
     * then prefer this one instead of single planar one (V4L2_CAP_VIDEO_CAPTURE)
     */
    buf_type =
        capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE ?
        V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    v4l2_query_controls(fd);

    if (selected_format.pixelformat == 0) {
        fprintf(stderr, "No frame format is selected for capturing\n");
        exit(EXIT_FAILURE);
    }

    memset(&format, 0, sizeof(format));
    format.type = buf_type;
    if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
        format.fmt.pix.pixelformat = selected_format.pixelformat;
        format.fmt.pix.width = selected_format.width;
        format.fmt.pix.height = selected_format.height;
    } else {
        format.fmt.pix_mp.pixelformat = selected_format.pixelformat;
        format.fmt.pix_mp.width = selected_format.width;
        format.fmt.pix_mp.height = selected_format.height;
    }

    if (-1 == ioctl(fd, VIDIOC_TRY_FMT, &format)) {
        fprintf(stderr, "VIDIOC_TRY_FMT failed: %s\n", strerror(errno));
    }

    fprintf(stdout, "Using following format:\n");
    v4l2_print_format(&format);

    if (-1 == ioctl(fd, VIDIOC_S_FMT, &format)) {
        fprintf(stderr, "VIDIOC_S_FMT failed: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    number_of_buffers = v4l2_query_buffers(fd, number_of_buffers, buf_type, memory);
    if (number_of_buffers < 0) {
        fprintf(stderr, "v4l2_query_buffers() failed\n");
        exit(EXIT_FAILURE);
    }

    if (v4l2_queue_buffers(fd, number_of_buffers, buf_type, memory)) {
        fprintf(stderr, "v4l2_queue_buffers() failed\n");
        exit(EXIT_FAILURE);
    }

    if (v4l2_video_capture(fd, number_of_frames, buf_type, memory)) {
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
    fprintf(stdout, "usage: %s [-n <frames>] [-b <buffers>] [-m <memory>] [-c] [-o <output-directory>] <filename>\n", progname);
    fprintf(stdout, " options:\n");
    fprintf(stdout, "  -n <frames>  --number-of-frames=<frames>   : number of frames to be captured (default: 1)\n");
    fprintf(stdout, "  -b <buffers> --number-of-buffers=<buffers> : number of buffers to be allocated for capturing (default: 1)\n");
    fprintf(stdout, "  -m <memory>  --memory=<memory>             : memory allocation type {mmap, userptr, dmabuf} (default: mmap)\n");
    fprintf(stdout, "  -c --use-compressed-formats                : if set, capturing will search for compressed formats\n");
    fprintf(stdout, "  -o <dir> --output-directory=<dir>          : if set, specifies directory for captured frames\n");
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

    if (buf_type >= ARRAY_SIZE(buf_types))
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

    if (memory >= ARRAY_SIZE(memories))
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

    if (type >= ARRAY_SIZE(types))
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

    if (type >= ARRAY_SIZE(types))
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
                "\tplane[%d]\n"
                "\t\tbytesused   : %u\n"
                "\t\tlength      : %u\n",
                i,
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
                    "\t\tuserptr     : 0x%lx\n",
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
                "\tuserptr     : 0x%lx\n",
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

static int v4l2_create_memory_fd(size_t size)
{
    int retval = -1;

    do {
        int memfd;
        int status;

        /*
         * Set the close-on-exec (FD_CLOEXEC) flag on the new file descriptor.
         * Allow sealing operations on this file. See the discussion
         * of the F_ADD_SEALS and F_GET_SEALS operations in fcntl(2).
         * The initial set of seals is empty.
         * If this flag is not set, the initial set of seals will be
         * F_SEAL_SEAL, meaning that no other seals can be set on the file.
         */
        memfd = memfd_create(MEMFD_FILE_NAME, MFD_CLOEXEC | MFD_ALLOW_SEALING);
        if (memfd == -1) {
            fprintf(stderr, "memfd_create(%s) failed: %s\n", MEMFD_FILE_NAME, strerror(errno));
            break;
        }

        status = ftruncate(memfd, size);
        if (status == -1) {
            fprintf(stderr, "ftruncate(%zu) failed: %s\n", size, strerror(errno));
            break;
        }

        /* udmabuf_create requires that file descriptors be sealed with F_SEAL_SHRINK */
        status = fcntl(memfd, F_ADD_SEALS, F_SEAL_SHRINK);
        if (status == -1) {
            fprintf(stderr, "fcntl(fd, F_ADD_SEALS, F_SEAL_SHRINK) failed: %s\n", strerror(errno));
            break;
        }

        retval = memfd;
    } while (0);

    return retval;
}

static int v4l2_create_dmabuf_fd(int memfd, size_t size)
{
    int retval = -1;

    do {
        int fd;
        int dmabuffd;
        int status;
        struct udmabuf_create udmabuf_create;

        if (memfd == -1)
            break;

        fd = open(UDMABUF_DEVICE_NAME, O_RDWR);
        if (fd == -1) {
            fprintf(stderr, "cannot open '%s': %s\n", UDMABUF_DEVICE_NAME, strerror(errno));
            break;
        }

        memset(&udmabuf_create, 0, sizeof(udmabuf_create));
        udmabuf_create.memfd = memfd;
        udmabuf_create.flags = UDMABUF_FLAGS_CLOEXEC;
        udmabuf_create.offset = 0;
        udmabuf_create.size = size;

        dmabuffd = ioctl(fd, UDMABUF_CREATE, &udmabuf_create);
        if (dmabuffd == -1) {
            fprintf(stderr, "ioctl(UDMABUF_CREATE) failed: %s\n", strerror(errno));
        }

        close(fd);

        retval = dmabuffd;
    } while (0);

    return retval;
}

static int v4l2_dma_alloc(size_t size, void **addr)
{
    int memfd;
    int dmabuffd;
    void *p;
    long pagesize = sysconf(_SC_PAGESIZE);

    if (pagesize <= 0 || !IS_POWER_OF_TWO(pagesize))
        pagesize = 0x1000; // set default value to 4KiB

    size = ALIGN(size, pagesize);

    memfd = v4l2_create_memory_fd(size);

    p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, 0);
    if (p == MAP_FAILED) {
        fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
        close(memfd);
        return -1;
    }

    if (addr)
        *addr = p;

    dmabuffd = v4l2_create_dmabuf_fd(memfd, size);

    /* memfd can be closed here.
    Only when all references to the memfd are dropped,
    it will be automatically released. */
    close(memfd);

    return dmabuffd;
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

        /*
         * If driver supports multiplanar format (V4L2_CAP_VIDEO_CAPTURE_MPLANE),
         * then prefer this one instead of single planar one (V4L2_CAP_VIDEO_CAPTURE)
         */
        enum v4l2_buf_type buf_type =
            capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE ?
            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.index = 0;
        fmtdesc.type = buf_type;
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

                if (selected_format.pixelformat == 0) {
                    if (flags == fmtdesc.flags) {
                        selected_format.pixelformat = frmsizeenum.pixel_format;

                        if (V4L2_FRMSIZE_TYPE_DISCRETE == frmsizeenum.type) {
                            selected_format.width = frmsizeenum.discrete.width;
                            selected_format.height = frmsizeenum.discrete.height;
                        }
                        else
                        if (V4L2_FRMSIZE_TYPE_STEPWISE == frmsizeenum.type) {
                            selected_format.width = frmsizeenum.stepwise.min_width;
                            selected_format.height = frmsizeenum.stepwise.min_height;
                        }
                        else {
                            selected_format.width = 0;
                            selected_format.height = 0;
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

static int v4l2_query_mmap_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type)
{
    int i;

    for (i = 0; i < number_of_buffers; ++i) {
        struct v4l2_buffer_descriptor* bd = buffer_descriptors + i;
        struct v4l2_buffer buffer;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        void* addr;

        memset(&buffer, 0, sizeof(buffer));
        buffer.index = i;
        buffer.type = buf_type;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            memset(&planes, 0, sizeof(planes));
            buffer.length = ARRAY_SIZE(planes);
            buffer.m.planes = planes;
        }

        if(-1 == ioctl(fd, VIDIOC_QUERYBUF, &buffer)) {
            fprintf(stderr, "VIDIOC_QUERYBUF[%d] failed: %s\n", i, strerror(errno));
            break;
        }

        fprintf(stdout, "VIDIOC_QUERYBUF[%u]:\n", i);
        v4l2_print_buffer(&buffer);

        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            unsigned plane;

            bd->index = i;
            bd->nplanes = buffer.length;

            for (plane = 0; plane < buffer.length; ++plane) {
                if (buffer.m.planes[plane].length > 0) {
                    addr = mmap(NULL, buffer.m.planes[plane].length,
                        PROT_READ | PROT_WRITE, MAP_SHARED,
                        fd, buffer.m.planes[plane].m.mem_offset);
                    if (MAP_FAILED == addr) {
                        fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
                        break;
                    }
                }

                bd->planes[plane].addr = addr;
                bd->planes[plane].size = buffer.m.planes[plane].length;
                bd->planes[plane].fd = -1;
            }
            if (plane < buffer.length)
                break;
        } else {
            addr = mmap(NULL, buffer.length,
                PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
            if (MAP_FAILED == addr) {
                fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
                break;
            }

            bd->index = i;
            bd->nplanes = 1;
            bd->planes[0].addr = addr;
            bd->planes[0].size = buffer.length;
            bd->planes[0].fd = -1;
        }
    }

    return i == number_of_buffers ? 0 /*success*/ : -1 /*failture*/;
}

static int v4l2_query_userptr_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type)
{
    int i;
    struct v4l2_format format;

    memset(&format, 0, sizeof(format));
    format.type = buf_type;
    if (-1 == ioctl(fd, VIDIOC_G_FMT, &format)) {
        fprintf(stderr, "VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }

    if (format.type != buf_type) {
        fprintf(stderr, "Incompatible buffer types detected\n");
        return -1;
    }

    for (i = 0; i < number_of_buffers; ++i) {
        struct v4l2_buffer_descriptor* bd = buffer_descriptors + i;
        size_t size;
        void* addr;

        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            unsigned plane;

            bd->index = i;
            bd->nplanes = format.fmt.pix_mp.num_planes;

            for (plane = 0; plane < bd->nplanes; ++plane) {
                if (format.fmt.pix_mp.plane_fmt[plane].sizeimage)
                    size = format.fmt.pix_mp.plane_fmt[plane].sizeimage;
                else
                if (format.fmt.pix_mp.plane_fmt[plane].bytesperline)
                    size = format.fmt.pix_mp.plane_fmt[plane].bytesperline * format.fmt.pix_mp.height;
                else
                    break;

                addr = malloc(size);
                if (addr == NULL)
                    break;

                bd->planes[plane].addr = addr;
                bd->planes[plane].size = size;
                bd->planes[plane].fd = -1;
            }
            if (plane < bd->nplanes)
                break;
        } else {
            if (format.fmt.pix.sizeimage)
                size = format.fmt.pix.sizeimage;
            else
            if (format.fmt.pix.bytesperline)
                size = format.fmt.pix.bytesperline * format.fmt.pix.height;
            else
                break;

            addr = malloc(size);
            if (addr == NULL)
                break;

            bd->index = i;
            bd->nplanes = 1;
            bd->planes[0].addr = addr;
            bd->planes[0].size = size;
            bd->planes[0].fd = -1;
        }
    }

    return i == number_of_buffers ? 0 /*success*/ : -1 /*failture*/;
}

static int v4l2_query_dma_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type)
{
    int i;
    struct v4l2_format format;

    memset(&format, 0, sizeof(format));
    format.type = buf_type;
    if (-1 == ioctl(fd, VIDIOC_G_FMT, &format)) {
        fprintf(stderr, "VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }

    if (format.type != buf_type) {
        fprintf(stderr, "Incompatible buffer types detected\n");
        return -1;
    }

    for (i = 0; i < number_of_buffers; ++i) {
        struct v4l2_buffer_descriptor* bd = buffer_descriptors + i;
        size_t size;
        void* addr;
        int dmabuffd;

        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            unsigned plane;

            bd->index = i;
            bd->nplanes = format.fmt.pix_mp.num_planes;

            for (plane = 0; plane < bd->nplanes; ++plane) {
                if (format.fmt.pix_mp.plane_fmt[plane].sizeimage)
                    size = format.fmt.pix_mp.plane_fmt[plane].sizeimage;
                else
                if (format.fmt.pix_mp.plane_fmt[plane].bytesperline)
                    size = format.fmt.pix_mp.plane_fmt[plane].bytesperline * format.fmt.pix_mp.height;
                else
                    break;

                dmabuffd = v4l2_dma_alloc(size, &addr);

                bd->planes[plane].addr = addr;
                bd->planes[plane].size = size;
                bd->planes[plane].fd = dmabuffd;
            }
            if (plane < bd->nplanes)
                break;
        } else {
            if (format.fmt.pix.sizeimage)
                size = format.fmt.pix.sizeimage;
            else
            if (format.fmt.pix.bytesperline)
                size = format.fmt.pix.bytesperline * format.fmt.pix.height;
            else
                break;

            dmabuffd = v4l2_dma_alloc(size, &addr);

            bd->index = i;
            bd->nplanes = 1;
            bd->planes[0].addr = addr;
            bd->planes[0].size = size;
            bd->planes[0].fd = dmabuffd;
        }
    }

    return i == number_of_buffers ? 0 /*success*/ : -1 /*failture*/;
};

static int v4l2_query_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type, enum v4l2_memory memory)
{
    int retval = -1;

    do {
        int status;
        struct v4l2_requestbuffers requestbuffers;

        memset(&requestbuffers, 0, sizeof(requestbuffers));
        requestbuffers.count = number_of_buffers;
        requestbuffers.type = buf_type;
        requestbuffers.memory = memory;

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

        switch (memory) {
            case V4L2_MEMORY_MMAP:
                status = v4l2_query_mmap_buffers(fd, requestbuffers.count, buf_type);
                break;

            case V4L2_MEMORY_USERPTR:
                status = v4l2_query_userptr_buffers(fd, requestbuffers.count, buf_type);
                break;

            case V4L2_MEMORY_DMABUF:
                status = v4l2_query_dma_buffers(fd, requestbuffers.count, buf_type);
                break;

            default:
                status = -1;
                break;
        };

        if (status)
            break;

        retval = requestbuffers.count;
    } while (0);

    return retval;
}

static int v4l2_queue_buffer(int fd, int index, enum v4l2_buf_type buf_type, enum v4l2_memory memory, int verbosity)
{
    int retval = -1;

    do {
        struct v4l2_buffer buffer;
        struct v4l2_buffer_descriptor* bd = buffer_descriptors + index;
        struct v4l2_plane planes[bd->nplanes];

        memset(&buffer, 0, sizeof(buffer));
        buffer.index = index;
        buffer.type = buf_type;
        buffer.memory = memory;
        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            memset(&planes, 0, sizeof(planes));
            if (memory == V4L2_MEMORY_USERPTR) {
                unsigned plane;
                for (plane = 0; plane < bd->nplanes; ++plane) {
                    planes[plane].m.userptr = (unsigned long)bd->planes[plane].addr;
                    planes[plane].length = bd->planes[plane].size;
                }
            }
            else
            if (memory == V4L2_MEMORY_DMABUF) {
                unsigned plane;
                for (plane = 0; plane < bd->nplanes; ++plane)
                    planes[plane].m.fd = bd->planes[plane].fd;
              }
            else {
                /* do nothing */
            }

            buffer.length = bd->nplanes;
            buffer.m.planes = planes;
        } else {
            if (memory == V4L2_MEMORY_USERPTR) {
                buffer.m.userptr = (unsigned long)bd->planes[0].addr;
                buffer.length = bd->planes[0].size;
            }
            else
            if (memory == V4L2_MEMORY_DMABUF) {
                buffer.m.fd = bd->planes[0].fd;
            }
            else {
                /* do nothing */
            }
        }

        if (-1 == ioctl(fd, VIDIOC_QBUF, &buffer)) {
            fprintf(stderr, "VIDIOC_QBUF[%d] failed: %s\n", index, strerror(errno));
            break;
        }

        if (verbosity > 0) {
            fprintf(stdout, "VIDIOC_QBUF[%d]:\n", index);
            v4l2_print_buffer(&buffer);
        }

        retval = 0;
    } while (0);

    return retval;
}

static int v4l2_queue_buffers(int fd, int number_of_buffers, enum v4l2_buf_type buf_type, enum v4l2_memory memory)
{
    int retval = -1;

    do {
        int i;
        int status;

        for (i = 0; i < number_of_buffers; ++i) {
            status = v4l2_queue_buffer(fd, i, buf_type, memory, 1);
            if (status)
                break;
        }

        if (i < number_of_buffers)
            break;

        retval = 0;
    } while (0);

    return retval;
}

static int v4l2_capture_frame(int fd, struct v4l2_iovec *iov, size_t iovcnt, enum v4l2_buf_type buf_type, enum v4l2_memory memory)
{
    int retval = -1;

    do {
        int status;
        struct v4l2_buffer buffer;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        fd_set fds;
        struct timespec ts;
        uint32_t index;
        uint32_t flags;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        ts.tv_sec = SELECT_TIMEOUT_SEC;
        ts.tv_nsec = 0;
        status = pselect(fd+1, &fds, NULL, NULL, &ts, NULL);
        if (-1 == status) {
            fprintf(stderr, "pselect() failed: %s\n", strerror(errno));
            break;
        } else
        if (0 == status) {
            fprintf(stderr, "no data within %d seconds, timeout expired\n", SELECT_TIMEOUT_SEC);
            break;
        }

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = buf_type;
        buffer.memory = memory;
        if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
            memset(&planes, 0, sizeof(planes));
            buffer.length = ARRAY_SIZE(planes);
            buffer.m.planes = planes;
        }

        if (-1 == ioctl(fd, VIDIOC_DQBUF, &buffer)) {
            fprintf(stderr, "VIDIOC_DQBUF failed: %s\n", strerror(errno));
            break;
        }

        fprintf(stdout, "VIDIOC_DQBUF:\n");
        v4l2_print_buffer(&buffer);

        index = buffer.index;
        flags = buffer.flags;

        memset(iov, 0, sizeof(*iov) * iovcnt);

        if (0 == (flags & V4L2_BUF_FLAG_ERROR)) {
            if (V4L2_TYPE_IS_MULTIPLANAR(buf_type)) {
                unsigned plane;
                for (plane = 0; plane < buffer.length && plane < iovcnt; ++plane) {
                    iov[plane].iov_base = buffer_descriptors[index].planes[plane].addr;
                    iov[plane].iov_len = buffer.m.planes[plane].bytesused;
                }
            }
            else {
                if (iovcnt > 0) {
                    iov[0].iov_base = buffer_descriptors[index].planes[0].addr;
                    iov[0].iov_len = buffer.bytesused;
                }
            }
        }

        status = v4l2_queue_buffer(fd, index, buf_type, memory, 0);
        if (status) {
            fprintf(stderr, "v4l2_queue_buffer() failed\n");
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

static void v4l2_store_frame(uint32_t fourcc, const struct v4l2_iovec *iov, size_t iovcnt, int counter)
{
    char image_filename[256];
    int fd = -1;

    do {
        int n;
        size_t i;

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
        if (-1 == fd) {
            fprintf(stderr, "cannot open '%s': %s\n", image_filename, strerror(errno));
            break;
        }

        for (i = 0; i < iovcnt && iov[i].iov_base; ++i) {
            if (-1 == write(fd, iov[i].iov_base, iov[i].iov_len)) {
                fprintf(stderr, "write() failed: %s\n", strerror(errno));
                break;
            }
        }
    } while (0);

    if (fd != -1)
        close(fd);
}

static int v4l2_video_capture(int fd, int number_of_frames, enum v4l2_buf_type buf_type, enum v4l2_memory memory)
{
    struct v4l2_iovec iov[VIDEO_MAX_PLANES];
    int i;

    if (-1 == ioctl(fd, VIDIOC_STREAMON, &buf_type)) {
        fprintf(stderr, "VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return -1;
    }

    i = 0;
    while (i < number_of_frames) {
        if (0 == v4l2_capture_frame(fd, iov, ARRAY_SIZE(iov), buf_type, memory)) {
            v4l2_store_frame(selected_format.pixelformat, iov, ARRAY_SIZE(iov), i + 1);
            i++;
        }
    }

    if (-1 == ioctl(fd, VIDIOC_STREAMOFF, &buf_type)) {
        fprintf(stderr, "VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}
