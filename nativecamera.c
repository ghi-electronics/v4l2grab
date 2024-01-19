/***************************************************************************
 *   v4l2grab Version 0.3                                                  *
 *   Copyright (C) 2012 by Tobias Müller                                   *
 *   Tobias_Mueller@twam.info                                              *
 *                                                                         *
 *   based on V4L2 Specification, Appendix B: Video Capture Example        *
 *   (http://v4l2spec.bytesex.org/spec/capture-example.html)               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
 
 /**************************************************************************
 *   GHI Electronics
 *   Modified to return Jpeg array
 ***************************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include "jpeglib.h"

#define ERROR_OPEN 1
#define ERROR_IDENTIFY 2
#define ERROR_NODEV 3

#define ERROR_NOTSUPPORT 4
#define ERROR_NOTSUPPORT2 5
#define ERROR_NOTSUPPORT_CAPTURE 6
#define ERROR_NOTSUPPORT_STREAM 7
#define ERROR_VIDIOC_S_FMT 8

#define ERROR_VIDIOC_STREAMOFF 9
#define ERROR_VIDIOC_QBUF 10
#define ERROR_VIDIOC_STREAMON 11

#define ERROR_SELECT_TIMEOUT 12

#define IO_READ
#define IO_MMAP
//#define IO_USERPTR


#define CLEAR(x) memset (&(x), 0, sizeof (x))

typedef enum {
#ifdef IO_READ
        IO_METHOD_READ,
#endif
#ifdef IO_MMAP
        IO_METHOD_MMAP,
#endif
#ifdef IO_USERPTR
        IO_METHOD_USERPTR,
#endif
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

static io_method        io              = 0;//IO_METHOD_READ;
static int              fd              = -1;
struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;

// global settings
static unsigned int width = 640;
static unsigned int height = 480;
static unsigned int bufferSize = 0;
static unsigned int framerate = 15;
static unsigned char jpegQuality = 70;
static char* jpegFilename = NULL;
static char* deviceName = "/dev/video0";
static char* RgbData = NULL;

/**
  Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV

  \param width width of image
  \param height height of image
  \param src source
  \param dst destination
*/

/**
  Print error message and terminate programm with EXIT_FAILURE return code.
  \param s error message to print
*/
static void errno_exit(const char* s)
{
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror (errno));
  exit(EXIT_FAILURE);
}

/**
  Do ioctl and retry if error was EINTR ("A signal was caught during the ioctl() operation."). Parameters are the same as on ioctl.

  \param fd file descriptor
  \param request request
  \param argp argument
  \returns result from ioctl
*/
static int xioctl(int fd, int request, void* argp)
{
  int r;

  do r = ioctl(fd, request, argp);
  while (-1 == r && EINTR == errno);

  return r;
}

static void JpegTo888(unsigned char* inBuf, int img_size, unsigned char* outBuf) {
  unsigned long x, y;

  unsigned long data_size;     // length of the file
  int channels;               //  3 =>RGB   4 =>RGBA 
  unsigned int type;  
  unsigned char * rowptr[1];    // pointer to an array
  unsigned char * jdata;        // data for the image

  struct jpeg_decompress_struct info; //for our jpeg info
  struct jpeg_error_mgr err;   

  info.err = jpeg_std_error(&err);     
  jpeg_create_decompress(& info);   //fills info structure

  //jpeg_stdio_src(&info, file);    

  
  jpeg_mem_src(& info, inBuf, img_size);
  jpeg_read_header(&info, TRUE);   // read jpeg file header

  jpeg_start_decompress(&info);    // decompress the file

  x = info.output_width;
  y = info.output_height;
  channels = info.num_components;
  

  data_size = x * y * 3;

  jdata = outBuf;
  while (info.output_scanline < info.output_height) // loop
  {
    // Enable jpeg_read_scanlines() to fill our jdata array
    rowptr[0] = (unsigned char *)jdata +  // secret to method
            3* info.output_width * info.output_scanline; 

    jpeg_read_scanlines(&info, rowptr, 1);
  }

  jpeg_finish_decompress(&info);   //finish decompressing

  jpeg_destroy_decompress(&info);   
}

static void Rgb888To565(unsigned char* src, unsigned char* dst) {

  unsigned char* tmp = dst;

  for (int i = 0; i < width * height *3; i+=3) {

    unsigned int color = src[i+2] | (src[i+1] << 8) | (src[i+0] << 16);
    *tmp++ = (unsigned char)(((color & 0x00001c00) >> 5) | ((color & 0x000000f8) >> 3));
    *tmp++ = (unsigned char)(((color & 0x00f80000) >> 16) | ((color & 0x0000e000) >> 13));

  }

}
/**
  process image read
*/
void imageProcess(const void* p, unsigned char* dst)
{
  unsigned char* src = (unsigned char*)p;

  memset(dst,0, bufferSize);

  memcpy(dst, src, bufferSize);
    
}

/**
  read single frame
*/

static int frameRead(unsigned char* dst)
{
  struct v4l2_buffer buf;
#ifdef IO_USERPTR
  unsigned int i;
#endif

  switch (io) {
#ifdef IO_READ
    case IO_METHOD_READ:
      if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            // Could ignore EIO, see spec.

            // fall through
          default:
            errno_exit("read");
        }
      }

      imageProcess(buffers[0].start,dst);
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
             // Could ignore EIO, see spec

             // fall through
          default:
            errno_exit("VIDIOC_DQBUF");
        }
      }

      assert (buf.index < n_buffers);

      imageProcess(buffers[buf.index].start, dst);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      break;
#endif

#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            // Could ignore EIO, see spec.

            // fall through
          default:
            errno_exit("VIDIOC_DQBUF");
                        
        }
      }

      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long) buffers[i].start && buf.length == buffers[i].length)
          break;

      assert (i < n_buffers);

      imageProcess((void *) buf.m.userptr, dst);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
      break;
#endif

    }

  return 1;
}

/** 
  mainloop: read frames and process them
*/
static int mainLoop(unsigned char* dst)
{
  unsigned int count;

  count = 1;

  while (count-- > 0) {
    for (;;) {
      fd_set fds;
      struct timeval tv;
      int r;

      FD_ZERO(&fds);
      FD_SET(fd, &fds);

      /* Timeout. */
      tv.tv_sec = 2;
      tv.tv_usec = 0;

      r = select(fd + 1, &fds, NULL, NULL, &tv);

      if (-1 == r) {
        if (EINTR == errno)
          continue;

        errno_exit("select");
      }

      if (0 == r) {
        //fprintf (stderr, "select timeout\n");
        //exit(EXIT_FAILURE);

        return ERROR_SELECT_TIMEOUT;
      }

      if (frameRead(dst))
        break;
        
      /* EAGAIN - continue select loop. */
    }
  }

  return 0;
}

int NativeMainLoop(unsigned char* dst) {
  return mainLoop(dst);
}

/**
  stop capturing
*/
static int captureStop(void)
{
  enum v4l2_buf_type type;

  switch (io) {
#ifdef IO_READ
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
#endif
#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
#endif
#if defined(IO_MMAP) || defined(IO_USERPTR)
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        //errno_exit("VIDIOC_STREAMOFF");
        return ERROR_VIDIOC_STREAMOFF;

      break;
#endif 
   }

   return 0;
}

int NativeCaptureStop(void) {
  return captureStop();
}

/**
  start capturing
*/
static int captureStart(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io) {
#ifdef IO_READ    
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }
                
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;
#endif

#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_USERPTR;
        buf.index       = i;
        buf.m.userptr   = (unsigned long) buffers[i].start;
        buf.length      = buffers[i].length;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
          //errno_exit("VIDIOC_QBUF");
          return ERROR_VIDIOC_QBUF;
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
         //errno_exit("VIDIOC_STREAMON");
         return ERROR_VIDIOC_STREAMON;

      break;
#endif
  }

  return 0;
}

int NativeCaptureStart(void) {
  return captureStart();
}


static int deviceUninit(void)
{
  unsigned int i;

  switch (io) {
#ifdef IO_READ
    case IO_METHOD_READ:
      free(buffers[0].start);
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
      if (-1 == munmap (buffers[i].start, buffers[i].length))
        errno_exit("munmap");
      break;
#endif

#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
        free (buffers[i].start);
      break;
#endif
  }

  free(buffers);

  return 0;
}

int NativeDeviceUninit(void) {
  return deviceUninit();
}

#ifdef IO_READ
static void readInit(unsigned int buffer_size)
{
  buffers = calloc(1, sizeof(*buffers));

  if (!buffers) {
    fprintf (stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  buffers[0].length = buffer_size;
  buffers[0].start = malloc (buffer_size);

  if (!buffers[0].start) {
    fprintf (stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  bufferSize = buffer_size;
}
#endif

#ifdef IO_MMAP
static void mmapInit(void)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count               = 4;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support memory mapping\n", deviceName);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", deviceName);
    exit(EXIT_FAILURE);
  }

  buffers = calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
    mmap (NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");

    bufferSize =  buf.length;
  }
}
#endif

#ifdef IO_USERPTR
static void userptrInit(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize ();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count               = 4;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support user pointer i/o\n", deviceName);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  buffers = calloc(4, sizeof (*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = memalign (/* boundary */ page_size, buffer_size);

    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }
  }
}
#endif

/**
  initialize device
*/
static int deviceInit(void)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  struct v4l2_streamparm frameint;
  unsigned int min;

  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      //fprintf(stderr, "%s is no V4L2 device\n",deviceName);
      //exit(EXIT_FAILURE);

      return ERROR_NOTSUPPORT;
    } else {
      //errno_exit("VIDIOC_QUERYCAP");
       return ERROR_NOTSUPPORT2;
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    //fprintf(stderr, "%s is no video capture device\n",deviceName);
    //exit(EXIT_FAILURE);
    return ERROR_NOTSUPPORT_CAPTURE;
  }

  switch (io) {
#ifdef IO_READ
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        fprintf(stderr, "%s does not support read i/o\n",deviceName);
        exit(EXIT_FAILURE);
      }
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
#endif
#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
#endif
#if defined(IO_MMAP) || defined(IO_USERPTR)
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        //fprintf(stderr, "%s does not support streaming i/o\n",deviceName);
        //exit(EXIT_FAILURE);
        return ERROR_NOTSUPPORT_STREAM;
      }
      break;
#endif
  }


  /* Select video input, video standard and tune here. */
  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;


  if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {        
    /* Errors ignored. */
  }

  CLEAR (fmt);

  // v4l2_format
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = width; 
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
  fmt.fmt.pix.field       = V4L2_FIELD_NONE;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
    //errno_exit("VIDIOC_S_FMT");
    return ERROR_VIDIOC_S_FMT;

  /* Note VIDIOC_S_FMT may change width and height. */
  if (width != fmt.fmt.pix.width) {
    width = fmt.fmt.pix.width;
    //fprintf(stderr,"Image width set to %i by device %s.\n",width,deviceName);
  }
  if (height != fmt.fmt.pix.height) {
    height = fmt.fmt.pix.height;
    //fprintf(stderr,"Image height set to %i by device %s.\n",height,deviceName);
  }

   /* If the user has set the fps to -1, don't try to set the frame interval */
  #if 1
    CLEAR(frameint);
    
    /* Attempt to set the frame interval. */
    frameint.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frameint.parm.capture.timeperframe.numerator = 1;
    frameint.parm.capture.timeperframe.denominator = framerate;
    if (-1 == xioctl(fd, VIDIOC_S_PARM, &frameint))
      fprintf(stderr,"Unable to set frame interval.\n");
  #endif

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  switch (io) {
#ifdef IO_READ
    case IO_METHOD_READ:
      readInit(fmt.fmt.pix.sizeimage);
      break;
#endif

#ifdef IO_MMAP
    case IO_METHOD_MMAP:
      mmapInit();
      break;
#endif

#ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
      userptrInit(fmt.fmt.pix.sizeimage);
      break;
#endif
  }

  return 0;
}

int NativeDeviceInit(int w, int h, int fps, int quality) {
  width = w;
  height = h;
  framerate = fps;
  jpegQuality = quality;

  return deviceInit();
}

/**
  close device
*/
static int deviceClose(void)
{
  if (-1 == close (fd))
    errno_exit("close");

  fd = -1;

  return 0;
}

int NativeDeviceClose() {
  return deviceClose();
}


/**
  open device
*/

static int deviceOpen(void)
{
  struct stat st;

  // stat file
  if (-1 == stat(deviceName, &st)) {
    //fprintf(stderr, "Cannot identify '%s': %d, %s\n", deviceName, errno, strerror (errno));
    //exit(EXIT_FAILURE);
    return ERROR_IDENTIFY;
  }

  // check if its device
  if (!S_ISCHR (st.st_mode)) {
    //fprintf(stderr, "%s is no device\n", deviceName);
    //exit(EXIT_FAILURE);
    return ERROR_NODEV;
  }

  // open device
  fd = open(deviceName, O_RDWR /* required */ | O_NONBLOCK, 0);

  // check if opening was successfull
  if (-1 == fd) {
    //fprintf(stderr, "Cannot open '%s': %d, %s\n", deviceName, errno, strerror (errno));
    //exit(EXIT_FAILURE);

    return ERROR_OPEN;
  }

  return 0;
}

int NativeDeviceOpen(char* dev, int io_mode) {
  deviceName = dev;  
  io = io_mode;

  return deviceOpen();

}

int GetBufferSize() {
  return bufferSize;
}
