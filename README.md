v4l2-video-capture - a small application making use of basic V4L2 api
==============================================================

This is small application which allows to capture N frames from any 
V4L2 compatible cameras. It was written mainy to learn V4L2 api.
So it does not serve any specific purpose other than educational.
On the other hand apart from standard MMAP and single planar buffers
it supports also:
- multi-planar buffers
- USERPTR memory model
- DMABUF memory model

# BUILD
This is cmake based application, so as with every cmake project, just run:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

This should give you a file named v4l2-video-capture.

# RUN
Several examples:

Capture 5 frames (allocating 3 V4L2_MEMORY_MMAP buffers) from /dev/video0 device
    $ v4l2-video-capture -b3 -n5 -mmmap -c /dev/video0
    
Capture 7 frames (allocating 4 V4L2_MEMORY_USERPTR buffers) from /dev/video0 device
    $ v4l2-video-capture -b4 -n7 -muserptr /dev/video0
    
Capture 9 frames (allocating 5 V4L2_MEMORY_DMABUF buffers) from /dev/video0 device
    $ v4l2-video-capture -b5 -n9 -mdmabuf /dev/video0

# NOTE
Using V4L2_MEMORY_DMABUF requires some midification of the linux kernel. 
I prepared a patch which I will try to merge to the kernel but not sure 
whether it will be accepted.


