#!/usr/bin/env python3
"""xwd -> png (TrueColor 24/32bpp only). 用法: xwd2png.py in.xwd out.png"""
import struct, sys
import numpy as np
from PIL import Image


def main(src, dst):
    d = open(src, 'rb').read()
    h = struct.unpack('>25I', d[:100])
    header_size, depth, w, hgt = h[0], h[3], h[4], h[5]
    bpp, bpl, vclass = h[11], h[12], h[13]
    rmask, gmask, bmask = h[14], h[15], h[16]
    ncolors = h[19]
    off = header_size + ncolors * 12
    buf = np.frombuffer(d[off:off + bpl * hgt], dtype=np.uint8).reshape(hgt, bpl)
    buf = buf[:, :w * (bpp // 8)]
    if bpp == 32:
        px = buf.reshape(hgt, w, 4)
        # masks 0xFF0000/0xFF00/0xFF + LSBFirst -> bytes are B,G,R,X
        rgb = px[:, :, [2, 1, 0]]
    elif bpp == 24:
        px = buf.reshape(hgt, w, 3)
        rgb = px[:, :, [2, 1, 0]]
    else:
        raise SystemExit('unsupported bpp %d (visual_class=%d)' % (bpp, vclass))
    Image.fromarray(rgb).save(dst)
    print('%s -> %s  %dx%d' % (src, dst, w, hgt))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
