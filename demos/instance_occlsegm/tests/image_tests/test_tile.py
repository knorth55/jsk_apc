import skimage.data

import numpy as np

import instance_occlsegm_lib


def test_tile():
    imgs = []
    for img in ['astronaut', 'camera', 'coffee', 'horse']:
        img = getattr(skimage.data, img)()
        imgs.append(img)

    tiled = instance_occlsegm_lib.image.tile(imgs, shape=(2, 2))
    assert tiled.dtype == np.uint8
