"""Bounded global display coverage. No LRU, time expiry, registration or map edits."""
import math
import numpy as np


class GlobalVoxelPreview:
    def __init__(self, voxel, limit):
        if not math.isfinite(voxel) or voxel <= 0 or not isinstance(limit,int) or limit < 1:
            raise ValueError('Invalid global preview resolution/capacity')
        self.initial_voxel = self.voxel = voxel
        self.limit = limit
        self.cells = {}
        self.coarsenings = 0
        self.merged = 0

    def _voxelize(self, xyz):
        keys = np.floor(xyz/self.voxel).astype(np.int64)
        unique, indices = np.unique(keys,axis=0,return_index=True)
        # Keep actual measured coordinates, not synthetic voxel centres.
        return {tuple(map(int,k)):tuple(map(float,p)) for k,p in zip(unique,xyz[indices])}

    def add(self, xyz):
        xyz = np.asarray(xyz,dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
            raise ValueError('Expected finite nonempty Nx3 cloud')
        if np.abs(xyz).max() > 10000:
            raise ValueError('Preview coordinates exceed accepted odom bounds')
        self.cells.update(self._voxelize(xyz))
        while len(self.cells) > self.limit:
            old_count = len(self.cells)
            points = self.points()
            self.voxel *= 2
            self.cells = self._voxelize(points)
            self.coarsenings += 1
            self.merged += old_count-len(self.cells)

    def points(self):
        return np.asarray(list(self.cells.values()),dtype=np.float32).reshape(-1,3)

    def __len__(self):
        return len(self.cells)
