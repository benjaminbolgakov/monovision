import numpy as np
#import OpenGL.GL as gl
#import pangolin

from multiprocessing import Process, Queue

class Point(object):
    """
    Represents a 3D point in the world
    """
    def __init__(self, pmap, loc, color, tid=None):
        self.pt = np.array(loc)
        self.frames = []
        self.idxs = []
        self.id = tid if tid is not None else pmap.add_point(self)
        #self.id = len(pmap.points)
        #pmap.points.append(self)

    def homogenize(self):
        if len(self.pt.shape) == 1:
            return np.concatenate([self.pt, np.array([1.0])], axis=0)
        else:
            return np.concatenate([self.pt, np.ones((self.pt.shape[0], 1))], axis=1)

    def orb(self):
        return [f.des[idx] for f,idx in zip(self.frames, self.idxs)]

    def hamming_distance(self, a, b):
        r = (1 << np.arange(8))[:, None]
        return np.count_nonzero((np.bitwise_xor(a,b) & r) != 0)

    def orb_distance(self, des):
        return min([self.hamming_distance(o, des) for o in self.orb()])

    def delete(self):
        for f,idx in zip(self.frames, self.idxs):
            f.pts[idx] = None
        del self

    def add_observation(self, frame, idx):
        assert frame.pts[idx] is None
        assert  frame not in self.frames
        frame.pts[idx] = self
        self.frames.append(frame)
        self.idxs.append(idx)


class PointMap(object):
    def __init__(self):
        self.frames = []
        self.points = []
        self.state = None
        self.q = Queue()
        self.max_frame = 0
        self.max_point = 0

    def display(self):
        poses, pts = [], []
        for f in self.frames:
            poses.append(f.pose)
        for p in self.points:
            pts.append(p.pt)
        self.q.put((np.array(poses), np.array(pts)))

    def serialize(self):
        ret = {}
        ret['points'] = [{'id': p.id, 'pt': p.pt.tolist()} for p in self.points]
        ret['frames'] = []
        for f in self.frames:
            ret['frames'].append({
                'id': f.id, 'K': f.K.tolist(), 'pose': f.pose.tolist(), 'h': f.h, 'w': f.w,
                'kpus': f.kpus.tolist(), 'des': f.des.tolist(),
                'pts': [p.id if p is not None else -1 for p in f.pts]})
        ret['max_frame'] = self.max_frame
        ret['max_point'] = self.max_point
        return json.dumps(ret)

    def deserialize(self, s):
        ret = json.loads(s)
        self.max_frame = ret['max_frame']
        self.max_point = ret['max_point']
        self.points = []
        self.frames = []

        pids = {}
        for p in ret['points']:
            pp = Point(self, p['pt'], p['color'], p['id'])
            self.points.append(pp)
            pids[p['id']] = pp

        for f in ret['frames']:
            ff = Frame(self, None, f['K'], f['pose'], f['id'])
            ff.w, ff.h = f['w'], f['h']
            ff.kpus = np.array(f['kpus'])
            ff.des = np.array(f['des'])
            ff.pts = [None] * len(ff.kpus)
            for i,p in enumerate(f['pts']):
                if p != -1:
                    ff.pts[i] = pids[p]
            self.frames.append(ff)

    def add_point(self, point):
        ret = self.max_point
        self.max_point += 1
        self.points.append(point)
        return ret

    def add_frame(self, frame):
        ret = self.max_frame
        self.max_frame += 1
        self.frames.append(frame)
        return ret

    # def optimize(self, local_window=LOCAL_WINDOW, fix_points=False, verbose=False):
    #     err =
