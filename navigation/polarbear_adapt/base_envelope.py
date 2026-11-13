"""Competition-only unpadded chassis mask, shared by both lidar branches."""
import json
from pathlib import Path
import numpy as np
import shapely
from chassis_projection import RigidProjection

BASE_ENVELOPE_CONFIG=Path(__file__).resolve().parents[1]/'config/base_envelope.json'

class BaseEnvelope:
    def __init__(self,shape):
        self.projection=RigidProjection(shape)
        self.shape=shape

    @classmethod
    def load(cls,config=BASE_ENVELOPE_CONFIG):
        config=Path(config)
        if not config.exists():return None
        settings=json.loads(config.read_text())
        if not settings.get('enabled',False):return None
        path=Path(settings['wkt_file'])
        if not path.is_absolute():path=config.parent/path
        return cls(shapely.from_wkt(path.read_text()))

    def mask(self,body_points):
        points=np.asarray(body_points)
        inside=np.zeros(len(points),bool)
        valid=np.isfinite(points).all(axis=1)
        inside[valid]=shapely.covers(self.shape,shapely.points(points[valid,:2]))
        return inside

    def planning_copy(self,grid,origin,resolution,pose):
        data=np.asarray(grid).copy()
        if pose is not None:
            data.flat[self.projection.covered_cells(data,origin,resolution,pose)]=0
        return data
