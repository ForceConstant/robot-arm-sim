"""One physical follower serving either piece colour in the interactive lab."""
from .env import ChessSimEnv
from .dual import DualArmEnv
from .config import EnvConfig


class SingleFollowerEnv(ChessSimEnv):
    # These helpers do not require a second model; use the same physical
    # retreat/candidate rules as the retained dual-arm debug fixture.
    park=DualArmEnv.park
    candidate_moves=DualArmEnv.candidate_moves

    def __init__(self,config: EnvConfig|None=None):
        super().__init__((config or EnvConfig()).model_copy(update={'dual_arms':False}))
        self.active_side='white'
        self._capture_count={'white':0}
        self._contexts={'white':dict(board=self.board,ik=self.ik,qpos=self._joint_qpos,
            dof=self._joint_dof,actuators=self._actuators,reach=self.reach)}

    def select_arm(self,side):
        if side not in ('white','black'):
            raise ValueError('Unknown piece side')
        # Side-to-move is not a second physical robot. Both colours use the
        # same calibrated model, joints, camera and fixed white-side mount.
        self.active_side='white'

    def reset(self,*args,**kwargs):
        out=super().reset(*args,**kwargs)
        self._capture_count={'white':0}
        return out

    def _commit(self,task,result):
        super()._commit(task,result)
        self._capture_count={'white':len(self._captured)}
