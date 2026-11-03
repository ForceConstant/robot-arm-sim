"""Pinned, unmodified LeRobot ACT neural graph, isolated from hardware imports.

Only AST-selected neural classes/functions are executed. Arithmetic is unchanged;
the policy registry, drivers, dataset stack and publication code are not loaded.
This is an offline qualification harness, not a deployed robot controller.
"""
import ast
import hashlib
import math
from itertools import chain
from pathlib import Path
from types import SimpleNamespace
from collections.abc import Callable

REVISION = '2774d9bddcbbda50e697e162e89e7eaada8d7105'
SOURCE_SHA256 = '8d622935319af7d03f1efd95d51e726af2921010f7e4f5ec3742f62280493111'
NAMES = {'ACT', 'ACTEncoder', 'ACTEncoderLayer', 'ACTDecoder', 'ACTDecoderLayer',
         'ACTSinusoidalPositionEmbedding2d', 'create_sinusoidal_pos_embedding', 'get_activation_fn'}


def load_act(source):
    import einops
    import numpy as np
    import torch
    import torchvision
    from torch import Tensor, nn
    from torch.nn import functional as F
    from torchvision.models._utils import IntermediateLayerGetter
    from torchvision.ops.misc import FrozenBatchNorm2d
    raw = Path(source).read_bytes()
    if hashlib.sha256(raw).hexdigest() != SOURCE_SHA256:
        raise ValueError('ACT source hash differs from the pinned implementation')
    tree = ast.parse(raw)
    nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in NAMES]
    if {n.name for n in nodes} != NAMES:
        raise ValueError('ACT graph is incomplete')
    scope = dict(einops=einops, np=np, torch=torch, torchvision=torchvision, Tensor=Tensor, nn=nn,
                 F=F, IntermediateLayerGetter=IntermediateLayerGetter, FrozenBatchNorm2d=FrozenBatchNorm2d,
                 ACTConfig=SimpleNamespace, math=math, chain=chain, Callable=Callable,
                 ACTION='action', OBS_STATE='observation.state', OBS_IMAGES='observation.images',
                 OBS_ENV_STATE='observation.environment_state')
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
    return scope['ACT']


def config():
    # Full default transformer width/depth; not a toy reduced ACT.
    return SimpleNamespace(
        vision_backbone='resnet18', pretrained_backbone_weights='ResNet18_Weights.IMAGENET1K_V1',
        replace_final_stride_with_dilation=False, dim_model=512, n_heads=8, dim_feedforward=3200,
        n_encoder_layers=4, n_decoder_layers=1, n_vae_encoder_layers=4, use_vae=True,
        latent_dim=32, pre_norm=False, dropout=0.1, feedforward_activation='relu', kl_weight=10.0,
        chunk_size=30, n_action_steps=10, image_features={'observation.images.wrist': (3, 224, 224)},
        robot_state_feature=SimpleNamespace(shape=(6,)), env_state_feature=SimpleNamespace(shape=(6,)),
        action_feature=SimpleNamespace(shape=(6,)))


def loss(model, batch):
    import torch
    prediction, (mu, logvar) = model(batch)
    valid = ~batch['action_is_pad'].unsqueeze(-1)
    l1 = ((prediction-batch['action']).abs()*valid).sum()/(valid.sum()*6).clamp_min(1)
    kl = (-0.5*(1+logvar-mu.square()-logvar.exp())).sum(-1).mean()
    return l1 + model.config.kl_weight*kl
