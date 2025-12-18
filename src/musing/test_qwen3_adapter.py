import pytest

torch = pytest.importorskip("torch")
nn = pytest.importorskip("torch.nn", reason="requires torch.nn")

from musing import AdapterConfig, Qwen3AdapterScaffold, SegmentBoundaryError


class _FakeAttn(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.o_proj = nn.Linear(dim, dim, bias=False)


class _FakeMLP(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.up_proj = nn.Linear(dim, dim, bias=False)


class _FakeLayer(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.self_attn = _FakeAttn(dim)
        self.mlp = _FakeMLP(dim)


class _FakeDecoder(nn.Module):
    def __init__(self, num_layers: int, dim: int):
        super().__init__()
        self.layers = nn.ModuleList([_FakeLayer(dim) for _ in range(num_layers)])


class _FakeModel(nn.Module):
    def __init__(self, num_layers: int = 2, dim: int = 8):
        super().__init__()
        self.model = _FakeDecoder(num_layers, dim)


def _zero_weights(scaffold: Qwen3AdapterScaffold, site: str):
    module = scaffold.adapter_modules[site]
    a = torch.zeros_like(module.A)
    b = torch.zeros_like(module.B)
    return a, b


def test_site_shapes_cover_expected_targets():
    scaffold = Qwen3AdapterScaffold(
        _FakeModel(),
        config=AdapterConfig(rank=2, target_layers=[0, 1]),
    )
    shapes = scaffold.site_shapes()
    assert "layer0.attn_o" in shapes
    assert "layer1.mlp_up" in shapes
    assert shapes["layer0.attn_o"].rank == 2


def test_adapter_updates_require_segment_boundary():
    scaffold = Qwen3AdapterScaffold(
        _FakeModel(),
        config=AdapterConfig(rank=1, target_layers=[0]),
    )
    scaffold.start_segment()
    site = "layer0.attn_o"
    a, b = _zero_weights(scaffold, site)

    with pytest.raises(SegmentBoundaryError):
        scaffold.apply_adapter_weights({site: (a, b)})
    with pytest.raises(SegmentBoundaryError):
        scaffold.set_alpha(1.0)

    scaffold.end_segment()
    scaffold.apply_adapter_weights({site: (a, b)})
    scaffold.set_alpha(1.0)
    assert scaffold.adapter_modules[site].alpha == 1.0
