"""FastAPI / Starlette compatibility patches."""
import starlette.routing

_original_router_init = starlette.routing.Router.__init__


def _patched_router_init(self, *args, **kwargs):
    kwargs.pop("on_startup", None)
    kwargs.pop("on_shutdown", None)
    _original_router_init(self, *args, **kwargs)


def apply_patches() -> None:
    starlette.routing.Router.__init__ = _patched_router_init