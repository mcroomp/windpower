"""Checked views of the Matplotlib operations used by simulation plots.

Matplotlib leaves some forwarded keyword arguments untyped. These protocols
describe supported operations without replacing its native figure/axes objects.
Import the protocols under TYPE_CHECKING; the helpers do not import Matplotlib.

For ``plt.figure`` use ``native_fig, fig = create_figure(plt, (14, 6))``.
For ``plt.subplots`` retain its native results and use ``figure_view(native_fig)``
and ``axes_view(native_ax)`` for individual axes. Keep the native figure for
``plt.close`` and native axes for ``fig.colorbar``.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence
    from os import PathLike
    from typing import Literal, Protocol, TypedDict, overload

    import numpy as np
    from numpy.typing import ArrayLike, NDArray
    from matplotlib.artist import Artist
    from matplotlib.axes import Axes
    from matplotlib.axis import Tick
    from matplotlib.collections import PathCollection
    from matplotlib.figure import Figure, SubFigure
    from matplotlib.gridspec import GridSpec, SubplotSpec
    from matplotlib.image import AxesImage
    from matplotlib.legend import Legend
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle
    from matplotlib.text import Annotation, Text
    from matplotlib.transforms import Transform
    from matplotlib.typing import LegendLocType

    type _PlotValues = Sequence[float] | NDArray[np.float64]
    type _PlotColor = str | Sequence[float] | NDArray[np.float64]

    class _TextBox(TypedDict, total=False):
        boxstyle: str
        fc: str
        ec: str
        alpha: float

    class PlotAxes(Protocol):
        @property
        def transAxes(self) -> Transform: ...

        @property
        def figure(self) -> Figure | SubFigure: ...

        @overload
        def set_xlim(self, left: float, right: float) -> tuple[float, float]: ...
        @overload
        def set_xlim(self, left: tuple[float, float]) -> tuple[float, float]: ...
        @overload
        def set_ylim(self, bottom: float, top: float) -> tuple[float, float]: ...
        @overload
        def set_ylim(self, bottom: tuple[float, float]) -> tuple[float, float]: ...
        def get_ylim(self) -> tuple[float, float]: ...
        def set_xticks(self, ticks: ArrayLike) -> list[Tick]: ...
        def axvline(
            self, x: float, *, color: _PlotColor, linewidth: float = ...,
            linestyle: str = ..., alpha: float = ..., lw: float = ...,
            ls: str = ...,
        ) -> Line2D: ...
        def axhline(
            self, y: float, *, color: _PlotColor, linewidth: float = ...,
            linestyle: str = ..., alpha: float = ..., label: str = ...,
            lw: float = ..., ls: str = ..., zorder: float = ...,
        ) -> Line2D: ...
        def axvspan(
            self, xmin: float, xmax: float, *, color: _PlotColor,
            alpha: float = ..., zorder: float = ..., label: str = ...,
        ) -> Rectangle: ...
        def text(
            self, x: float, y: float, s: str, *, color: _PlotColor,
            fontsize: float = ..., va: Literal["top", "center"] = ...,
            rotation: float = ..., alpha: float = ..., fontweight: str = ...,
            transform: Transform = ..., ha: Literal["center"] = ...,
        ) -> Text: ...
        def annotate(
            self, text: str, xy: tuple[float, float], *, fontsize: float,
            color: _PlotColor, ha: Literal["center"] = ..., bbox: _TextBox = ...,
            xytext: tuple[float, float] = ...,
            arrowprops: dict[str, str | float] = ...,
        ) -> Annotation: ...
        def imshow(
            self, X: ArrayLike, *, origin: Literal["upper", "lower"],
            zorder: float = ...,
        ) -> AxesImage: ...
        def axis(self, arg: str, /) -> tuple[float, float, float, float]: ...
        def scatter(
            self, x: _PlotValues, y: _PlotValues, *,
            c: _PlotValues = ..., cmap: str = ..., s: float = ...,
            linewidths: float = ..., zorder: float = ..., marker: str = ...,
            color: _PlotColor = ..., label: str = ...,
        ) -> PathCollection: ...
        def plot(
            self, x: _PlotValues, y: _PlotValues, *, color: _PlotColor = ...,
            linewidth: float = ..., linestyle: str = ..., alpha: float = ...,
            label: str = ..., lw: float = ..., ls: str = ..., marker: str = ...,
            ms: float = ..., zorder: float = ...,
        ) -> list[Line2D]: ...
        def set_xlabel(self, xlabel: str, *, fontsize: float = ...) -> Text: ...
        def set_ylabel(
            self, ylabel: str, *, color: _PlotColor = ..., fontsize: float = ...,
        ) -> Text: ...
        def set_title(self, label: str, *, fontsize: float = ...) -> Text: ...
        @overload
        def legend(
            self, *,
            loc: LegendLocType,
            fontsize: float, ncol: int = ..., title: str = ...,
            title_fontsize: float = ..., framealpha: float = ...,
        ) -> Legend: ...
        @overload
        def legend(
            self, handles: Sequence[Artist], labels: Sequence[str],
            *, loc: LegendLocType,
            fontsize: float, ncol: int = ..., title: str = ...,
            title_fontsize: float = ..., framealpha: float = ...,
        ) -> Legend: ...
        @overload
        def legend(
            self, *, handles: Sequence[Artist],
            loc: LegendLocType, fontsize: float, ncol: int = ..., title: str = ...,
            title_fontsize: float = ..., framealpha: float = ...,
        ) -> Legend: ...
        def set_aspect(
            self, aspect: Literal["equal"], *, adjustable: Literal["datalim"],
        ) -> None: ...
        def grid(self, visible: bool, *, alpha: float) -> None: ...
        def twinx(self) -> PlotAxes: ...
        def tick_params(
            self, *, labelsize: float = ..., axis: Literal["y"] = ...,
            labelcolor: str = ...,
        ) -> None: ...
        def get_legend_handles_labels(self) -> tuple[list[Artist], list[str]]: ...

    class _PlotColorbar(Protocol):
        @property
        def ax(self) -> PlotAxes: ...

        def set_label(self, label: str, *, fontsize: float) -> None: ...

    class PlotFigure(Protocol):
        def suptitle(
            self, t: str, *, fontsize: float, fontweight: str = ...,
        ) -> Text: ...
        def add_subplot(self, spec: SubplotSpec, /) -> Axes: ...
        def add_gridspec(
            self, nrows: int, ncols: int, *, wspace: float,
        ) -> GridSpec: ...
        def colorbar(
            self, mappable: PathCollection, *, ax: Axes, pad: float,
        ) -> _PlotColorbar: ...
        def savefig(
            self, fname: str | PathLike[str], *, dpi: float,
            bbox_inches: Literal["tight"] = ...,
        ) -> None: ...

    class _Pyplot(Protocol):
        def figure(self, *, figsize: tuple[float, float]) -> Figure: ...


def figure_view(figure: PlotFigure) -> PlotFigure:
    """Expose the supported plotting operations with a checked native input."""
    return figure


def axes_view(axes: PlotAxes) -> PlotAxes:
    """Expose the supported plotting operations with a checked native input."""
    return axes


def create_figure(
    pyplot: _Pyplot, figsize: tuple[float, float],
) -> tuple[Figure, PlotFigure]:
    """Retain the native ownership handle alongside the checked plotting view."""
    figure = pyplot.figure(figsize=figsize)
    return figure, figure


def add_subplot(
    figure: PlotFigure, spec: SubplotSpec,
) -> tuple[Axes, PlotAxes]:
    """Retain the native colorbar handle alongside the checked plotting view."""
    axes = figure.add_subplot(spec)
    return axes, axes
