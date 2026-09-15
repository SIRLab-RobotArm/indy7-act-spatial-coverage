# Publication Figures

These figures are prepared for a 120 mm print width with labels sized for readability at the final output scale.

| Basename | Content | Intended print size |
|---|---|---|
| `experimental-setup` | Experimental apparatus and camera layout | Paper layout dependent |
| `experimental-conditions` | Experimental condition summary | Paper layout dependent |
| `task-demonstration` | Demonstration sequence | Paper layout dependent |
| `position-results` | Position-based result summary | Paper layout dependent |
| `success-groups` | Success-rate group comparison | Paper layout dependent |
| `gripper-open-and-grasp` | Open Mark 7 hand and PVC-U cylinder grasp | 120 x 48 mm |
| `gripper-cad-dimensions` | Equal-scale front and side CAD projections with envelope dimensions | 120 x 86 mm |

Use PNG files for preview or office documents and PDF files in LaTeX. SVG is also available for the two gripper figures. In those files, PDF and SVG annotations are vector graphics while the photographs and CAD surfaces are rasterized. The gripper PNG files are exported at 600 dpi. Avoid inserting them below the intended width because the labels will become smaller than their designed size.

The figures contain no title or specification box so that captions can be managed by the paper. The photographs retain their original content, brightness, and color; only rectangular cropping and output resampling were applied.

## Suggested captions

**Photograph:** The Mark 7 five-finger robotic hand in (a) an open configuration and (b) a grasp of the PVC-U cylinder. Grip tape is wrapped around the grasp region of the cylinder. The hand provides six degrees of freedom.

**CAD:** Orthographic views of the supplied Mark 7 CAD geometry: (a) front (x-z) and (b) side (y-z) projections at the same scale. The axis-aligned envelope of the geometry in its stored configuration is 157.2 mm in width, 58.9 mm in depth, and 176.6 mm in length, including the base. Dimensions are derived from the supplied STL meshes and do not represent physical measurements of the assembled hardware.

## Provenance and limitations

- The unrounded CAD envelope is 157.15065384 x 58.90369987 x 176.64608766 mm.
- The dimensions are the axis-aligned bounds of all supplied STL meshes in their shared stored coordinates.
- The CAD orientation differs from the pose in the photographs; the dimension labels apply only to the displayed CAD orientation.
- The source CAD package and the original figure-generation script are not included in this repository.
- `archive-v1/` contains the earlier PNG exports. Use the files in this directory for the paper.
- Source-image paths, crop coordinates, and SHA-256 checksums are recorded in `figure-provenance.json`.

## LaTeX example

```latex
\begin{figure}[t]
  \centering
  \includegraphics[width=\linewidth]{figures/06-publication-figures/gripper-open-and-grasp.pdf}
  \caption{The Mark 7 five-finger robotic hand in (a) an open configuration
  and (b) a grasp of the PVC-U cylinder. Grip tape is wrapped around the
  grasp region of the cylinder. The hand provides six degrees of freedom.}
  \label{fig:mark7-grasp}
\end{figure}
```
