# Screen-Space Vector Edges (`--stroke-source screen`)

A second chain extractor for the stroke edge pass (`--edges on`). Instead of
extracting feature edges from MESH TOPOLOGY (the default Freestyle-faithful
`--stroke-source mesh`), it vectorizes the per-pixel edge G-buffer AOVs
(`viewZ` / `objectId` / `normal`) by crack tracing. Both sources share the
same stylization and rasterization back half (`edges/stroke_render.hpp`), so
all style flags (`--stroke-thickness`, `--stroke-taper`, `--stroke-smooth`,
`--edge ID=spec`, ...) apply to either.

```sh
umbreon_cli scene.pov --edges on --stroke-source screen -W 1200 -H 1200
```

## Why

The mesh source fails on coarse tube meshes bending toward the camera: the
smooth n.v==0 contour goes sparse at grazing folds and the ray-cast QI
visibility becomes unstable, dashing the outline. A ChimeraX-style per-pixel
method has the opposite trade-off: robust, tessellation-independent
silhouettes, but no vector curves to stylize. The screen source combines the
two: pixel-exact edge detection, then VECTORIZATION into continuous polylines.

## How it works

1. **Crack classification** (`classifyCracks`): every 4-neighbor pixel pair
   of the hi-res AOVs gets a class on the "crack" between the two pixels:
   - `Silhouette` -- exactly one side is background,
   - `ObjectId`   -- both foreground, section/kind id differs,
   - `DepthGap`   -- same id, view-z discontinuity. Slope-adaptive: both
     one-sided planar extrapolations must miss the far pixel (a smooth or
     grazing surface is predicted by at least one side; a pure slope change
     -- a facet kink -- is predicted exactly by the steep side), plus
     non-maximum suppression across the crack so the boundary stays one
     crack thin, plus a background-clearance kill near the outline,
   - `Crease`     -- shading-normal fold (off by default, `--stroke-crease`).
2. **Crack tracing** (`traceCrackChains`): cracks form paths/loops on the
   pixel-corner lattice; region boundaries are traced into maximal chains
   deterministically. **Continuity is guaranteed by construction** -- closed
   loops or junction-to-junction paths, no voting, no chaining tolerance.
   Visibility is exact and free (the AOVs are the z-buffered first hit); no
   QI rays run.
3. **Cleanup**: collinear collapse, Chaikin corner cutting (endpoints pinned
   at junctions), Douglas-Peucker simplification, junction-aware speck
   filter (isolated specks and free-end spurs drop; short junction-to-
   junction pieces of a larger boundary survive).
4. **Draw**: chains are split into same-class runs (sharing their boundary
   vertex, so geometry stays continuous across a style change), mapped onto
   the per-section `EdgeStyle` slots (Silhouette -> `sil`, ObjectId -> `obj`,
   DepthGap -> `disc` with a `sil` fallback, Crease -> `crease`) and handed
   to the shared stroke renderer.

## Flags

| Flag | Default | Effect |
|---|---|---|
| `--stroke-source <mesh\|screen>` | mesh | chain extractor; mesh output is byte-identical to before this feature |
| `--stroke-depth-gap <f>` | 12 | DepthGap threshold, world units per lateral pixel (a slope cutoff). Facet kinks of a coarse mesh measure a few px; genuine self-occlusion steps measure hundreds -- lower it only for very subtle depth steps |
| `--stroke-screen-simplify <f>` | 0.4 | Douglas-Peucker tolerance, FINAL px |
| `--stroke-screen-smooth <int>` | 2 | Chaikin iterations |
| `--stroke-screen-minlen <f>` | 4 | drop isolated chains shorter than this, FINAL px (0 = keep all) |

The nature toggles keep their meaning under the screen source:
`--stroke-silhouette` gates the fg/bg contour AND the same-id depth gap,
`--stroke-border` gates the object-id boundary, `--stroke-crease` gates the
normal fold. The `--edge-qi-*` flags are mesh-only (no QI runs here).

`UMBREON_SCREEN_EDGE_DEBUG=1` prints one stats line per frame (traced chains,
edgels per class, drawn chains) for tuning.

## Limitations

- Chains live at hi-res pixel resolution; Chaikin+RDP smooth the staircase
  but accuracy is bounded by the (supersampled) pixel grid. Sub-pixel
  refinement from the viewZ gradients is a possible later step.
- The AOVs are captured from the FIRST hit including transparent surfaces,
  so the screen source outlines a front transparent veil where the mesh QI
  path would see through it.
- Junctions are exact T-vertices, but the chains meeting there are smoothed
  independently (endpoints pinned), which can leave a small kink.
- Hidden lines cannot be drawn (the z-buffer only sees visible surfaces);
  the mesh source remains the option for hidden-line styles.

## History

`docs/plans/edge-extraction-screenspace.md` describes a RETIRED plan for
per-pixel (non-vector) Warabi-style edges; its G-buffer AOVs were built and
are what this extractor consumes. The per-pixel compositing approach was
replaced by the Freestyle stroke pipeline, and this module closes the loop by
giving the stroke pipeline a screen-space chain source.
