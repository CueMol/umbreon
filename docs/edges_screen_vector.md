# Screen-Space Vector Edges

The chain extractor for the stroke edge pass (`--edges on`). It vectorizes the
per-pixel edge G-buffer AOVs (`viewZ` / `objectId` / `normal`) by crack tracing.
It shares the stylization and rasterization back half (`edges/stroke_render.hpp`)
with the retired mesh-topology source, so all style flags
(`--stroke-thickness`, `--stroke-taper`, `--stroke-smooth`, `--edge ID=spec`,
...) apply.

```sh
umbreon_cli scene.pov --edges on -W 1200 -H 1200
```

## Why

The old mesh-topology source failed on coarse tube meshes bending toward the
camera: the smooth n.v==0 contour goes sparse at grazing folds and the ray-cast
QI visibility becomes unstable, dashing the outline. A ChimeraX-style per-pixel
method has the opposite trade-off: robust, tessellation-independent
silhouettes, but no vector curves to stylize. The screen source combines the
two: pixel-exact edge detection, then VECTORIZATION into continuous polylines.

## How it works

1. **Crack classification** (`classifyCracks`): every 4-neighbor pixel pair
   of the hi-res AOVs gets a class on the "crack" between the two pixels:
   - `Silhouette` -- exactly one side is background,
   - `ObjectId`   -- both foreground and the two pixels belong to DIFFERENT
     sections (`objectId >> 2`). Primitives of the SAME section (a sphere,
     cylinder and mesh mixed in one CueMol section) never produce an internal
     edge: their boundary is skipped entirely -- no ObjectId, and it does not
     fall through to DepthGap/Crease -- so a bond embedded in an atom joins
     seamlessly. The primitive-kind bits (Sphere/Cylinder/Mesh) are ignored,
   - `DepthGap`   -- same id, view-z discontinuity. Slope-adaptive: both
     one-sided planar extrapolations must miss the far pixel (a smooth or
     grazing surface is predicted by at least one side; a pure slope change
     -- a facet kink -- is predicted exactly by the steep side), plus
     non-maximum suppression across the crack so the boundary stays one
     crack thin. DepthGap cracks carry a STRONG/weak hysteresis tier: strong
     needs the full `--stroke-depth-gap` threshold AND step dominance -- the
     raw step must exceed `stepDominanceK` (default 250) times the near
     side's wide-baseline recession slope. This kills the facet-horizon
     slivers a coarse mesh throws off at grazing incidence (a sight line
     skims a facet edge and lands a few pixels' worth of the same grazing
     ramp deeper -- a real micro-occlusion, measured ratios <= ~200 on
     mesh4, that the smooth-shaded reference does not draw), while a true
     contour (ratios >= ~500) stays strong even where the surface it sits
     on is steep. Weak cracks (above `weakGapRatio` = 0.5 of the threshold)
     trace normally but survive only with chain support (Stage 2.5). The
     background-clearance kill applies to weak cracks only, and spares a
     crack whose along-crack strip reaches the background (the terminal
     piece of a contour landing on the outline),
   - `Crease`     -- shading-normal fold (off by default, `--stroke-crease`).
2. **Crack tracing** (`traceCrackChains`): cracks form paths/loops on the
   pixel-corner lattice; region boundaries are traced into maximal chains
   deterministically. **Continuity is guaranteed by construction** -- closed
   loops or junction-to-junction paths, no voting, no chaining tolerance.
   Visibility is exact and free (the AOVs are the z-buffered first hit); no
   QI rays run.
   **Stage 2.5** (`pruneWeakChains`): hysteresis prune + retrace. A chain
   survives with any non-DepthGap edgel or ~2 final px of strong DepthGap
   edgels; a pure-weak chain survives only when BOTH its endpoint corners
   junction into kept chains with interior (non-silhouette) support on at
   least one side -- the near-cusp tail of a contour bridging the strong
   contour body and the silhouette outline. Everything else (isolated
   slivers, spurs with a free end, weak lines hugging the outline with both
   ends on it) is erased and the field is retraced, so survivors re-merge
   across the dissolved junctions into maximal chains again.
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
| `--stroke-depth-gap <f>` | 12 | DepthGap threshold, world units per lateral pixel (a slope cutoff). Facet kinks of a coarse mesh measure a few px; genuine self-occlusion steps measure hundreds -- lower it only for very subtle depth steps |
| `--stroke-screen-simplify <f>` | 0.4 | Douglas-Peucker tolerance, FINAL px |
| `--stroke-screen-smooth <int>` | 2 | Chaikin iterations |
| `--stroke-screen-minlen <f>` | 4 | drop isolated chains shorter than this, FINAL px (0 = keep all) |

The nature toggles keep their meaning under the screen source:
`--stroke-silhouette` gates the fg/bg contour AND the same-id depth gap,
`--stroke-border` gates the CROSS-section object-id boundary,
`--stroke-crease` gates the normal fold. Every ID-keyed boundary (between
sections, and between mixed primitive kinds of one section) is depth-aware:
it inks only across a genuine depth step (occlusion), while depth-continuous
contact/intersection contours -- a stick penetrating a ribbon of another
section, a bond embedded in an atom -- are always suppressed, thresholded by
`--stroke-depth-gap`. Same-section steps ink as depth-gap lines under
`--stroke-silhouette`. The `--edge-qi-*` flags are inert here
(no QI runs; visibility is exact from the z-buffer).

`UMBREON_SCREEN_EDGE_DEBUG=1` prints one stats line per frame (raw/kept
chains, edgels per class, strong DepthGap count, drawn chains) for tuning;
`=2` also lists every kept chain (bbox, class mix, strong count).
`UMBREON_SCREEN_EDGE_DUMP=<prefix>` writes Stage-1 diagnostics: a colorized
crack-lattice PPM (silhouette white, ObjectId orange, DepthGap strong red /
weak blue, NMS-suppressed cyan, bg-killed yellow, near-threshold green), a
per-crack CSV (gaps, slopes, raw step, kill reason, px-normalized) and the
raw viewZ/objectId planes. `UMBREON_SCREEN_EDGE_DUMP_ROI=x0,y0,x1,y1`
restricts the PPM/CSV to a hi-res pixel rectangle. The hysteresis internals
(`weakGapRatio`, `stepDominanceK`) are `ScreenClassifyParams` struct
defaults, deliberately not CLI flags.

## Limitations

- Chains live at hi-res pixel resolution; Chaikin+RDP smooth the staircase
  but accuracy is bounded by the (supersampled) pixel grid. Sub-pixel
  refinement from the viewZ gradients is a possible later step.
- The AOVs are captured from the FIRST hit including transparent surfaces,
  so the screen source outlines a front transparent veil where the mesh QI
  path would see through it.
- Junctions are exact T-vertices, but the chains meeting there are smoothed
  independently (endpoints pinned), which can leave a small kink.
- Hidden lines cannot be drawn (the z-buffer only sees visible surfaces).

## History

`docs/plans/edge-extraction-screenspace.md` describes a RETIRED plan for
per-pixel (non-vector) Warabi-style edges; its G-buffer AOVs were built and
are what this extractor consumes. The per-pixel compositing approach was
replaced by the Freestyle stroke pipeline, and this module closes the loop by
giving the stroke pipeline a screen-space chain source.
