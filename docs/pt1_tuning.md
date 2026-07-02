# pt1 Path-Traced GI: Tuning Guide

`--integrator pt1` enables the per-pixel one-bounce path-traced indirect integrator.
This document explains the energy model, depth cue levers, and recommended recipes.
For the algorithmic design and conventions, see [pt1_design.md](pt1_design.md).

## Quick start

```sh
umbreon_cli <scene>.pov -W 1200 -H 1200 \
  --integrator pt1 --supersample 1 --seed 0
```

This gives physically-grounded indirect diffuse illumination in about 0.9 s for a
97k-triangle molecular surface at 1200x1200 (vs ~13 s for the irradiance cache).

## Quality presets

`--quality <draft|high|ultra>` expands to a pt1 configuration at the point of
appearance; put it FIRST so later explicit flags override individual values.
Measured on the 97k-triangle scene at 1200x1200, `--supersample 1`:

| Preset | Expansion | pt1 time | Character |
|---|---|---|---|
| `draft` | 8 spp, half res, 1 bounce | ~0.6 s | interactive checks |
| `high` | 64 spp, full res, 2 bounces | ~11 s | full-res detail + multi-bounce fill |
| `ultra` | 256 spp, full res, 3 bounces | ~48 s | near-converged reference |

All presets imply `--integrator pt1` (and therefore `--gi on`). Multi-bounce
(`--gi-bounces`) routes light around corners, so deep pockets brighten
physically instead of staying pitch black; Russian roulette keeps 3-bounce
paths unbiased without tracing every one.


## How the energy balance works

The `.pov` light setup has three additive terms:

```
total = direct_spec  (_light_inten * (1-_amb_frac) * (1-_flash_frac))
      + flash        (_light_inten * (1-_amb_frac) *    _flash_frac  )
      + ambient      (_light_inten *    _amb_frac                    )
```

- **direct_spec**: a distant area-light that casts shadows and creates the main
  directional shading.
- **flash**: a shadowless camera-aligned fill light that reduces perceived
  contrast. Higher `_flash_frac` flattens the image.
- **ambient**: with `--gi off` this is a constant flat term. With `--integrator pt1`
  this energy is routed through the pt1 gather: the integrator computes how much
  of the hemisphere is occluded at each surface point and returns a physically-
  correct irradiance that replaces the flat ambient.

The GI-on defaults (`_amb_frac=0.5`, `_flash_frac=0.5`) split energy equally
between the two paths. Only the energy flowing through `_amb_frac` carries
occlusion information; the rest remains flat.

**Key insight**: raising `_amb_frac` transfers more energy from flat lighting
into the path-traced gather, directly amplifying perceived depth without
changing the total scene brightness much.


## Depth cue recipes

All examples use `--integrator pt1 --supersample 1 -W 1200 -H 1200 --seed 0`.

### Baseline (GI-on defaults)

```sh
umbreon_cli <scene>.pov --integrator pt1 --supersample 1 -W 1200 -H 1200 --seed 0
```

`_amb_frac=0.5`, `_flash_frac=0.5`. Moderate depth cue; flash fill keeps the
shadowed side readable.

### More depth: raise `_amb_frac`

```sh
... --declare _amb_frac=0.7
```

Moves 70% of energy through the path-traced gather. Concave pockets and
inter-domain grooves darken; exposed convex patches stay bright. The flash fill
(`_flash_frac` unchanged at 0.5) still counteracts some of the contrast.

### Strongest depth: raise `_amb_frac` + reduce flash

```sh
... --declare _amb_frac=0.7 --declare _flash_frac=0.25
```

Recommended starting point for maximum depth cue. The reduced flash means the
directional shading from `direct_spec` is also more visible, so the combined
effect is a strong contrast between lit and shadowed areas.

### Directional sky gradient

```sh
... --sky gradient --ao-ground "#666666"
```

Changes the gather's sky model from uniform white to a zenith-bright / ground-dark
gradient (camera-up axis). Upward-facing patches receive more indirect light than
downward-facing ones, adding a shape cue that is independent of occlusion. The
`--ao-ground` value sets the ground hemisphere color; `#666666` (40% grey) gives
a noticeable but not extreme gradient.

### Combined: gradient sky + high `_amb_frac`

```sh
... --declare _amb_frac=0.7 --sky gradient --ao-ground "#666666"
```

Both the occlusion contrast and the directional gradient contribute. Occlusion
dominates in tight cavities; the gradient adds shape cue on open surfaces.


## Parameter reference

| Flag | Default | Effect |
|---|---|---|
| `--declare _amb_frac=<f>` | 0.5 (GI-on) | Fraction of energy routed through pt1 gather. Higher → more depth cue. |
| `--declare _flash_frac=<f>` | 0.5 (GI-on) | Fraction of non-ambient energy in the camera fill (shadowless). Lower → more directional contrast. |
| `--declare _light_inten=<f>` | 1.6 (GI-on) | Overall scene brightness. |
| `--sky uniform\|gradient` | uniform | Sky model for gather misses. `gradient` makes upper hemisphere brighter. |
| `--sky-radiance r,g,b` | 1,1,1 | Zenith radiance for the gather sky term. |
| `--ao-ground #RRGGBB` | #ffffff | Ground hemisphere color for `--sky gradient`. |
| `--spp N` | 8 | Gather samples per pixel (half-res grid). Higher → less noise before denoise, more time. |
| `--indirect-res full\|half` | half | half-res gathers at W/2×H/2 + joint bilateral upsample. `full` is ~4x slower. |
| `--denoise on\|off` | on | OIDN denoiser on the indirect E buffer (before compositing). |
| `--gi-intensity <f>` | 1.0 | Global gain on the indirect contribution. Raising this brightens shadowed areas but can saturate exposed patches. |
| `--gi-max-dist <f>` | inf | Gather ray tfar. Default is physical (infinite); lower values (e.g. `--gi-max-dist 5`) give cache-like local contrast. |
| `--gi-env-intensity <f>` | 1.0 | Scale on the sky/ground radiance seen by gather misses. |
| `--seed N` | 0 | Deterministic RNG seed. Different seeds give independent noise patterns. |
| `--pt1-ld on\|off` | off | Stratified first-bounce sampling (Hammersley + per-pixel Cranley-Patterson shift). Lower variance at the same spp; recommended on. |
| `--pt1-clamp <L>` | 0 (off) | Per-sample luminance clamp; suppresses rare bright fireflies on multi-bounce paths at a small bias cost. |


## Path tracing: what the integrator does

pt1 computes indirect diffuse illumination with a one-bounce cosine-weighted
hemisphere gather per pixel:

1. **Primary hit**: the standard direct shading pass runs first (shared with the
   irradiance cache path). This fills position, shading normal, geometry normal,
   and albedo buffers.

2. **Gather** (`pt1GatherPoint`): for each mesh-hit pixel, `spp` directions are
   drawn from a cosine-weighted hemisphere. Each direction is tested against the
   scene with Embree:
   - **Hit**: `oneBounceRadiance` evaluates the outgoing radiance at the bounce
     point (direct illumination only; no emission to avoid double-counting).
   - **Miss**: `environmentRadiance` returns the sky/ground gradient value.
   - Directions with `dot(wi, N_geom) <= 0` (shading/geometry normal mismatch)
     are treated as occluded (contribution 0, count toward `spp`).
   The mean is stored as `E_stored = (1/spp) * sum(L_i)` — the same energy
   convention as the irradiance cache (no 1/pi factor; see pt1_design.md).

3. **Denoising**: OIDN denoises the half-res (or full-res) E buffer with albedo
   and normal guides before upsampling. This lets low spp (8) produce clean
   output in under 1 s.

4. **Upsampling** (half-res only): joint bilateral upsample to full render
   resolution, using full-res normal and depth as edge stops. Normal weight
   `dot(N_full, N_half)^32` and depth weight `exp(-|z_full - z_half| / (0.02 * z_full))`
   prevent GI from bleeding across sharp edges.

5. **Composite**: `color += gi_intensity * albedo * E_stored`, identical to the
   cache path.

### Expected differences vs the irradiance cache

| Aspect | irradiance cache | pt1 |
|---|---|---|
| Gather tfar | 0.1 × scene diagonal | infinity (default) |
| Concave pocket contrast | strong (short tfar emphasises locality) | physically correct; deep pockets may appear slightly lighter |
| Noise pattern | record-placement artifacts | smooth after OIDN |
| Speed (97k tri, 1200x1200) | ~13 s | ~0.9 s (half-res + OIDN) |
| Multi-bounce | no | no (1-bounce; architecture allows extension) |

The gather tfar difference is the main A/B visual difference. Pass
`--gi-max-dist <0.1 * scene_diagonal>` to pt1 to match the cache's locality.


## Fog (depth fade)

CueMol-exported `.pov` files include a linear fog term (default: start 200,
end 294 in world units for the standard `_distance=200` camera). This fades
geometry that is farther from the camera toward the background color, giving a
soft distance cue that is independent of the GI integrator. It is applied
automatically when present in the scene file.


## Known limitations

- **Outline decoration primitives** (baked NPR silhouette cylinders / joint
  dots) receive no indirect illumination and act as black occluders for gather
  rays, matching the AO behaviour. REAL CSG primitives (spacefill atom balls,
  bond cylinders) participate fully under pt1: they receive gathered indirect
  in place of their constant ambient and scatter light onto nearby surfaces.
  The irradiance cache remains mesh-only.
- **Environment dome lights** (`--env-light`) count the sky as direct light.
  Using them with `--integrator pt1` double-counts the sky (the gather miss term
  also sees it). Use `--sky` / `--sky-radiance` for the GI sky instead.
- **Default is one bounce**. Pass `--gi-bounces 2` (or 3) for multi-bounce
  path continuation with Russian roulette from the third segment; deep pockets
  brighten physically (light routed around corners) at roughly +50% gather
  cost per extra bounce at 8 spp.
