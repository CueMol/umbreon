# Tone Hatching (`--hatch`)

NPR shading that renders illumination as **mark density** -- crosshatching,
halftone screens, stippling, colored-pencil shading -- instead of continuous
tone. Aimed at CueMol publication and presentation figures.

```sh
# a hand-drawn colored-pencil ribbon figure, one flag
umbreon_cli scene.pov --hatch on --hatch-look richardson --edges on \
  -W 1600 -H 1250 --supersample 3
```

Composable with both edge methods (`--edges` / `--obj-edges`): the hatch
carries the tone, the edges carry the contours. Off by default; with
`--hatch off` no AOV is allocated and the output is byte-identical to a build
without the feature.

Design record: [plans/npr-tone-hatching.md](plans/npr-tone-hatching.md).
Public API: `<umbreon/render/hatch_types.hpp>`, `<umbreon/npr/hatch_shade.hpp>`.

---

## 1. How it works

Tone **generation** and ink **consumption** are separated:

| Stage | Where | What |
|---|---|---|
| Tone | `hit_shader`, next to AO | A lighting-only scalar per first hit (`ToneRecipe`), written to the `hatchTone` / `hatchMask` AOVs |
| Ink | End of `renderFrame` | The mark lattices are evaluated and multiplied into the display-encoded frame (`applyHatch`) |

The tone is deliberately **not** the color luminance. CueMol figures are color
coded by chain and secondary structure; using color luminance would hatch a
dark blue helix black and a yellow one white at identical illumination. The
tone is built from lighting only:

```
tone = ambient + diffuseWeight * SUM_lights[ saturate(N.L)^brilliance * shadow * luma(light) ]
tone *= contactAo^contactAoPow * shapeAo^shapeAoPow
```

The tone is generated at the supersampled resolution and box-downsampled with
the frame -- that average **is** the tone antialiasing.

### Nesting (why marks never crawl)

Marks sit on a lattice of pitch `spacing / 2^subdiv`. Lattice index `j`
appears at nesting level `firstLevel(j, K)` (a count-trailing-zeros) with a
tone threshold that decreases with the level, so darkening the tone only ever
**inserts** marks between existing ones. A mark that has appeared never
disappears and never moves: every perturbation is a pure function of the
lattice-index hash and the along-mark coordinate, never of the tone. This is
the Tonal Art Map guarantee of Praun et al. (SIGGRAPH 2001), reduced to a bit
trick; the grow-from-zero-width fade follows Webb et al. (NPAR 2002).

Regression-tested: per-pixel ink coverage is monotone non-decreasing as the
tone darkens, under every preset and perturbation.

### Ink resolution and pixel units

`--hatch-res hi` (**default**) lays the ink at the supersampled resolution and
lets the box downsample average the strokes into a drawing-like grain.
`--hatch-res out` inks at the output resolution instead, keeping strokes
pixel-exact and crisp.

**All pixel-unit parameters mean OUTPUT pixels in both modes.** Under
`hi` the renderer scales them onto the hi-res grid, so the same numbers give
the same look at any `--supersample` factor. The practical difference is the
minimum stroke pitch:

| Mode | Minimum pitch | Character |
|---|---|---|
| `hi` (default) | 2 / ss output px (ss=4 -> 0.5 px) | fine grain; sub-pixel strokes merge into an exact coverage tone |
| `out` | 2 output px | crisp, individually resolvable strokes |

A minimum feature size exists because the ink is a raster: below ~2 px per
lattice step the marks alias against the pixel grid. `subdiv` is clamped so
the finest level always clears it.

Under `hi` the frame is display-encoded before the downsample, so the linear
color denoisers cannot run; they are normalized off automatically.

---

## 2. Looks (`--hatch-look`)

A **look** configures the whole style at once: paper/ink model, tone recipe,
pencil pressure and the mark layers. This is the recommended entry point.

```sh
umbreon_cli scene.pov --hatch on --hatch-look richardson --edges on
```

| Look | Paper / ink | Marks | Use |
|---|---|---|---|
| `richardson` | paper base, ink from each section's own color, 3 pencils of one hue, pressure `inkShadeDark 0.4`, tone fog on | `pencil`, pitch 2 px, width 1.8 px | Jane-Richardson-style colored-pencil ribbon drawings |
| `ink-cross` | white paper, fixed black ink | `pen-cross` | plain pen-and-ink monochrome figures |
| `manga` | flat albedo fill (posterized to 4 steps), fixed black ink | `screentone-60` | comic-style flat fill under a halftone screen |

### `richardson` in detail

Reproduces the look of the hand-drawn ribbon figures: **there is no flat
fill**. The paper carries the highlights, midtones are stroke density, and
darks are denser strokes of a *darker pencil of the same hue*. Its settings:

| Setting | Value | Why |
|---|---|---|
| `base` / `ink` | Paper / FromAlbedo | every section draws in its own color on bare paper |
| `paperColor` | `#F0ECDD` | warm drawing paper |
| layers | `pencil` x3, `inkScale` 1.0 / 0.62 / 0.38 | three pencils: light wash, darker cross, darkest shadow core |
| `inkShadeDark` | 0.4 | pencil pressure: the same stick darkens in shadow |
| `spacing` / `width` | 2.0 / 1.8 output px | fine strokes; `--supersample` decides how far below a pixel they land |
| `tone.whitePoint` | 0.97 | opens the lit side up to bare paper |
| `tone.gamma` | 2.2 | pushes midtones into the stroke range |
| `tone.specularCut` | 0.1 | punches the highlight through as pure paper |
| `toneFog` | on | the far side fades into the paper |

Scene-side companions (not part of the look, since they are scene data):

```sh
--shadows on \
--declare _light_inten=1.3 --declare _flash_frac=0.15 --declare _amb_frac=0
```

The CueMol export is dominated by a camera-mounted flash light, which flattens
the shading; rebalancing toward the directional key light is what gives each
ribbon its light-to-dark gradient. AO is best left **off** here: the tone
should follow surface orientation, and AO darkening in the crevices muddies
it.

---

## 3. Mark presets (`--hatch-preset`)

A preset chooses only the **layers** (mark kind, angles, thresholds,
perturbations). Use it to restyle the marks of a look, or on its own.

| Preset | Layers | Character |
|---|---|---|
| `pen-cross` (default) | 3 Line: 45 / -45 / 0 deg | hard crosshatch, no perturbation, seed-independent |
| `pencil` | 3 Line: 55 / -35 / 80 deg, `inkScale` 1 / 0.62 / 0.38 | individual strokes: wobble, per-stroke length/pressure/angle, tapered ends, paper tooth |
| `engraving` | 1 Line, `subdiv` 3 | copperplate: one direction, tone by insertion + width modulation |
| `stipple` | 1 Dot, `jitter` 0.4 | scientific stippling |
| `screentone-60` | 1 Dot at 45 deg, `subdiv` 0 | classic AM halftone screen (~60 lpi on a 300 dpi figure) |
| `manga-square` | 1 Dot at 45 deg, `shapeExponent` 16 | square-element screen |

Layers act as **pencils**: each has its own tone threshold range and its own
ink darkness (`inkScale`), so a drawing reaches its darks both by adding
strokes and by switching to a darker stick.

### Mark shapes (Dot layers)

`shapeExponent` is one continuous Lp knob: 1 = diamond, 2 = circle, 4 =
rounded square, >= 16 = square. Radii are area-normalized by `1/sqrt(A_p)`, so
**changing the shape does not change the apparent density** (regression
tested). Past ~50% coverage the dots freeze and white holes grow on the dual
lattice, so full black is reachable with a continuous, monotone transition.

Dot screens benefit from the default `--hatch-res hi` twice over: the box
downsample antialiases each dot's rim (at `out` the same dot quantizes into a
hard blocky cell), and the screen can go finer than one output pixel simply by
asking for it. A 300 dpi figure at `--supersample 4` takes
`--hatch-spacing 1.5` (or lower) without breaking up -- the dot count is a
parameter, not a resolution limit.

---

## 4. Options

### Style

| Flag | Default | Meaning |
|---|---|---|
| `--hatch <on\|off>` | off | master switch |
| `--hatch-look <name>` | -- | complete look (see 2) |
| `--hatch-preset <name>` | `pen-cross` | mark layers (see 3) |
| `--hatch-mode <ink\|over>` | `ink` | `ink`: rebuild the picture from paper/fill + ink (GI and color denoisers normalized off). `over`: multiply the hatch into the shaded frame |
| `--hatch-base <paper\|albedo>` | `paper` | ink-mode background: paper, or a flat unshaded fill of the object color |
| `--hatch-ink <fixed\|albedo>` | `fixed` | ink color: a fixed color, or each surface's own color |
| `--hatch-ink-color <#RRGGBB>` | `#000000` | fixed ink (display-encoded) |
| `--hatch-paper-color <#RRGGBB>` | `#FFFFFF` | paper (display-encoded) |
| `--hatch-ink-shade <f>` | 1 | pencil pressure: ink darkens toward `ink * f` in shadow |
| `--hatch-min-contrast <f>` | 0.25 | minimum display-luma gap between base and ink |

The four base/ink combinations are all useful: paper+black (pen figure),
paper+object color (colored pencil), flat fill+black (comic), flat fill+object
color (print-like). Whichever you pick, the **contrast guarantee** keeps the
ink visible: if the ink is not darker than its base by `--hatch-min-contrast`
it is darkened hue-preserving; on a base too dark for that it is lifted
instead (bright hatching on dark ground). Only the ink moves -- moving the
base would break the color coding.

### Density and geometry

| Flag | Default | Meaning |
|---|---|---|
| `--hatch-spacing <px>` | preset | base lattice pitch, all layers (output px) |
| `--hatch-width <px>` | preset | mark width, all layers (output px) |
| `--hatch-res <hi\|out>` | `hi` | ink resolution (see 1) |
| `--hatch-layer <i:k=v,...>` | -- | per-layer override, repeatable |

`--hatch-layer` keys: `kind=line|dot`, `angle`, `spacing`, `subdiv`, `width`,
`tonehi`, `tonelo`, `fade`, `opacity`, `inkscale`, `soft`, `seed`, `shape`,
`aspect`, `dotangle`, `jitter`, `invert=on|off`, `wobble`, `wobwave`,
`wjitter`, `slen`, `sgap`, `taper`, `anglejitter`, `lenjitter`, `tooth`,
`toothscale`.

```sh
# start from pencil, but make the shadow pencil darker and the strokes longer
--hatch-preset pencil --hatch-layer "2:inkscale=0.25" --hatch-layer "0:slen=80"
```

### Tone

| Flag | Default | Meaning |
|---|---|---|
| `--hatch-tone <k=v,...>` | -- | tone recipe |
| `--hatch-tone-fog <on\|off>` | on | fade the tone toward paper with the scene fog |

`--hatch-tone` keys: `diffuse` (weight of the summed per-light diffuse),
`ambient` (floor; 0 crushes shadows to solid ink), `contact` / `shape` (AO
exponents), `black` / `white` (level remap), `gamma` (artistic curve),
`speccut` (blow the specular highlight out to paper), `levels` (posterize the
tone to N bands).

Tone pipeline order: linear tone -> black/white remap -> `gamma` -> display
encode -> `levels` -> threshold. Every stage maps 1 to 1, so a fully lit
surface stays exactly ink-free.

`--hatch-tone-fog` makes distant strokes thin out and (with
`--hatch-ink-shade`) lighten, matching how the silhouette ink fades -- the way
a drawing lightens its far side. Drive its strength from the scene's fog.

### Per-section styling

`--hatch-style ID=spec` overrides the style for one CueMol section, mirroring
`--edge ID=spec` (same `_show` prefix stripping and `--list-groups`
discovery):

```sh
umbreon_cli scene.pov --hatch on \
  --hatch-style "_34_35=base=albedo" \
  --hatch-style "_34_38=base=paper:ink=albedo:density=2:tone=0.75"
```

Spec entries: `off` (leave the section normally shaded), `base=paper|albedo`,
`ink=fixed|albedo`, `color=#RRGGBB`, `tone=F` (tone scale), `layers=MASK`
(layer bitmask), `density=F` (mark-density multiplier), `width=F`.

`density` matters for small features: a thin stick or ligand catches only a
couple of marks at the ribbon's pitch and reads as flat, so give it a finer
grain -- or a flat fill, which carries color better at that size.

### AO interaction

When AO is on (`--ao-samples N`) with `--hatch`, the CLI switches to
NPR-friendly AO defaults unless you set them yourself: `--ao-res out`
(the coarse output-resolution gather doubles as a tone denoiser),
`--ao-ld on` (free variance reduction) and `--ao-res-fallback-mul 4`
(oversamples the bilateral-rejected rim pixels so the binarization does not
speckle).

---

## 5. Recipes

```sh
# Richardson-style colored pencil (the reference recipe)
umbreon_cli scene.pov -W 1600 -H 1250 --supersample 3 --shadows on \
  --declare _light_inten=1.3 --declare _flash_frac=0.15 --declare _amb_frac=0 \
  --hatch on --hatch-look richardson --edges on

# monochrome pen figure
umbreon_cli scene.pov --hatch on --hatch-look ink-cross --edges on

# comic style: flat fill + halftone screen
umbreon_cli scene.pov --hatch on --hatch-look manga --edges on

# scientific stippling on paper
umbreon_cli scene.pov --hatch on --hatch-preset stipple --edges on

# hatch over the shaded color instead of rebuilding the picture
umbreon_cli scene.pov --hatch on --hatch-mode over
```

Scene-side settings that matter as much as the hatch flags: a directional key
light (see 2), the background / fog colors (set them in CueMol or the `.pov`;
the paper color only fills the object interior, not the background), and the
palette itself -- with `ink=albedo` the strokes ARE the object colors, so a
muted palette gives muted strokes.

---

## 6. Library use

`applyHatch` is exported as a plain-pointer image op, so a host can hatch a
tone it produced itself (a hand-painted or retouched one, for instance):

```cpp
#include <umbreon/npr/hatch_shade.hpp>

umbreon::HatchOptions opt;
umbreon::applyHatchLook(opt, "richardson");
opt.enable = true;
umbreon::applyHatch(w, h, rgba, tone, mask, albedo, opt);
```

`rgba` is the display-encoded canvas the ink multiplies into (the composite
only ever darkens, so contour ink already present survives); `tone` is the
linear shading tone and `mask` the surface coverage (0 = background, left
untouched). Deterministic: every output value is a pure function of its
coordinates and the inputs, so thread count never changes the result.

Through `render()`, set `RenderOptions::hatch` and read the `hatchTone` /
`hatchMask` / `hatchGroup` AOVs from `FrameResult` if you want the tone
itself.

---

## 7. Not implemented

- **Object-space stroke direction.** Strokes run at fixed screen-space angles,
  not along the surface (a ribbon's strokes do not follow its flow). The
  design keeps the extension point (`hatchUv`, falling back to screen
  coordinates when absent), but the direction field itself is future work.
- Object-space UV parameterization (lapped textures), TAM texture generation,
  temporal coherence for animation, GPU implementation, optimization-based
  stipple placement (the jitter approximates it).

## References

- Praun, Hoppe, Webb, Finkelstein, *Real-Time Hatching*, SIGGRAPH 2001
- Webb, Praun, Finkelstein, Hoppe, *Fine Tone Control in Hardware Hatching*, NPAR 2002
- Winkenbach, Salesin, *Computer-Generated Pen-and-Ink Illustration*, SIGGRAPH 1994
- Tarini, Cignoni, Montani, *Ambient Occlusion and Edge Cueing for Enhancing Real Time Molecular Visualization*, TVCG 2006
- Ostromoukhov, Hersch, *Artistic Screening*, SIGGRAPH 1995
