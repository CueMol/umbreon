# Screen-Space (Warabi-style) NPR Edge Rendering for umbreon

*Plan location:* `docs/plans/edge-extraction-screenspace.md`
*Status:* planned, not started. Read-only design record.

> **Reviewer note (citation pass).** Every `file:line` below was re-verified against the working tree. Corrections folded in since the first draft: (a) the baked-POV-edge removal hook had a self-contradictory anchor (`main.cpp:131-137` is the geometry *move*, not a pre-move point, and the `--alpha` resolution it wants to parallel is at `:160-189`, *after* the move) — resolved in §5.3; (b) the per-pixel section key `HitShade::group` is `int` (`hit_shader.hpp:29`), not `uint16_t` (the underlying primitive tables are `uint16_t`); (c) the background sentinel is an *initial value*, not something written on the no-hit break; (d) the AOV-allocation gate lives in `EmbreeRenderer::render` (`embree_renderer.cpp:77-82`), not `umbreon.cpp:79-82`; (e) the cylinder material tables are built in `curve_build.cpp`, not `scene_build.cpp:85-96` (that block fills sphere tables only); (f) XeGTAO is **not present on disk**, so the crease class cites a *re-derived* substitute rather than a file.

## 1. Summary

Add a Warabi-style **screen-space** NPR edge/outline pipeline to umbreon. A Stage A G-buffer capture in the primary loop (at supersample resolution) records per-pixel linear view-z, the face-forwarded world normal, and two new `uint32` AOVs (`objectId`, `materialId`) with a background sentinel; a Stage B post-process runs a 3×3 neighborhood pass producing a per-class `[0,1]` strength for the five Warabi edge classes; a Stage C styles each class (independent color/opacity/width-via-dilation) and composites over the shaded color **in linear space** at hi-res, after which the existing `boxDownsample` yields Warabi-style resolution-dependent AA. Everything is gated behind a single `edges` on/off master flag; a default-constructed `RenderOptions` allocates no new buffers and runs no new pass, so output is **byte-identical to today**.

**How it differs from `docs/plans/edge-extraction-embree.md`.** That plan (Japanese, *計画済み・未着手*; verified present, 15 KB) keeps edge detection in **object space**: umbreon owns silhouette/crease extraction, visibility, clipping and cylinder/corner generation via Embree on an indexed `Scene` mesh (with an fp64 narrow-phase escalation), the destination for CueMol's CGAL removal. This plan is the **alternative**: no geometric edge primitives are generated — edges are a pure image-space post-process on G-buffer AOVs, gaining the Warabi disconnected-face class (class 2, which has *no* object-space equivalent), per-class screen-space styling, and zero new ray casts, at the cost of being view-dependent (no analytic edge geometry). The two share only the section/`group` plumbing and the baked-edge-removal heuristic; if both are ever enabled, screen-space `--edges` takes over silhouette generation and must disable the object-space path to avoid triple-drawn edges. This plan does **not** touch the embree doc.

**How it satisfies the user's added requirement.** Edge configuration is driven entirely from `umbreon_cli`, **per CueMol section** (`#if (_show_XX_XX)` group), mirroring per-section transparency one-for-one: a repeatable `--edge ID=spec` (the `--alpha ID=value` data path) plus a master `--edges on|off`, resolved against `geo.groupNames` exactly as `--alpha` is, and reusing `--list-groups` as the discovery UX. Because every primitive already carries its section as a `uint16_t group` *table* (`triGroupId`/`sphereGroup`/`cylGroup`), surfaced through `HitShade::group` (an `int`, non-negative; `hit_shader.hpp:29`), and the `objectId` AOV is derived from it, a pixel maps back to its section for free to pick its style. The baked POV edge primitives (`edge_line`/`edge_line2` → cylinders) are removed when `--edges` is on (globally or per-section) via a parse-time origin flag, so screen-space edges do not double-draw against the baked ones.

## 2. Verified findings about current umbreon

Each item is `file:line`, with any discrepancy vs the brief called out.

- **AOVs declared vs written.** `FrameResult` declares `color` (w·h·4 RGBA), `albedo` (w·h·3), `normal` (w·h·3 world-space), `depth` (w·h, "ray distance from camera") — `render_types.hpp:55-58`. The renderer allocates and writes **only** `color` and `depth` (`embree_renderer.cpp:80-81` assign; `:128-133` write per pixel). **`albedo` and `normal` are dead**: declared, never sized, never assigned. Confirmed.
- **`depth` is ray-tfar, not view-z.** Stored `depth = pr.depth` = `nearDepth` = `rh.ray.tfar` (`transparency.hpp:97`, `embree_renderer.cpp:133`), the Euclidean slant distance along the **normalized** primary ray. Equals view-z only under orthographic (`rd == dir`, `embree_renderer.cpp:116-118`); under perspective `rd` is per-pixel normalized (`:119-121`) so tfar overshoots by the slant factor. The brief's "depth = linear view-z" is a **target**, not the current state.
- **Primary loop / parallelism.** `EmbreeRenderer::render` parallelizes over image **rows** with `tbb::parallel_for(tbb::blocked_range<int>(0, H), …)`, serial inner column loop, disjoint framebuffer writes (`embree_renderer.cpp:105-136`). This is the construct Stage B/C reuse.
- **Per-pixel integration.** `integratePixel` inlines the front-to-back transparency walk (`transparency.hpp:73-155`); there is **no** reusable hit-collection helper — all per-hit state except composited color and `nearDepth` is discarded. First-hit depth is set at the `if (nearDepth == 0.0f) nearDepth = rh.ray.tfar;` guard (`:97`), **before** `shadeHit` is called (`:99`) and before any opaque/transparent decision (`:101`). `PixelResult` carries only `r,g,b,a,depth` (`:26-32`). No-hit break at `:93-96`.
- **`shadeHit` is the single G-buffer point.** `detail::shadeHit` resolves `geomID→rec`, `primID`, the **face-forwarded world normal** `N` (mesh interpolated slot 0, `:62`; primitive `rh.hit.Ng`, `:106-107`), material, and group, for all three primitive kinds (`hit_shader.hpp:47-139`). `N = faceForward(normalize(...), rd)` — already flipped toward viewer, in world space. The **geometric** normal `Ng` is also in scope (mesh `:70`, primitive `:113`) and is preferable for the crease class (less interpolation noise). `HitShade` carries `color, opacity, group` only (`:26-30`), where **`group` is `int`** (the gate `hs.group >= 0` at `transparency.hpp:108` relies on it).
- **Supersample / downsample.** `umbreon::render` renders at `finalW*ss × finalH*ss` (`umbreon.cpp:80-85`), applies fog at hi-res on `frame.depth` (`umbreon.cpp:89-92`), then `boxDownsample(color,…,4,ss)` (`:95`) and the **already-guarded** `if(!albedo.empty())` / `if(!normal.empty())` downsamples (`:96-101`). So writing `normal` is auto-downsampled with no extra change. **Discrepancy:** the brief/Draft A say the `.pov` default supersample is "2"; the effective value is **3** (`main.cpp:293` `povDefaultSs = 3`). Note the *in-tree* comment `cli.hpp:42` and usage string `cli.cpp:166` are **stale** (both still print "2"); do not re-introduce that mistake when editing the usage block.
- **ids today.** The only identity is the CueMol section `uint16_t group`, plumbed end-to-end: `Mesh::triGroupId`+`groupForTri()` (`scene.hpp:153,163-165`, group 0 = default), `Sphere::group` (`:180`), `Cylinder::group` (`:190`), surfaced as `HitShade::group` from `mesh.groupForTri(primID)` (`hit_shader.hpp:89`) or per-primitive group tables (`:119-121`). **No global object id, no global material id exist.** Materials split: mesh `triMaterialId` (`uint8`, ≤256) into `Mesh::materials` via `materialForTri` which returns a `Material&` **not an id** (`scene.hpp:146,158-161`); spheres/cylinders carry a `Material` **by value** (`scene.hpp:179,189`), copied to primID tables `sphereMat`/`cylMat`/`cylCapMat` (declared `scene_build.hpp:66,71,77`). The **sphere** table is filled at `scene_build.cpp:95`; the **cylinder/capped** tables are filled in `curve_build.cpp::buildCylinderGeometry` (`curve_build.hpp:23`), *not* in `scene_build.cpp`. `GeomKind` (Mesh/Sphere/Cylinder/CylinderCapped) is recorded per geomID in `records` (`scene_build.hpp:50-55`); attach order is fixed mesh→spheres→cylinders (`scene_build.cpp:128-130`).
- **CLI / transparency / pov.** `Options::sectionAlpha` (`std::map<std::string,float>`, `cli.hpp:65`) + `listGroups` (`cli.hpp:67`), with `transparentBackground`/`transparency` at `:69-71`. `--alpha ID=value` splits on `=`, strips a leading `_show` (`id.rfind("_show",0)==0 → substr(5)`), stores in `sectionAlpha[id]` (`cli.cpp:93-105`); `--list-groups` at `:106-107`; `parseBool` helper at `:9-13` (used by `--transparency` `:112-115`, `--transparent-bg` `:108-111`); unknown-dash fallthrough at `:136`. Section→group resolution: `main.cpp` builds `name→index` from `geo.groupNames`, warns on miss ("try --list-groups"), then multiplies opacity into every matching primitive and records `scene.veilGroups` (`main.cpp:160-189`); `--list-groups` reporting at `:110-129`. `veilGroups` → `isVeil[group]` once per frame (`embree_renderer.cpp:90-94`), consumed at `transparency.hpp:107-122`. `ropt.*` populated `main.cpp:225-296` before `umbreon::render` (`:323`).
- **Baked POV edges.** `mesh2_reader::parseEdgeLine(bool two)` is the **sole** producer (`mesh2_reader.cpp:633-666`); `parseSphere` (`:557`) and `parseCylinder` (`:600`) are independent and emit no edge geometry. Both `edge_line`/`edge_line2` expand to **one** `Cylinder` (never a sphere) from `(v1+raise·n1)` to `(v2+raise·n2)`, radius `w`, color from the `col` arg (`:645`, black `<0,0,0>` in real scenes — verified in `data/1ab0_scene2.inc`, e.g. `:24916`), `edge_line2` per-endpoint transmit baked into `color.w`/`opacity1` (`:654-660`), `c.group = curGroup_` (`:661`), **`c.open = true`** (`:664`), and it never calls `textureIn()` so the material stays `Material::flatOutline()` (contrast `parseCylinder` `:617-619`). Dispatch at `:115-120`. Routed to `GeomKind::Cylinder` (open), distinct from capped bonds (`open=false` → `GeomKind::CylinderCapped`).
- **Build/test.** `Taskfile.yml`: `task build` (`cmake -S . -B build; cmake --build build -j`, `:162-166`), `task test` (`+ ctest --test-dir build --output-on-failure`, `:168-173`), `task render` (`:175`). **Discrepancy: there is NO `task lint` target** (only build/test/render/deps/clean and `:static` variants), so the global "lint is `task lint`" rule does not apply — validate with `task test`. Render-dependent tests are gated on `if(TARGET umbreon)` (`tests/CMakeLists.txt`: `test_fog`, `test_image_ops`, `test_render`, in the `if(TARGET umbreon)` block ~`:29-44`). Image I/O: `writeImage(path, w, h, const float* pixels, channels)` channels 3 or 4 (`image_io.cpp:148-151`), `compareImages` PSNR/SSIM (`:223`).

## 3. Data-structure changes

### 3.1 `FrameResult` new / repurposed AOVs (`render_types.hpp:52-61`)

All sized **only when `edges.enable`** (default path stays byte-identical; `boxDownsample` already guards `albedo`/`normal` at `umbreon.cpp:96-101`). Keep `depth` as ray-tfar (fog consumes it, `umbreon.cpp:89-92`) and add a **separate** edge-only `viewZ` AOV so fog stays byte-identical.

| AOV | Type / layout | Meaning |
|---|---|---|
| `normal` | `vector<float>` w·h·3, world-space | The face-forwarded world normal **already computed but discarded** (`hit_shader.hpp:62`/`106-107`). For the crease class, prefer the **geometric** `Ng`-derived face-forwarded normal (`:70`/`:113`; less interpolation noise). |
| `viewZ` (new) | `vector<float>` w·h | **Linear view-z** = `tfar · dot(rd, dir)` (identity under ortho; slant-corrected under perspective). Edge-only; leaves `depth` untouched. |
| `objectId` (new) | `vector<uint32_t>` w·h | Per-pixel representative-fragment object identity. |
| `materialId` (new) | `vector<uint32_t>` w·h | Per-pixel global material identity. |

**Background sentinel.** The sentinel is an **initial value**, not a write performed at the no-hit break. Initialize `PixelResult.objectId = 0xFFFFFFFFu` and `viewZ/depth = 0` at construction; a real first hit overwrites them at `transparency.hpp:97`. The no-hit break (`:93-96`) simply exits, leaving the initialized sentinel in place. Silhouette keys on `objectId == INVALID` to avoid the `viewZ==0` collision with a genuine zero-distance hit.

`PixelResult` (`transparency.hpp:26-32`) gains `uint32_t objectId = 0xFFFFFFFFu; uint32_t materialId = 0xFFFFFFFFu; Vec3 worldNormal; float viewZ = 0.0f;`, captured from the **first** `shadeHit`, written at the store site (`embree_renderer.cpp:128-133`).

### 3.2 `RenderOptions` / `EdgeOptions` + per-section `EdgeStyle` (`render_types.hpp:14-49`)

`RenderOptions` has no edge fields today; add `EdgeOptions edges;` defaulted **OFF**. Reconciling the two drafts: `EdgeClassStyle` is per-class style; `EdgeStyle` is the per-section bundle of five class styles; `EdgeOptions` is the global block holding detection scalars, a `defaultStyle`, and the master gate.

```cpp
enum class EdgeClass : uint8_t {       // the five Warabi classes
  Silhouette=0, Disconnected=1, Object=2, Material=3, Crease=4, Count=5 };

struct EdgeClassStyle {                // independent per class
  bool  enabled = false;
  Vec3  color   {0,0,0};               // linear RGB, composited in linear space
  float opacity = 1.0f;                // 0..1
  float width   = 1.0f;                // mask dilation radius, hi-res (supersample) px
};

struct EdgeStyle {                      // per CueMol section (per group)
  EdgeClassStyle cls[static_cast<int>(EdgeClass::Count)];
};

struct EdgeOptions {
  bool  enable        = false;          // MASTER gate; false => zero new work, byte-identical
  // detection thresholds (Mol* analogues)
  float distanceThreshold = 1.0f;       // depth-gap, LINEAR VIEW-Z world units, pixelSize-scaled
  float curvatureGate     = 0.75f;      // Mol* curvature-veto constant (re-tune; see risks)
  float creaseAngleDeg    = 30.0f;      // crease: fire when dot(n) < cos(creaseAngleDeg)
  int   neighborhood      = 4;          // 4 (±x,±y) default; 8 (+diagonals) thicker/closed
  // suppression tables: group ids 1..32, bit i set => co-group => boundary suppressed
  std::array<uint32_t,33> objectSuppress{};
  std::array<uint32_t,33> materialSuppress{};
  // styling: section without an override uses defaultStyle
  EdgeStyle defaultStyle;
  // later-phase calligraphic pen (parsed/stored now, ignored until phase 2)
  float penHardness=1.0f, penRoundness=1.0f, penSlant=0.0f;
};
```

A render-side per-group table resolves sections to styles: `std::vector<EdgeStyle> Scene::groupEdgeStyle`, indexed by group, sized to `groupNames.size()`, every slot initialized to `EdgeOptions::defaultStyle`. Populated in the `--edge` resolution loop (§5). Stage C looks up `groupEdgeStyle[objectId >> 2]`.

### 3.3 Global object-id and material-id spaces

**`objectId` (uint32).** The section group alone is too coarse (group 0 swallows all unsectioned geometry; meshes/spheres/cylinders share a group). Synthesize at the hit:

```
objectId = (static_cast<uint32_t>(group) << 2) | kindBits   // kindBits from GeomKind {Mesh,Sphere,Cyl,CylCapped}
```

`group` reaches `shadeHit` as a non-negative `int` (the primitive tables are `uint16_t`, so the value range is `[0, 65535]`); the explicit `uint32_t` cast before the shift is required. `GeomKind` is free from `records[geomID].kind`. The low 2 bits separate a mesh surface from a sphere/bond in the same section; Stage B/C recover `group = objectId >> 2` for suppression and per-section styling. Background `0xFFFFFFFFu` cannot collide with any real `(group<<2)|kind` because the largest real value is `(65535<<2)|3 = 0x0003FFFF`. (For *styling lookup only*, the relevant key is always `group`; the kind bits matter only to the object-boundary **class**.)

**`materialId` (uint32, global).** Merge the two split spaces at scene-build by offsetting each primitive table above the mesh material count:

```
materialId(mesh tri)  = triMaterialId[primID]
materialId(sphere)    = meshMatCount + sphereMatIndex[primID]
materialId(cyl)       = meshMatCount + sphereMatCount + cylMatIndex[primID]
materialId(cylCapped) = meshMatCount + sphereMatCount + cylMatCount + cylCapMatIndex[primID]
```

Build per-primitive `*MatIndex` side-tables during scene build. The **sphere** index table is built next to the existing `sphereMat` fill (`scene_build.cpp:84-99`); the **cyl/cylCapped** index tables must be built inside `curve_build.cpp::buildCylinderGeometry` (`curve_build.hpp:23`), *alongside* `cylMat`/`cylCapMat` — `scene_build.cpp:85-96` fills sphere tables only, so the cylinder dedup work is a `curve_build.cpp` edit. **Dedup needs an explicit comparator:** `umbreon::Material` (`scene.hpp:102-130`) has no `operator==`/hash, so deduplicating "identical `Material` values per kind" requires either (a) a field-wise equality over its 10 floats + `bool metallic`, or (b) skipping dedup entirely and assigning a raw per-primitive index (simpler; only inflates the id space, which is irrelevant to the `id != ref` boundary test). Option (b) is recommended for phase 1. `shadeHit` reads the precomputed index by `primID` — no per-hit `Material` comparison. Keep all offset arithmetic 32-bit; `triMaterialId` is `uint8` so the mesh block never exceeds 256.

## 4. Stage A / B / C designs

### Stage A — G-buffer capture (in the primary loop, hi-res)

The representative fragment is the **FIRST (nearest)** hit of the transparency walk, captured **before** any opaque/transparent compositing decision — exactly where `nearDepth` is set today (`transparency.hpp:97`), so edges draw against the frontmost visible surface (even a transparent veil). No hit-collection helper exists; capture must happen at the first `shadeHit`. Two coordinated edits:

1. **`shadeHit` (`hit_shader.hpp`)** — extend `HitShade` (`:26-30`) with `Vec3 normal; uint32_t objectId; uint32_t materialId;`. Populate in both branches: `hs.normal = N` after `:62` (mesh) / `:106-107` (primitive — prefer geometric `Ng` from `:70`/`:113` for crease); `hs.objectId = (static_cast<uint32_t>(hs.group) << 2) | kindBits` (kindBits from `rec.kind`); `hs.materialId` from the precomputed per-primitive index (mesh near `:64`; primitives near `:103-105`).
2. **`integratePixel` (`transparency.hpp`)** — at the first-hit guard (`:97`), after `shadeHit` (`:99`) so `hs` is in scope but still gated on "first hit" (test `nearDepth==0` before its assignment):

```cpp
if (nearDepth == 0.0f) {
  nearDepth = rh.ray.tfar;
  firstNormal     = hs.normal;        // face-forwarded toward viewer
  firstObjectId   = hs.objectId;
  firstMaterialId = hs.materialId;
  firstViewZ      = rh.ray.tfar * dot(rd, dir);   // linear view-z (Mol* getViewZ analogue)
}
```

`firstObjectId`/`firstMaterialId`/`firstViewZ` are **initialized** to the sentinel/zero (§3.1); on the no-hit break (`:93-96`) they keep those initials, which IS the sentinel — nothing is written at the break. `dir` = normalized `cam.direction` (forward axis), available in `EmbreeRenderer::render` (`embree_renderer.cpp:50`) and passed into the integrator (extend its signature or the `ShadeContext`). Then write the new AOVs at the pixel store (`embree_renderer.cpp:128-133`) and **size them at `render()` top (`embree_renderer.cpp:77-82`) only when `opt.edges.enable`** — this is the byte-identical allocation gate (it is NOT `umbreon.cpp:79-82`, which is merely the hi-res-dimension setup).

*Why view-z, not ray-tfar:* the perspective path is live (`embree_renderer.cpp:119-121`; struct default `fovy=40`, `scene.hpp:213`). Under perspective, tfar conflates "surface receded" with "surface off-axis", and Mol*'s `getViewZ`/`getPixelSize`/curvature math (`outlines.frag.ts:27,55`) assumes planar view-z. The `dot(rd,dir)` conversion is identity under ortho and ports the Mol* formulas verbatim. Keeping a separate `viewZ` AOV (not mutating `depth`) keeps fog byte-identical.

### Stage B — edge detection (post-process, hi-res, between fog and downsample)

**Placement.** In `umbreon::render()` after fog (`umbreon.cpp:92`) and **before** `boxDownsample` (`:94`), on the hi-res `FrameResult` (`finalW*ss`), reusing the renderer's row `tbb::parallel_for` construct for the 3×3 pass. Default 4-neighbor (±x,±y); 8-neighbor option for thicker/closed boundaries.

**Per-pixel precompute.** `selfViewZ` = center `viewZ` (or `backgroundViewZ = 2·far` if `objectId==INVALID`). `pixelSize` = world units per pixel at this depth (Mol* `getPixelSize`, `outlines.frag.ts:55-57`, which differences two inverse-projected screen neighbors): C++ analogue `pixelSize ≈ (2·halfH / heightPx)·|viewZ| / focalDist` from `cam.height`/`fovy` half-extents (`embree_renderer.cpp:54-58`), or exact inverse-projection reconstruction. Scales the gap threshold so sensitivity is perspective-robust.

The five classes (compute per-class `strength ∈ [0,1]`; `gap = |selfViewZ − sampleViewZ|`; `threshold = pixelSize · distanceThreshold`):

1. **Silhouette** *(Mol* `outlines.frag.ts` `isBackground` sentinel, `:51`)* — center surface (`objectId != INVALID`), any neighbor background (`sampleObjectId == INVALID`). Falls out of the gap test against `2·far`. `strength = clamp((gap − threshold)/threshold, 0, 1)`.
2. **Disconnected-face (signature Warabi line)** *(Mol* gap + lazy curvature veto, `outlines.frag.ts:112-126`; no object-space/Blender analogue)* — `sampleObjectId == selfObjectId` AND `gap > threshold`. Then **curvature veto** (mirrors Mol* `:117-126`): `ddx=|vzL+vzR−2·selfViewZ|`, `ddy=|vzU+vzD−2·selfViewZ|`, `curv=max(ddx,ddy)`; if `curv < pixelSize·curvatureGate`, cancel. Smooth sphere/SES (near-linear depth) survives; gentle curvature does not crease. `strength = clamp((gap−threshold)/threshold,0,1)` post-veto.
3. **Object boundary** *(Blender `id != ref` gather4 in `overlay_outline_detect_frag.glsl:41-45,168`, re-derived in fresh MIT C++, + Mol* gap)* — `sampleObjectId != selfObjectId` AND `gap > threshold` AND **not** `coGrouped(selfGroup, sampleGroup)` where `selfGroup = objectId>>2` and `coGrouped` consults `objectSuppress[selfGroup] & (1u<<sampleGroup)`. Line on the surface side by default.
4. **Material boundary** *(same re-derived `id != ref`, keyed on `materialId`; no Mol* analogue)* — `sampleMaterialId != selfMaterialId`, optionally gated on a small gap (avoid co-planar seams), suppressed via `materialSuppress`. `strength = 1` on id change (or gap-scaled if gated).
5. **Crease / ridge-valley** *(authored substitute — XeGTAO is **not present on disk**, so this class has no cited reference file; the grazing-angle bias is re-derived from the XeGTAO view-space-normal idea)* — `dot(n_center, n_neighbor) < cos(creaseAngleDeg)` on the geometric face-forwarded world-normal AOV. To suppress grazing-angle false positives: (a) scale `creaseAngleDeg` up at grazing angles via `dot(n_center, V)`; (b) reuse the curvature-veto idea on smoothly curved regions. Gate against the gap so it does not double-fire on a silhouette/boundary. `strength = clamp((cos(creaseAngleDeg) − dot(n)) / (1 + cos(creaseAngleDeg)), 0, 1)` — numerator goes 0⁺ just past threshold and reaches 1 at a full 180° fold (`dot(n)=−1`), since `cos(θ)−(−1)=1+cos(θ)` is exactly the available span below threshold; bounded and monotonic.

**Precedence (most-specific structural edge wins).** `Silhouette > ObjectBoundary > MaterialBoundary > DisconnectedFace > Crease`. Because width (dilation) differs per class and must be applied *before* the over-composite, **accumulate one mask per class** and let Stage C dilate then composite in precedence order.

### Stage C — styling & composite (hi-res, then downsample)

Operates between fog (`umbreon.cpp:92`) and `boxDownsample` (`:94`), compositing over `frame.color` **in linear space**.

- **Per-section/per-class style.** Look up `Scene::groupEdgeStyle[objectId >> 2]` (defensive: bounds-check; disabled class contributes nothing); apply its per-class `{color, opacity, width}`.
- **Width via dilation** *(Mol* `getOutline` disk, `postprocessing.frag.ts:61,75-80`; pass scale `outline.ts:71`)* — per class mask, dilate over `x·x + y·y <= width²`, taking the nearest masked pixel. Runs at hi-res; the box-average yields the AA.
- **Composite.** In precedence order, `over`-composite each dilated mask: `color = mix(color, classColor, mask · classOpacity)`. Optional Mol* fog modulation of line color (`postprocessing.frag.ts:141,157` `mix(..., uFogColor, smoothstep(uFogNear, uFogFar, viewDist))`) so lines fog with the scene. Then existing `boxDownsample(color,…,4,ss)` (`umbreon.cpp:95`) produces the final antialiased image.
- **Calligraphic pen (later phase).** `{hardness, roundness, slant}` replaces the circular disk with a rotated ellipse (slant=rotation, roundness=aspect, hardness=falloff). Fields parsed/stored now, **ignored until phase 2**; phase 1 uses the circular disk only.

## 5. Per-object (per-section) CLI edge configuration + baked-pov-edge removal

### 5.1 New CLI options (same syntax style as `--alpha`)

Parse in the same `else-if` chain as `--alpha`, before the unknown-dash fallthrough (`cli.cpp:136`); use `parseBool` for the on/off flag (as `--transparency`, `cli.cpp:112-115`):

```
--edges <on|off>      master screen-space edge switch                       [off]
--edge  <ID=spec>     per-section edge style override (repeatable)
--list-groups         (REUSED) list section ids to target with --edge
```

`--edge` mirrors `--alpha ID=value` (`cli.cpp:93-105`): split on the first `=`, strip a leading `_show`, store under the stripped id in `Options::sectionEdge` (`std::map<std::string, EdgeStyle>`). The spec selects enabled **classes** and per-class style; class tokens `sil`/`disc`/`obj`/`mat`/`crease` map to the five classes; `all`/`none` are shorthands:

```
--edge _34_35=sil,crease:color=#000000:width=1.5:opacity=1.0   # silhouette+crease, black 1.5px
--edge _17_19=disc:color=#404040:width=1.0:opacity=0.6          # Warabi lines only, thin grey
--edge _34_35=obj:width=2.0,crease:width=1.0:color=#000000      # per-class width
--edge _99_99=none                                              # disable for one section
```

Unlisted sections fall back to `EdgeOptions::defaultStyle`. Group 0 (geometry outside any `#if (_show_*)`, listed as `(default)`, `groupNames_[0] = ""` per `mesh2_reader.cpp:930`) is not addressable by a section-id string — its style comes solely from `defaultStyle`, matching `--alpha` semantics. Add usage lines next to the existing `--alpha`/`--list-groups` lines at `cli.cpp:169-170` (and, while there, fix the stale `.pov: 2` supersample string at `:166` → `3`).

### 5.2 Wiring (parallel to `sectionAlpha`, no `veilGroups` mutation)

- **CLI** (`cli.hpp:62-71`): add `bool edges = false;`, `std::map<std::string,EdgeStyle> sectionEdge;`, and global scalars (`edgeDistanceThreshold`, `edgeCurvatureGate`, `edgeCreaseAngleDeg`, …).
- **Resolution** (`main.cpp:160-189`, clone the `--alpha` loop): build `gidx` from `geo.groupNames`, warn-on-miss, write each resolved `EdgeStyle` into `Scene::groupEdgeStyle[g]` (sized to `groupNames.size()`, slots pre-initialized to `defaultStyle`). Do **not** touch `veilGroups`. **This loop runs on the already-moved `scene` (the move is at `main.cpp:131-133`), so it has both the resolved per-section styles and `scene.cylinders` in hand — which is exactly what the per-section POV-edge filter (§5.3) needs.**
- **RenderOptions** (`render_types.hpp:14-49`): add `EdgeOptions edges;` (default OFF). Populate `ropt.edges` from `opt` next to the other `ropt.*` assignments (`main.cpp:225-296`), before `umbreon::render` (`:323`).

A pixel maps back to its section via `group = objectId >> 2`; Stage B/C index `groupEdgeStyle[group]`.

### 5.3 Removing baked-in POV edge primitives

**Preferred signature: a parse-time origin flag** (100%-precise, survives `--outline-scale` at `main.cpp:134-137` and material edits). Add `bool fromEdgeMacro = false;` to `Cylinder` (`scene.hpp:185-203`) and set it `true` at `mesh2_reader.cpp:664` next to `c.open = true`. `parseEdgeLine` is the single choke point (`:633-666`); `parseCylinder` (`:600`) never sets it, so user `cylinder{...open...}` bonds keep `fromEdgeMacro=false`.

Heuristic ranking (most→least robust): (1) `fromEdgeMacro == true` — recommended; (2) `open && material == flatOutline && color ≈ black` — fallback if adding a field is undesirable, risks clipping a user's open black bond (note: real edge cylinders verified black `<0,0,0>` in `data/1ab0_scene2.inc:24916+`); (3) per-section gating — combine (1) with "only drop edges whose `group` has the silhouette/disconnected class enabled"; (4) color-only — rejected.

**Removal hook — resolved location (corrected).** The first draft contradicted itself ("before geometry is moved into the scene (`main.cpp:131-137`)" vs "parallel to the `--alpha` loop"). In fact `main.cpp:131-133` *is* the `std::move` of `geo.cylinders` into `scene.cylinders`, and the `--alpha` resolution it parallels is at `:160-189`, operating on `scene.cylinders` *after* the move. Pick **one** consistent site:

- **Global filter** (`fromEdgeMacro == true`, no per-section state needed): filter `geo.cylinders` **before** the move, i.e. drop matching entries between `main.cpp:133` and the move — or equivalently filter `scene.cylinders` immediately after `:133`. Simplest.
- **Per-section filter (preferred):** must run **after** the cloned `--edge` resolution loop (after `main.cpp:189`), because it needs the resolved `groupEdgeStyle[group]`/`sectionEdge` to decide. Drop a baked edge from `scene.cylinders` only when its `group` resolves to a section that has the silhouette/disconnected class enabled, so an un-`--edge`'d section keeps its baked POV edges (graceful degradation).

Do **not** filter inside `buildCylinderGeometry` (`scene_build.cpp:128-130` / `curve_build.cpp`) — that is further from policy and the stitching pass runs there. When `opt.edges` is on, apply the chosen filter; with `--edges off` (default) nothing is filtered (byte-identical). Capped bonds (`open == false`, `GeomKind::CylinderCapped`) are **never** removed. A future `--keep-baked-edges on` escape hatch can suppress the filter even with `--edges` on, for A/B comparison.

## 6. Parameter list and defaults

All behind `EdgeOptions::enable` (master gate). Default `EdgeOptions{}` → `enable == false` → no AOV allocation, Stage B/C skipped → **byte-identical output**.

| Parameter | Default | Scope | Notes |
|---|---|---|---|
| `enable` | `false` | global master | the byte-identical gate |
| `distanceThreshold` | `1.0` | global | depth-gap, linear view-z world units, `pixelSize`-scaled |
| `curvatureGate` | `0.75` | global | Mol* veto constant; **expect re-tuning** (umbreon feeds view-z directly, not `[0,1]` clip depth) |
| `creaseAngleDeg` | `30.0` | global | crease dot threshold |
| `neighborhood` | `4` | global | 4 (±x,±y) or 8 (+diagonals) |
| `objectSuppress` / `materialSuppress` | all-zero (no suppression) | global | per group-id co-group bitmasks |
| per-class `enabled` | `false` | per section (and `defaultStyle`) | all five classes off by default |
| per-class `color` | `{0,0,0}` linear | per section | composited in linear space |
| per-class `opacity` | `1.0` | per section | |
| per-class `width` | `1.0` | per section | dilation radius in **hi-res** (supersample) px |
| `penHardness/Roundness/Slant` | `1/1/0` | global | parsed/stored, **ignored until phase 2** |

## 7. Resolved design questions

- **Depth metric:** convert ray-tfar → linear view-z at capture (`tfar · dot(rd, dir)`) so the gap/curvature tests are correct under perspective and Mol* formulas port verbatim; store as a **separate** `viewZ` AOV so fog (which reads `depth`) stays byte-identical.
- **Perspective robustness:** scale the gap threshold by per-pixel `pixelSize` (Mol* `getPixelSize`, `outlines.frag.ts:55`) so sensitivity is depth-independent; without it distant geometry over-detects and near under-detects.
- **Smooth-surface false positives:** (a) Mol* curvature veto (gate `0.75`, re-tune; `outlines.frag.ts:112-126`) cancels disconnected-face/crease where the view-z 2nd derivative is sub-threshold; (b) crease uses the **geometric** face normal + view-incidence bias (XeGTAO absent on disk — standard re-derived substitute, not a cited file).
- **Transparency:** the G-buffer fragment is unconditionally the **first (nearest)** hit (`transparency.hpp:97`), captured before any opaque/transparent decision; edges draw against the frontmost visible surface, even a transparent veil. Mol*'s separate opaque/transparent tracks (`outlines.frag.ts:68-99` + the near-opaque suppression at `:108`) are a possible later refinement. Policy for a section both veiled (`--alpha`) *and* edge-styled: edges follow the front surface; optionally suppress edges on veiled sections (explicit toggle, later).
- **Object-space plan interaction:** screen-space `--edges` takes over silhouette generation and disables the object-space path; the two share section/group plumbing and the baked-edge heuristic only; never enable both edge generators at once.
- **Baked-edge removal precision:** parse-time `fromEdgeMacro` flag (exact) over color/geometry heuristics; removal in the CLI policy layer (global before/at the move, or per-section after the resolution loop, §5.3), automatic with `--edges`, reversible by default-off.

## 8. Incremental implementation order

Each step is independently buildable and visually verifiable. The byte-identical-default guarantee holds throughout: every new AOV is allocated only when `edges.enable` (gate in `EmbreeRenderer::render`, `embree_renderer.cpp:77-82` and the writes at `:128-133`), Stage B/C are skipped when off, and `boxDownsample` already skips empty `albedo`/`normal` (`umbreon.cpp:96-101`).

1. **AOV capture + debug dumps (no edges yet).** Extend `PixelResult`/`HitShade` with `worldNormal`, `objectId`, `materialId`, `viewZ`; capture on the first hit (`transparency.hpp:97`); convert tfar→view-z; write AOVs at `embree_renderer.cpp:128-133`; allocate them at `:77-82` under `if(opt.edges.enable)`. **Eyeball:** pack each AOV into a 3/4-channel float buffer and `writeImage` (`image_io.cpp:148`) — false-color ids (cross-check `--list-groups`), `normal*0.5+0.5` (smooth on spheres, toward camera), normalized `viewZ` (monotonic front-to-back), background = sentinel. **Diff:** color output byte-identical to pre-change at `--edges off`.
2. **Silhouette only.** Insert Stage B/C between post-fog (`umbreon.cpp:92`) and `boxDownsample` (`:94`), reusing the row `parallel_for`. Implement class 1 via the background sentinel; composite a fixed-color line in linear space. **Eyeball:** outlines around the molecule; smooth AA after downsample. **Diff vs step 1:** changes only at object/background boundaries.
3. **Crease.** Add class 5 on the world-normal AOV, gated against the gap. **Eyeball:** ridge/valley lines on faceted regions; toggle `creaseAngleDeg`. **Critical:** smooth spheres/SES must **not** crease (else pull the curvature veto in earlier).
4. **Depth-gap classes (disconnected-face + object boundary) + pixelSize + curvature veto.** Add `getPixelSize`, `gap > pixelSize·distanceThreshold`, and the curvature veto (gate `0.75`) — the single most important port for not creasing smooth surfaces. Class 2 = same `objectId` + gap; class 3 = different `objectId` + gap. **Eyeball:** Warabi lines inside same-object discontinuities; spheres stay clean; sweep `distanceThreshold` for stability across a dolly/zoom (not one frame).
5. **Material/object boundary + group suppression.** Add class 4 on `materialId` (re-derived `id != ref`, never copying Blender GLSL); add `objectSuppress`/`materialSuppress` co-group checks. **Eyeball:** lines between differently-materialed regions; suppression removes internal seams.
6. **Per-section CLI styling + width/dilation.** Wire `Options::sectionEdge` → `Scene::groupEdgeStyle` (clone of `main.cpp:160-189`); Stage B/C look up per-pixel style by `group`; add per-class disk dilation (Mol* `getOutline`). **Eyeball:** `--edge _34_35=...` changes only that section; per-class color/opacity/width independent; compare two differently-styled sections.
7. **POV edge-primitive removal wired to `--edges`.** Add `Cylinder::fromEdgeMacro` (set at `mesh2_reader.cpp:664`); filter per §5.3 (global before/at the move `main.cpp:133`, or per-section after the resolution loop `:189`) when `opt.edges` on. **Eyeball:** with `--edges on`, baked silhouettes vanish, only screen-space edges remain (no doubled/fuzzy outline); capped bonds untouched; `--edges off` restores baked edges byte-identically.
8. **Calligraphic pen (later).** Hardness/roundness/slant on the dilation kernel; independent, does not affect the default guarantee.

## 9. Open risks

- **Curvature-veto tuning on SES.** `0.75` is Mol*'s GPU constant against *their* `[0,1]` depth packing (`outlines.frag.ts`); umbreon feeds **view-z directly**, so absolute `curv` magnitudes differ and the gate likely needs re-tuning per scene scale. Too low → spheres crease; too high → genuine gaps vetoed. Crease (step 3) is most exposed; expose `curvatureGate`/`creaseAngleDeg` as `EdgeOptions` scalars.
- **Perspective threshold stability.** The perspective path is live (`embree_renderer.cpp:119-121`; struct default `fovy=40`, `scene.hpp:213` — though real CueMol scenes are frequently orthographic, e.g. `data/1ab0_scene2.pov` sets `_perspective=0`). `pixelSize` and the `tfar·dot(rd,dir)` conversion must be correct under both projections or lines flicker/thicken with distance. Validate `distanceThreshold` across a dolly/zoom on a perspective scene specifically.
- **id-space overflow / sentinel collision.** `objectId = (group<<2)|kindBits` fits `uint32` (max real `0x0003FFFF`, vs sentinel `0xFFFFFFFF`); `group` arrives as a non-negative `int`, cast explicitly before the shift. `materialId` merges `uint8` mesh ids with offset primitive tables — keep all offsets 32-bit.
- **materialId dedup comparator.** `Material` (`scene.hpp:102-130`) has no `operator==`; per-kind dedup needs a field-wise comparator over its 10 floats + `bool`. Phase 1 may skip dedup and use a raw per-primitive index (only inflates the id space, harmless to the `id != ref` test). The cyl/cylCap index tables are a `curve_build.cpp` edit (not `scene_build.cpp`).
- **Supersample memory / cost of 4 extra AOVs.** Stage A runs at `width*ss × height*ss` (`.pov` default ss = **3**, `main.cpp:293`); four extra AOVs (`normal` 3×f32, `objectId`/`materialId` 1×u32 each, `viewZ` 1×f32) roughly 3–4× the G-buffer footprint at hi-res, plus the 3×3 re-read. Gate **allocation** behind `edges.enable`; consider packing `objectId`/`materialId` if memory-bound.
- **Transparency edge cases.** Edges follow the first hit regardless of opacity, so a transparent veil in front of opaque geometry may draw an edge where Mol* (separate opaque/transparent tracks with the near-opaque suppression rule, `outlines.frag.ts:108`) would not. Interaction of a section both `--alpha`-veiled and edge-styled needs the explicit policy in §7.
- **Object-space plan conflict.** Must not co-enable with `docs/plans/edge-extraction-embree.md` (verified present); define precedence (screen-space takes over) to avoid triple-drawn edges (object-space + screen-space + baked POV).

## 10. Validation

- **Harness.** CMake/ctest + go-task: `task build` (`Taskfile.yml:162-166`), `task test` (`:168-173`). **No `task lint` exists** — validate with `task test`. New AOV/edge tests under `tests/`, gated `if(TARGET umbreon)` like `test_render`/`test_fog` (`tests/CMakeLists.txt`, the `if(TARGET umbreon)` block ~`:29-44`).
- **End-to-end.** Drive from `umbreon_cli`. Render a real CueMol scene — `data/1ab0_scene2.pov` (`#declare _scene = #include "1ab0_scene2.inc"`, `:199`; `_show_34_35` defaults to 1, `:177-178`), whose `.inc` has live `edge_line(...)` inside `#if (_show_34_35)` (`data/1ab0_scene2.inc:61` opens the section, edges at `:24916+`, all black `<0,0,0>`) — at `--edges off`, then `--edges on`, then per-section `--edge _34_35=...`. Use `--list-groups` to confirm targetable sections (reporting at `main.cpp:110-129`).
- **Debug-image dumps.** At each AOV/class step, pack into a 3/4-channel float buffer and call `writeImage` (`image_io.cpp:148`, PNG/PPM by extension) — false-color ids, `normal*0.5+0.5`, normalized view-z, per-class masks. No new writer needed.
- **Byte-identical default regression.** Save a golden PPM from the pre-change binary; render the same scene/args with the new binary at `--edges off`; assert equality with `compareImages` (PSNR=∞, `image_io.cpp:223`). Wire as a ctest so every step re-verifies the default-off path stays POV-faithful.
- **Class-isolation diffs.** Enable one class at a time (via `--edge` class tokens) and diff successive images so each class's contribution is independently visible and regression-testable.

### Critical Files for Implementation
- /home/ishitani/proj64/umbreon/src/umbreon/render/render_types.hpp
- /home/ishitani/proj64/umbreon/src/umbreon/render/transparency.hpp
- /home/ishitani/proj64/umbreon/src/umbreon/render/hit_shader.hpp
- /home/ishitani/proj64/umbreon/src/umbreon/umbreon.cpp
- /home/ishitani/proj64/umbreon/src/umbreon/render/embree_renderer.cpp
- /home/ishitani/proj64/umbreon/src/bench/main.cpp
- /home/ishitani/proj64/umbreon/src/bench/cli.cpp
- /home/ishitani/proj64/umbreon/src/bench/geom/mesh2_reader.cpp
- /home/ishitani/proj64/umbreon/src/umbreon/render/scene_build.cpp
- /home/ishitani/proj64/umbreon/src/umbreon/render/curve_build.cpp