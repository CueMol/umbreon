# Plans

設計合意ドキュメント置き場（実装状況は各項目に付記）。

- [ao-soft-shadow.md](ao-soft-shadow.md) — Ambient Occlusion + ハード影 + ソフト影（エリアライト）の実装プラン。
  OSPRay `scivis`/`ao` を C++17 へ移植、`rtcOccluded1` ベース、single-bounce。既定 OFF で現行出力と bit-exact。
  **実装完了**（branch `ao_impl_0620`、AO=`8b8de83` / ハード影=`5b879f5` / ソフト影=`c9d0f2f`）。
- [ao-quality.md](ao-quality.md) — AO 立体感向上の実装プラン。距離減衰+マルチスケール ambient obscurance、
  bent normal+半球グラデーション ambient、albedo-aware multibounce 補正、低食い違いサンプリング、任意 diffuse 減衰
  レバー。単一レイセット共有でレイ増ゼロ、二重ゲートで既定 OFF/bit-exact。将来の surface irradiance cache + OIDN
  へ再利用できる first-hit/AOV・contact/shape 分離・bent normal infra も整備。**提案・未着手**。
- [surface-irradiance-cache.md](surface-irradiance-cache.md) — Adaptive Surface Irradiance Cache(one-bounce
  diffuse GI)+ ray-traced contact/shape AO + denoiser(自前 edge-aware à-trous 既定 / OIDN を optional backend）の
  実装プラン。Ward–Heckbert/Křivánek の irradiance caching を「決定的 placement → 並列 fill → read-only 補間」の
  3分割でスレッド数非依存・bit-exact 化(lazy 挿入を排除)。ao-quality.md を土台に積む。master gate `--gi` 既定 OFF。
  OIDN は CMake option で optional 依存。**提案・未着手**。
- [edge-extraction-embree.md](edge-extraction-embree.md) — silhouette/crease エッジ抽出を CueMol(CGAL) から
  umbreon(Embree) へ移管する実装プラン。production は直接 `umbreon::Scene` 渡し、umbreon が検出＋可視性＋
  クリッピングを所有。fp32/fp64 精度エスカレーションと PoC ファースト方針を含む。
- [edge-analytic-silhouette.md](edge-analytic-silhouette.md) — `--edges`（stroke パス）で sphere/cylinder の
  解析 n·v シルエットも輪郭化し、ball-and-stick を mesh と同じ chain/QI/ribbon で描く。共有コア
  `render/analytic_silhouette` へ obj-edges の emitter を移設（byte-identical）、リング circumscription で
  自己遮蔽=破線を回避。解析シルエットは常に遮蔽境界として使い `--stroke-analytic` は描画のみゲート（off でも
  ribbon 隠線は正しい）。**実装完了**。
- [mesh-indexed-refactor.md](mesh-indexed-refactor.md) — `Mesh` を三角形スープ → indexed 化する refactoring。
  load 時に 1 回溶接して属性分割つき indexed mesh を作り Embree 直結（ribbon1 で頂点 157200→約27000）。
  描画用(位置+法線+色)とトポロジー用(位置のみ)の二系統併合が要点。**提案・未着手**。cusp バグとは無関係。
- [fog-opengl-linear.md](fog-opengl-linear.md) — fog を POV 指数近似 → CueMol の OpenGL 線形 fog
  （`fog_inc.glsl`）相当へ置換し、画面表示と一致させる。POV `distance` と `_distance` から `fogStart/fogEnd`
  を復元。透過背景時は fog 色を焼き込まず alpha フェード（後段の背景差し替えが破綻しない）。**提案・未着手**。
