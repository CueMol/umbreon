# Plans

設計合意ドキュメント置き場（実装状況は各項目に付記）。

- [ao-soft-shadow.md](ao-soft-shadow.md) — Ambient Occlusion + ハード影 + ソフト影（エリアライト）の実装プラン。
  OSPRay `scivis`/`ao` を C++17 へ移植、`rtcOccluded1` ベース、single-bounce。既定 OFF で現行出力と bit-exact。
  **実装完了**（branch `ao_impl_0620`、AO=`8b8de83` / ハード影=`5b879f5` / ソフト影=`c9d0f2f`）。
- [edge-extraction-embree.md](edge-extraction-embree.md) — silhouette/crease エッジ抽出を CueMol(CGAL) から
  umbreon(Embree) へ移管する実装プラン。production は直接 `umbreon::Scene` 渡し、umbreon が検出＋可視性＋
  クリッピングを所有。fp32/fp64 精度エスカレーションと PoC ファースト方針を含む。
- [mesh-indexed-refactor.md](mesh-indexed-refactor.md) — `Mesh` を三角形スープ → indexed 化する refactoring。
  load 時に 1 回溶接して属性分割つき indexed mesh を作り Embree 直結（ribbon1 で頂点 157200→約27000）。
  描画用(位置+法線+色)とトポロジー用(位置のみ)の二系統併合が要点。**提案・未着手**。cusp バグとは無関係。
- [fog-opengl-linear.md](fog-opengl-linear.md) — fog を POV 指数近似 → CueMol の OpenGL 線形 fog
  （`fog_inc.glsl`）相当へ置換し、画面表示と一致させる。POV `distance` と `_distance` から `fogStart/fogEnd`
  を復元。透過背景時は fog 色を焼き込まず alpha フェード（後段の背景差し替えが破綻しない）。**提案・未着手**。
