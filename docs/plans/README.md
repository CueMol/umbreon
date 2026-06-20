# Plans

設計合意ドキュメント置き場（実装状況は各項目に付記）。

- [ao-soft-shadow.md](ao-soft-shadow.md) — Ambient Occlusion + ハード影 + ソフト影（エリアライト）の実装プラン。
  OSPRay `scivis`/`ao` を C++17 へ移植、`rtcOccluded1` ベース、single-bounce。既定 OFF で現行出力と bit-exact。
  **実装完了**（branch `ao_impl_0620`、AO=`8b8de83` / ハード影=`5b879f5` / ソフト影=`c9d0f2f`）。
- [edge-extraction-embree.md](edge-extraction-embree.md) — silhouette/crease エッジ抽出を CueMol(CGAL) から
  umbreon(Embree) へ移管する実装プラン。production は直接 `umbreon::Scene` 渡し、umbreon が検出＋可視性＋
  クリッピングを所有。fp32/fp64 精度エスカレーションと PoC ファースト方針を含む。
