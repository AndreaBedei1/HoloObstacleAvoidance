# Legacy primitive scenarios (unsupported)

These scenarios approximate obstacles with spawn_prop primitives (spheres /
boxes) on the packaged stock HoloOcean worlds. They are **no longer part of
the supported workflow**: the main simulation path uses the EXTERNAL modified
engine with the real custom anchor mesh — see
`../custom_anchor_visible.yaml` and the "Custom Real-Anchor Worlds" section
of the repository README.

Kept only for reference and for loader/oracle regression tests. They may
still work against the packaged stock worlds, but they are not maintained,
not the default anywhere, and not exercised by the closed-loop scripts.
