# DirectX 12 XPBD Cloth Demo

This project is a compact DirectX 12 sample that demonstrates a position-based cloth solver with:

- arbitrary triangle mesh input
- a simple XPBD-style stretch constraint solver
- sphere and capsule colliders
- cursor-driven collider dragging in the viewport

## Build on Windows

Use CMake in a Visual Studio 2022 environment:

```bash
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

Then run the generated executable from the build output directory.

## Controls

- Left mouse drag: move the selected collider
- Space: reset the cloth

## Notes

This sample is intentionally small and readable. It is meant as a starting point for a real cloth simulation in DirectX 12 rather than a production renderer.
