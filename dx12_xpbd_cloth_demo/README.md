# DirectX 12 XPBD Cloth Sample

This project is a small DirectX 12 sample that demonstrates:

- XPBD-based cloth simulation
- support for arbitrary triangle meshes, not just a fixed grid
- sphere and capsule collision volumes
- collider dragging by cursor in the viewport

It is intentionally small and self-contained so that it can be used as a starting point for a larger DX12 simulation project.

## Features

- Any input mesh can be converted into a cloth by supplying vertices and triangle indices.
- Cloth is solved using a position-based XPBD constraint loop.
- Sphere and capsule colliders are supported and can be moved with the mouse.
- Rendering is done with DirectX 12 using a simple unlit shading pipeline.

## Build on Windows

Use CMake:

```bash
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

Then launch the generated app from `build/Release/dx12_xpbd_cloth_demo.exe`.

## Controls

- Left click: pick a collider and drag it.
- Right click / drag: move the active collider along the ground plane.
- Space: reset the cloth.

## Notes

This sample is meant as a practical starting point rather than a full production renderer. The solver is CPU-based, which makes the implementation easier to understand and modify.
