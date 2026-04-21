# dynamic_voronoi

Dynamic Voronoi provides incremental Euclidean distance map and Voronoi diagram updates on occupancy grids.

This repository builds a C++ library and an example program:
- Library target: `dynamicvoronoi`
- Executable target: `example`

## Features

- 2D dynamic distance transform and Voronoi diagram update
- 3D implementation included in the same library
- Incremental obstacle updates for dynamic environments
- PPM visualization output for quick inspection

## Repository Layout

- `include/`: public headers
- `src/`: source code and example program
- `data/testmap.pgm`: sample PGM map for the example
- `output/`: optional folder to store generated images

## Build (Windows, Visual Studio Generator)

From repository root:

```powershell
cmake -S . -B build
cmake --build build --config Debug --target example
```

Notes:
- Do not use `make` with Visual Studio generators.
- If the project is already configured, only run the second command.

## Run Example

From repository root:

```powershell
.\build\Debug\example.exe .\data\testmap.pgm
```

Or from `output/` (writes outputs into `output/` directly):

```powershell
..\build\Debug\example.exe ..\data\testmap.pgm
```

Optional second argument:

```text
prune
pruneAlternative
```

Both are accepted by the current example program.

## Expected Output

Successful run prints messages similar to:

```text
Map loaded (...).
Generated initial frame.
Performed update with random obstacles.
Done with final update (all random obstacles removed).
```

Generated files:
- `initial.ppm`
- `update_001.ppm` to `update_010.ppm`
- `final.ppm`

Output files are written to the current working directory.

## Common Issues

### 1) `Unknown argument --target`

Cause: command typo such as `cmake cmake --build ...`.

Fix:

```powershell
cmake --build build --config Debug --target example
```

### 2) `Error reading pgm map.`

Cause: binary PGM (`P5`) must be read in binary mode on Windows.

Status: fixed in `src/example.cpp` by opening the map with `std::ios::binary`.

### 3) `Could not find catkin`

If ROS/catkin is not installed, the project now still configures and builds in plain CMake mode.

## License and Citation

This project is commonly distributed under BSD terms and is based on the dynamic Voronoi update method described by:

- B. Lau, C. Sprunk, W. Burgard, IROS 2010.

If you use this project in research, please cite the original work.
