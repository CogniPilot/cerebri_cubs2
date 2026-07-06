Generated control artifacts do not live in this directory.

Rules:
- Do not commit generated C/H artifacts here.
- Keep the controller source in `modelica/FixedWingOuterLoop.mo`.
- Let CMake run pinned Rumoca and write generated eFMI output under
  `${CMAKE_BINARY_DIR}/generated/rumoca`.
- If generated behavior needs to change, update the Modelica source instead of
  patching generated output.

Current status:
- `FixedWingOuterLoop` is generated from `modelica/FixedWingOuterLoop.mo` with
  Rumoca `v0.9.11` and the `galec-production` target.
- The active flight stack calls generated eFMI controller code through thin
  handwritten wrappers; controller equation changes belong in Modelica.
