# Project guidance for AI coding agents

- Read the relevant design notes in `docs/` before changing behavior. Keep those
  notes and the README aligned with user-visible changes.
- Add Doxygen comments when adding or changing project-owned C++ public APIs in
  `src/`: types, functions, methods, and non-obvious fields. Put comments by the
  declaration in the header. Describe the purpose, units and coordinate space,
  input constraints, return or failure behavior, and side effects where relevant.
- Use `///` for short descriptions and `/** ... */` with `@param`, `@return`, or
  `@warning` when the contract needs them. Explain behavior and invariants rather
  than repeating the C++ signature. Keep documentation accurate when code changes.
- Add Doxygen comments to internal algorithms when their assumptions or steps
  are not clear from the code. Do not blanket-document trivial implementation
  details or edit vendored code under `lib/` and `tools/BSAFileExtractor/`.
- Keep `docs/api.md` and `Doxyfile` current when adding a major module. If
  Doxygen is installed, run `doxygen Doxyfile` and resolve new documentation
  warnings before finishing.
- When changing a user-visible analysis or export option, trace every entry
  point that calls `app::Run`, including the Windows UI and CLI. Expose the
  setting in the UI when operators need it, persist and validate its value,
  and verify that the UI passes it to the shared run path. Build the desktop
  executable but do not test the actual UI workflow, only CLI invocation.
- Treat named cells and mods supplied as bug examples as reproduction data.
  Keep behavior, UI help, and synthetic fixture names location independent
  unless a location-specific rule is explicitly required.
- When changing behavior, do not add comments mentioning the previous behavior.
