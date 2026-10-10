# Project guidance for AI coding agents

- Read the relevant design notes in `docs/` before changing behavior and keep
  those notes aligned with the changes. The README is a welcome page for users,
  not a technical change log. Update it only when the introduction, getting-started
  steps, or user workflow meaningfully changes; document implementation details,
  bug fixes, and algorithm contracts in the relevant `docs/` pages instead.
- Add Doxygen comments when adding or changing project-owned C++ public APIs in
  `src/`: types, functions, methods, and non-obvious fields. Put comments by the
  declaration in the header. Describe the purpose, units and coordinate space,
  input constraints, return or failure behavior, and side effects where relevant.
- Use `///` for short descriptions and `/** ... */` with `@param`, `@return`, or
  `@warning` when the contract needs them. Explain behavior and invariants rather
  than repeating the C++ signature. Keep documentation accurate when code changes.
- Add Doxygen comments to internal algorithms when their assumptions or steps
  are not clear from the code. Do not blanket-document trivial implementation
  details or edit vendored code under `lib/`.
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
- Do not hardcode values in the documentation. These values can evolve

## Readability and design

- Write project-owned code for a human reader. Use descriptive names, explicit
  control-flow blocks, and one statement per line. Format C++ with the repository
  `.clang-format`; restrict formatting to project-owned files under `src/` and
  any tests being changed. Never format vendored dependencies.
- Preserve dependency-sensitive include order, especially Windows SDK and NIF
  headers. Formatting must not reorder includes across prerequisite headers.
- Give each function one coherent responsibility and keep its abstraction level
  consistent. Extract named helpers when a function mixes independent stages or
  needs deep nesting. Prefer cohesive helpers over arbitrary line-count limits;
  keep a longer algorithm together when splitting it would obscure its invariants.
- Introduce each non-obvious algorithm stage with a comment explaining its
  purpose, assumptions, and data invariants. Explain units, coordinate changes,
  index remapping, ownership, and failure policy where they matter. Do not narrate
  obvious statements or use comments to compensate for misleading names.
- Apply SOLID proportionately: separate orchestration, domain algorithms, input
  resolution, and serialization (single responsibility); add implementations
  behind established extension points (open/closed); preserve interface contracts
  in every implementation (Liskov substitution); expose only the operations a
  caller needs (interface segregation); keep neutral algorithms independent of
  platform and Skyrim I/O and use narrow existing boundaries for replaceable
  policies (dependency inversion).
- Prefer plain functions and value types for stateless work. Introduce a class
  only for a cohesive responsibility with state or an actual polymorphic boundary.
  Do not add speculative interfaces, inheritance, frameworks, or dependencies
  solely to demonstrate SOLID.
- Keep behavior-preserving refactors separate from feature changes. Preserve
  output formats, diagnostics, processing order, cancellation, and error handling;
  use existing regression fixtures and CLI checks to verify those contracts.
- Document new module responsibilities in `docs/architecture.md` and `docs/api.md`.
  Build the desktop executable and run relevant automated tests after refactoring;
  ensure assertions are enabled when running the assert-based C++ test suite.
