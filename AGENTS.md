# Project: OpenPATH (FRC Autonomous Path & Code Generator)

## Core Purpose
OpenPATH is a visual desktop/web tool for planning FIRST Robotics Competition (FRC) autonomous routines and exporting drop-in code matching our team's custom WPILib command-based architecture.

## Team Library Architecture (Strict Rules)
All generated code MUST adhere to our 3-tier hierarchy (see `@examples/` for exact imports, class signatures, and coordinate conventions):
1. **Waypoints:** Named constants/objects representing FRC field coordinates `(X, Y)` and heading/rotation `(theta)`.
2. **Autonomous Commands:** Reusable command methods/factories that string together waypoints (e.g., `driveTo(waypoint)`, `driveToWithIntake(waypoint1, waypoint2)`).
3. **Autonomous Routines (`SequentialCommandGroup`):** Top-level classes/methods bundling commands in sequence, assigned a human-readable name (e.g., `"Left Bump Far"`), and registered to our `SendableChooser` module.

## Tech Stack & UI Standards
- **Frontend:** React + TypeScript + Tailwind CSS + HTML5 Canvas (or Konva/SVG) for smooth 60fps field rendering.
- **Coordinate System:** Strictly map canvas pixels to real FRC field meters/inches (origin and rotation conventions must match our robot codebase).
- **Design:** Dark-mode robotics telemetry aesthetic (high contrast, crisp grid lines, clear robot bumper outline with heading indicator, zero clutter).

## Verification Rules
- Never hallucinate custom library method names; only use methods defined in the user's loaded project or `@examples/`.
- Run type-checks and verify that exported Java/C++ code compiles cleanly against our template syntax.
