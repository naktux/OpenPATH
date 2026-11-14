# OpenPATH

**Alpha · Visual autonomous planning for FRC 2026 REBUILT, presented by Haas.**

OpenPATH is a local browser app for placing robot poses on a field, building reusable command chains, and exporting Java that matches the team's custom WPILib library. It follows the same structure as the robot code: **waypoints → commands → autonomous sequences**.

## Quick start

Install Node.js and npm. Node.js 22 is recommended for this project.

Clone the repository, then run these commands in PowerShell:

```powershell
git clone https://github.com/naktux/OpenPATH.git
cd OpenPATH
npm.cmd ci
npm.cmd run dev
```

Open **http://127.0.0.1:5173/**. If that port is occupied, use the address Vite prints in the terminal. Keep the terminal running while using the app; press **Ctrl+C** to stop it.

In other shells, use `npm` instead of `npm.cmd`.

The app starts with the bundled `RobotContainer.java` example. You can explore it immediately or use **Import / Open** to load your own compatible Java file or an OpenPATH JSON save.

## Start with MagicPath

MagicPath turns field clicks into a reusable command without requiring you to assemble each drive step manually.

1. Click **MagicPath** in the right sidebar.
2. Click the field to place stops in order. A dotted line follows your cursor to preview the next segment.
3. Hold **Shift** to constrain the next segment horizontally or vertically. The dominant direction selects the axis; it stays locked until you release Shift or place the stop.
4. Click near an existing waypoint to reuse its ID, coordinates, and heading. The default matching distance is **0.15 m**, adjustable under **Snap settings**. Snapping respects the active axis lock.
5. Adjust the stops: reorder or remove them, undo the last stop, or edit a new stop's position and heading. New waypoints face the next stop automatically unless you choose a manual heading.
6. Click **Review path**, name the command, and optionally adjust the drive function, speed, and arrival tolerance. When launched from a selected sequence, you can also append the command to that sequence.
7. Click **Create & customize**. The step editor opens so you can switch drive steps to intake functions, add aiming or shooting actions, insert waits, or reorder the chain.

MagicPath requires at least two stops. Reused waypoints keep their saved pose; choose **Make independent** to create a separate waypoint you can change. Cancelling the drawing workflow leaves the project unchanged. After **Create & customize**, the command has been created; cancelling the subsequent step editor only discards those additional edits.

## Workspace

The **left sidebar** contains properties, the **center** contains the field and collapsible Java panel, and the **right sidebar** contains the project library.

| Library mode | What it contains |
| --- | --- |
| **Sequences** | Named autonomous routines registered with the robot's chooser. |
| **Commands** | Reusable functions containing an ordered chain of actions. |
| **Waypoints** | Shared field positions and headings referenced by commands. |

Use **Add sequence**, **Add command**, or **Add waypoint** to open the corresponding editor. Changes in these dialogs apply when you save; **Cancel** or **Escape** discards the draft.

### Waypoint editing

- Drag the robot outline to move a waypoint; drag its round handle to rotate it.
- Enter exact X, Y, and heading values in the properties panel or **Edit on field** dialog.
- Drag the field background to pan. Use the zoom controls or mouse wheel, and **Reset view** to return to the default view.
- Enable **Snap** for a 0.05 m position grid and 15° heading increments in the main field editor. MagicPath's nearby-waypoint matching is a separate setting.
- Open **Robot & calibration** to set bumper length, width, and front direction. The initial **0.9 × 0.9 m** footprint is a placeholder; replace it with your measured dimensions.

Each waypoint has a stable numeric ID, displayed inside its robot outline and throughout the editors. IDs survive renaming, reordering, and JSON saves. Duplicates receive new IDs, and deleted IDs are not reused. A fresh Java import assigns IDs in source order.

Waypoint edits affect every command that references that waypoint. Duplicate it when you need an independent pose. Remove or reassign references before deleting a waypoint that is in use.

### Command and sequence editing

Use **Edit step chain** to choose a step type and configure its settings. Supported choices include waypoint functions from the loaded robot library, reusable commands, waits, and nested command groups.

Each step exposes relevant settings, such as its waypoint, arrival tolerance, or speed. Drag steps to reorder them, or use the up/down buttons. Changing between compatible drive and intake functions preserves the selected waypoint and shared parameters.

For existing sequences, **Organize** lets you select consecutive top-level steps and turn them into a reusable command. Hovering a step focuses its path on the field; **Full path** displays the entire selected sequence.

Complex Java predicates, suppliers, and lambda bodies remain text expressions. Advanced settings preserve imported command modifiers such as timeouts and conditions.

## Coordinates and robot conventions

| Property | Convention |
| --- | --- |
| Position units | Meters |
| Origin | Purple pixel **(92, 731)** in the bundled **1601 × 779** field image, measured from its top-left |
| Positive X | Right, toward the red side |
| Positive Y | Up |
| Heading | Degrees counterclockwise from +X |
| Robot pose | Center of the robot outline |
| Default robot front | Robot +X; configurable |
| Field dimensions | **16.541 × 8.069 m** |

Image margins are not field coordinates. Calibration uses the inside field-wall scale and the marked origin; replacing or resizing the image requires updating its calibration.

The supplied robot code stores blue-frame waypoints and flips them for red alliance at runtime. OpenPATH preserves that logic. **Preview red** changes the visualization without rewriting stored coordinates or adding another flip to the export.

## Playback

Select a sequence or command and press **Play** to animate its waypoint progression. You can pause, scrub the timeline, change playback speed, or stop the preview.

Playback is an estimated visual preview, not a robot physics or trajectory simulation. It does not predict acceleration, collisions, mechanism behavior, or live vision updates. Conditional groups preview their first branch, repeated groups preview one cycle, and concurrent groups preview a representative drive branch. Runtime conditions and dynamic headings are not simulated.

## Saving and importing projects

- **Save project** downloads a JSON file containing waypoints, IDs, commands, sequences, robot settings, calibration, and imported Java source.
- **Import / Open** loads an OpenPATH JSON save or a compatible `RobotContainer.java` file. Loading a file replaces the active project.
- The app also autosaves to browser storage after editing settles. Use JSON downloads for portable backups.

Browser autosaves belong to that browser and origin, including the port. Switching browsers or ports, clearing site data, or using a private session can make an autosave unavailable. Local Java and JSON files are processed in the browser; OpenPATH does not write directly to your robot repository.

## Java export

OpenPATH targets the supplied team's Java 17/WPILib architecture:

| Layer | Export structure |
| --- | --- |
| Waypoints | `positions` enum entries and `waypoints.put(...)` definitions using `Pose2d` and `Rotation2d.fromDegrees(...)`. |
| Commands | Reusable `Command` methods calling the loaded helpers, such as `driveTo`, `driveToWayPoint`, and `driveToIntakeWayPoint`. |
| Sequences | `SequentialCommandGroup` routines registered through `m_chooser.addOption(...)` or `setDefaultOption(...)`. |

Expand **Java export** to view highlighted routine, waypoint, reusable-command, or full-file code. Use **Copy code** for the displayed section, or **Export Java** to download a complete updated `RobotContainer.java`.

Paste sections into the locations indicated by their comments. The full-file export preserves unrelated source and existing initialization/alliance behavior. OpenPATH never overwrites your local robot file automatically.

The app checks references, identifiers, recursive command definitions, and Java syntax. **“Syntax valid” does not mean the code has been compiled against your robot dependencies.** Build the exported file in your robot project before using it on the robot.

## Current limits

- Java import/export is designed around the bundled `RobotContainer.java` structure and custom library. It is not a general-purpose Java editor or a C++ exporter.
- Field connections are waypoint-order guides; OpenPATH does not generate splines, perform obstacle avoidance, or calculate a dynamically feasible trajectory.
- The current editor labels helpers with a `positions` parameter as **Go to waypoint**, including `aimingAutonCommand` and `firingAutonCommand`. Their actual function names and exported behavior are preserved.
- There is no live robot connection or deployment control.

## Development

The app uses React, TypeScript, Vite, SVG field rendering, `java-parser`, and Highlight.js. Java export validation runs in a Web Worker so dragging does not repeatedly parse the full file on the UI thread. Motion effects respect the operating system's reduced-motion preference.

```powershell
npm.cmd test        # Model, geometry, identity, and import/export regression tests
npm.cmd run build  # Type-check and create the production build in dist/
```

Browser interactions can be checked with Playwright. Those checks are development tooling; Playwright is not required to run OpenPATH.

### Optional robot compilation checks

The local `2026-FRC-Rebuilt/` robot reference project is ignored by Git and is not required to run the planner. The bundled example is `public/RobotContainer.java`.

If you have that local reference project and the matching WPILib Java toolchain, prepare an isolated edited-export check:

```powershell
node --import tsx tests/prepare-compile.ts --edited
cd output/compile-check
.\gradlew.bat compileJava --console=plain --no-daemon
```

Configure `JAVA_HOME` for your WPILib installation first. The helper copies the local robot project into `output/compile-check` and writes a representative edited export there, leaving the reference project untouched. Generated output, browser artifacts, dependencies, and production builds are ignored by Git.
