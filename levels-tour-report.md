# Levels and Pairdriver tour

The canonical GitHub source was inspected at `ec4db51be424799cc2ce51ab62e1996f2a13087a` before editing. Its tracked files matched the hosted source checkout.

## Player flow

- Play opens level selection: Pairdriver tour, Level 1 (the existing home), and Level 2 (Open room).
- Level 1 retains the original furniture, path loop, pickups, and experimental black-cat options.
- Level 2 uses the same large 26 × 18 floor, with only a box and a plant. The black cat is absent and critical events stay off, including if a saved experiment setting is enabled.
- Ordinary levels offer Start and Guide beside each other. Start enters play immediately. Guide presents the four existing illustrated pages as a side panel with the simulation paused.
- The tour always begins with welcome and controls pages. Explanation cards pause simulation; exercises run at 55% speed. Previous/Next and horizontal swipes navigate the cards, with Next gated by the actual task.

## Tour tasks and defaults

1. Interaction: use the existing direction blend at 30% player / 70% system; then move a lesson-only slider to 90% / 10% and steer again. Intent acceptance is suspended only during this weight demonstration. Opposition drives the existing pulse feedback API and a visual resistance indicator. Saved player preferences are not changed.
2. Arbitration comparison: in Full Automation, a continuous input of at least 0.35 magnitude for two real seconds switches to Manual. AD back on reactivates automation and uses the existing return-to-cleaning behavior. The same takeover behavior is available in ordinary Full Automation runs. Short inputs remain ignored for normal steering; emergency hazard swipes remain immediate.
3. Pairdriver arbitration: Collaborative remains active while the existing intent detector and yielding states change the balance smoothly.
4. Inclusion: reveal a nearby red +250 target, then gold at least 10.5 units away. Gold uses the existing sustained-aim inclusion lock, path planner, collection, and return to unfinished floor. Task completion requires actual collection and return, not a time-based substitute. Teaching pickups remain available for 600 simulation seconds.

Phone direction blending, warning vibration and sound are a simplified illustration. They do not apply steering-wheel force or estimate arm impedance. The comparison takeover behavior is a game condition, not a claim about Pairdriver.

## Architecture

- `Levels.ts` centralizes room definitions and takeover/tour tuning.
- `Tour.ts` orchestrates the existing simulation; it has no separate motion controller.
- `Scenario.ts` gives each grid its own collision query. Path, inclusion, return, bonus placement and obstacle feedback accept that query. The default room and algorithms are preserved; the route-loop generator is unchanged.
- `Simulation.ts` owns effective mode transitions, teaching pickup reveal and the temporary weight-only lesson flag.
- `TourPanel.ts`, `UI.ts`, `PageSwipe.ts`, and `BackNavigation.ts` own teaching presentation, page gestures, and browser Back routing. Floor swipes remain separate from page swipes.
- Results use compact two-column metrics and visible action buttons. Native Back pauses play, then returns through app parent pages; at home the next Back can leave.

## Data

Trial schema 11 records level and initial mode, while each existing telemetry sample records the effective mode. Takeover, AD reactivation, tour steps and completion are events. Tutorial flow is labelled `tour`; it does not enter scored comparison results. Existing experiment settings, critical-event logging and device-local JSON export remain available.

## Validation

114 automated tests passed in total: the complete 112-test suite plus two subsequently added tests for the full uninterrupted tour and native Back history behavior. TypeScript validation, production build and offline-asset validation passed.

The open room completed actual floor coverage without collisions. The full scripted tour completed all tasks with two bonus collections and zero collisions. Tests also cover two-real-second takeover under slow motion, authority weights, sparse collision geometry, critical-cat suppression, slide navigation and guide/start choices. Existing navigation, magnet, critical-event statistics and feedback tests passed.

Browser preview control is unavailable in this environment, so no rendered browser/device QA was performed. Android installed-app Back behavior, portrait/landscape layouts, touch comfort and physical vibration strength still require hands-on gameplay checks.

## Guide visibility correction

The guide originally covered the robot at the default screen centre on portrait phones. Camera framing now uses the guide’s measured rectangle and the top HUD bounds to select uncovered floor. Robot visibility takes precedence if distant gold cannot fit within the existing comfortable zoom cap. A minimal camera translation keeps the robot visible during eased transitions. Normal gameplay framing is unchanged when no guide is open.

Practice cards open compactly and offer Instructions / Minimise controls; explanations remain expanded. The camera recalculates for expanded cards, compact cards, screen rotation and viewport size.

19 focused tests passed with TypeScript validation and the production/offline build. These include full-body projection checks for the reported portrait layout, smaller phones and landscape, plus transition safety, guide resizing and compact-card controls. Browser/device visual QA remains unavailable.
