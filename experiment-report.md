# Latest update: levels and tour

See [Levels and Pairdriver tour](levels-tour-report.md) for the current level flow and Full Automation takeover behavior. A continuous two-second input now switches AD to Manual; AD back on restores automation. This supersedes older emergency-only input descriptions below.

# CATNAP CLEANER — experiment update

## Architecture inspected and preserved

| Responsibility | Existing files reused |
|---|---|
| Modes, locked runs, comparison | `src/game/Session.ts`, `src/game/types.ts`, `src/main.ts` |
| Movement, navigation, shared authority | `src/game/Simulation.ts`, `src/control/SteeringSystem.ts`, `DirectionControl.ts`, `IntentDetector.ts`, existing route planners |
| Joystick / Pointer Events | `src/control/Input.ts` |
| Rider cat and disturbance | `src/systems/DisturbanceSystem.ts`, `src/world/Renderer.ts` |
| Camera | `src/world/Renderer.ts`, `GoldCamera.ts` |
| Baseline sound, haptics, settings persistence | `src/systems/Feedback.ts` |
| Settings and experiment export UI | `src/ui/UI.ts`, `src/main.ts` |
| Metrics / telemetry | `src/game/Simulation.ts`, `types.ts`, `Session.ts` |

The original haptic baseline is event-based: intervention 6 ms, conflict 9 ms, recognition [12,30,12], sync 15 ms, bonus 12 ms, and collision [40,20,20]. These remain intact when Dynamic Force Feedback is OFF.

No changes to `SteeringSystem.ts`, `ReturnPlanner.ts`, `InclusionPlanner.ts`, `PathPlanner.ts`, `DisturbanceSystem.ts`, or `Session.ts`. The loop geometry, return/recovery methods, collaboration state machine, and magnet logic are unchanged. The only `Scenario.ts` change passes the initial position to gold placement.

## Added and modified files

Added: `src/experiment/ExperimentConfig.ts`, `CriticalEvents.ts`, `DynamicForce.ts`, `TrialArchive.ts`; `src/control/EmergencySwipe.ts`; `src/world/BlackCatRenderer.ts`; `tests/experiment.test.ts`, `critical-statistics.test.ts`.

Modified integration points: `Simulation.ts`, `types.ts`, `Input.ts`, `Feedback.ts`, `main.ts`, `UI.ts`, `style.css`, `Renderer.ts`, `GoldCamera.ts`, `BonusPlacement.ts`, and the single gold-placement call in `Scenario.ts`. README, validation notes, this report, and the downloadable source archive were refreshed.

## Critical events and emergency control

A separate charcoal-black cat rests on existing furniture surfaces. With events Off it remains passive and has no hazard collider. When enabled it can jump down, approach through navigable floor, enter an actionable forward corridor, pause in surprise, then cross or flee. Later roaming and event scheduling use runtime randomness independent of the cleaning scenario seed. Events are serialized, with a minimum interval after resolution; irrelevant approaches do not start reaction timing.

On ACTIVE_HAZARD, Full Automation draws `random() < fullAutoCriticalAvoidSuccessProbability`. A capable response attempts a swept-clear lateral maneuver; an inadequate response continues ordinary navigation. This probability applies only to black-cat events. Collision and success are subsequently measured from actual positions, not assigned from the random draw. Successful draws are not invulnerability; geometry can still defeat an attempted avoidance.

A rolling Pointer Events window recognizes a quick sideways drag, including a jerk from an already-held joystick. Small/slow motion, pre-onset motion, extra fingers, and duplicate accepted swipes do not trigger responses. The screen vector uses the existing camera-to-XZ transform. All modes can evade; Full Automation still discards every ordinary steering command.

The maneuver adds a short lateral displacement and briefly holds heading to prevent an immediate pull back into the hazard. It then hands movement back to the existing controller, without introducing a path-rejoin state or modifying any route planner. A swipe toward a blocked side can still fail.

Reaction time is `(humanInterventionTimestamp - hazardOnsetTimestamp) * 1000`, using the same paused simulation clock in all modes. Resolution is about 16.7 ms; these are game timestamps, not laboratory-calibrated sensor latencies. Missing responses remain null. Late responses, autonomous success, human rescue, combined avoidance, ordinary avoidance, collision, irrelevant approaches, and interrupted events remain distinguishable. Combined maneuvers do not falsely attribute success solely to either actor.

Direct collisions add substantial disturbance, recoil the Roomba, startle its rider, and make the black cat flee toward another valid rest location. Successful avoidance/near misses add less disturbance; irrelevant encounters add none. The black cat continues its visual escape during the game-over animation.

## Central tuning and settings

All new experiment constants are in `src/experiment/ExperimentConfig.ts`.

| Parameter | Default |
|---|---|
| Critical-event frequency | Off; Low 28–42 s, Medium 20–30 s, High 14–22 s |
| Full Auto capable avoidance probability | 0.10; editable in Settings |
| Black-cat approach / flee speed | 7 / 9 world units per second |
| Direct collision / near miss disturbance | +32 / +5 percentage points, capped at 100 |
| Gesture threshold | 48 CSS pixels, 700 px/s, maximum 0.22 s; rolling window 0.14 s |
| Evasive motion | 8 units/s lateral impulse for 0.30 s; heading clearance hold 1.35 s |
| Human intervention window | 1.15 s from actionable onset; physical impact can happen earlier |
| Dynamic Force Feedback | ON; independent Settings toggle |
| Static-obstacle sensing | 3.5 units forward; proximity exponent 1.6 |
| Authority multiplier | 0.30 at full player authority; 0.65 balanced; 1.00 at full Roomba authority |
| Force smoothing / pulse range | Rate 5; 2–7 pulses/s; 12–45 ms pulses |
| Gold placement | 10.5–13 units away; at least 6 units from initial position |
| Gold value / duration | Unchanged: 1,500 points / 18 s |
| Camera | Normal zoom 1.3 (short landscape 1.6, portrait 0.92), minimum 78% of normal zoom in landscape / 68% in portrait, easing rate 2.8 |
| Camera stability | 36 px side padding, 0.08 zoom dead band, 0.65 s return delay |

Frequency intervals are lower bounds between opportunities; the cat's approach and need for a genuinely actionable, escapable corridor can delay a hazard. Critical events default Off to preserve ordinary replay behavior. Enabled critical events intentionally do not replay identically across comparison modes; the house, cleaning route, and scheduled bonuses still do.

## Dynamic Force Feedback

The sensor sweeps the Roomba footprint forward against static walls/furniture only, selects the first relevant obstacle, and derives proximity/time-to-collision risk. Objects behind or safely beside the Roomba do not dominate feedback. Black-cat hazards use their own cues and never enter this static-obstacle calculation.

Force = obstacle risk × authority multiplier, followed by exponential smoothing. More player authority means less force; more Roomba authority means more force. Pulse duration and frequency convey intensity, with a soft audible pulse and a visible resistance percentage on the shared-direction readout. Settings includes a feedback test and reports missing or blocked vibration support. Existing event cues take priority, with a cooldown to avoid vibration spam. A subtle amber edge on the shared-direction readout provides a visual fallback.

OFF removes only this additional obstacle/authority modulation. Original Collaborative feedback, collision feedback, critical-event cues, audio, and controls remain available. The separate Haptics preference still controls hardware vibration globally. Browser vibration provides pulse timing, not true mechanical resistance or controllable hardware amplitude; unsupported devices retain visual/audio feedback.

## Telemetry and export

Settings and up to six recent trial snapshots are stored in localStorage. Storage pressure may retain fewer snapshots; failed persistence leaves the current in-memory export available. Export after each trial for durable research collection. No backend or transmission is involved.

Download JSON from Results, Pause, Settings, or Final Comparison. Schema version 10 contains uniquely identified `trials[]`, their initial/final settings, configuration-change events, tuning snapshot, mode/seed/status, metrics, critical-event rows, and sampled robot/black-cat positions and shared authority. Samples include Dynamic Force Feedback state and, only while active, obstacle distance/risk, generated force, time to collision, and minimum clearance. Vibration API availability is recorded; generated force does not imply hardware delivery.

Critical rows contain onset/intervention/resolution timestamps, reaction milliseconds or null, the configured probability and random draw, attempt and physical outcomes, near miss/collision, pre/post disturbance, and player/Roomba authority at onset. Incomplete attempts are labelled and excluded from the existing completed-run comparison ledger. Duplicate saves replace the same trial ID.

## Validation and remaining manual checks

The final `npm test` run passed all 85 checks, including the physical statistical sample. A deterministic-telemetry regression found during development was fixed by keeping the passive OFF cat at a fixed rest location; enabled event randomness remains independent.

The physical statistical test ran 96 complete or bounded Full Automation simulations: 321 resolved actionable events, 27 capable draws (8.4%), 25 autonomous successes (7.8%), 293 collisions (91.3%), and 3 other avoided encounters. Six irrelevant approaches were excluded from actionable-event statistics. No human responses were supplied. This is consistent with probabilistic variation around the configured 10% response chance; the implementation does not impose a nine-fail/one-win sequence.

Focused checks cover passive cat behavior, event intervals, duplicate-event/collision prevention, all-mode reaction and rescue, Full Auto input restrictions, flee/cooldown, interrupted events, swipe rejection, authority scaling, static-obstacle relevance, baseline haptics when OFF, pulse limits, settings/export persistence, and smooth camera fitting/hysteresis. Existing cleaning completion, shared control, path-loop/return, magnet, audio, touch, and PWA checks remain covered. Protected source blocks were compared directly with the previous commit.

TypeScript compilation, Vite production bundling, and offline-cache generation are release checks. No new browser visual or physical-device test was completed: the earlier preview was blocked by browser URL policy. Android/iOS gesture comfort, vibration strength, rendered black-cat placement/jumps, camera comfort, audio on speakers, accelerated WebGL performance, and native PWA installation still require manual gameplay testing.

## Camera, feedback, and loaf-pose refinement

Gold framing is now calculated from the destination pair and camera orientation rather than the lagging camera position. Compact HUD padding and bounded zoom preserve object scale. The camera eases toward the padded pair center. The waiting black cat now tucks all paws beneath a rounded body, wraps its tail, narrows its eyes, and smoothly unfolds to walk; the rider also has a more compact sleeping silhouette.

31 targeted checks passed for camera framing, feedback fallback and API failure reporting, loaf transitions, baseline sounds, critical-event behavior, and gold inclusion. TypeScript and the production build are release checks. Actual speaker/vibration feel and rendered appearance still require physical-device review; no new browser visual test was performed. The original loop, return/recovery, magnet, and critical-event mechanics were not changed.

## Status placement and attentive black cat

Removed the gameplay path legend. Non-impact status messages now occupy a touch-through row directly under the disturbance/shared-direction meters; collision bonks remain beside the Roomba. The black cat visually tracks the displayed Roomba in menus and gameplay, including targets behind it, while its loaf tail gently swishes. Navigation and critical-event behavior are unchanged. Five focused rendering/math/feedback checks passed, including all-direction gaze and visible resting-tail motion. Browser visual review remains outstanding.

Head-only gaze correction: the black cat body now follows its locomotion heading only. The head eases toward the Roomba within an 80-degree turn limit; loaf tail animation remains intact. Five focused framing/feedback/model checks passed.


## Pairdriver teaching and remaining-floor navigation (October 2026)

Added a cited, in-game Pairdriver explanation accessible from Main Menu, Settings and the pre-run welcome. It distinguishes physical haptic control from the game's direction blend and vibration/audio/visual analogy. New runs wait on a mode-specific welcome explaining objectives, controls, scoring and collectibles. Free Play and next Comparison modes begin only on Start cleaning; fast retry remains immediate.

The existing return planner can now consult the actual floor map and choose a reachable lane with remaining dust. Full Automation and passive Collaborative automatically plan toward remaining work after route displacement or prolonged travel over cleaned floor. Connectors, merge checks, return recovery, speed, original loop geometry and magnet physics remain in place. Cleaning-only redirects generate a cleaning_repath event but do not generate collaboration-return score or metrics.


## Welcome slides and full intervention share

The welcome now uses four illustrated slides (goal, controls, pickups, mode behavior) with fixed Previous/Next navigation and Start cleaning on the last slide. Opening the Pairdriver page preserves the current slide; each new comparison mode resets to slide one. Normal phone layouts no longer require scrolling to reach the navigation. Enlarged text can scroll the content area while navigation remains visible.

Player share supports 10–100% in 5% steps, including local persistence. At 100% and full drag, the pre-inclusion vector blend gives the player full authority; yielding does not lower it to 90%. Existing inclusion locking, navigation, recovery, and magnet mechanics remain unchanged.

Validation: focused welcome, settings, and control tests plus TypeScript/production build. Physical-device layout and touch testing still recommended.


## Compact mode selection and Pairdriver pages

Mode selection now fits three compact cards and Start on a viewport-sized panel. Portrait uses three horizontal rows; landscape uses three columns. Brief descriptions are hidden only on very short portrait screens. Pairdriver now presents six concise pages with Previous/Next, a page counter, and Back to game on the final page. Research links and the simulation limitations remain available. The originating welcome slide and Settings screen are preserved when returning.

Validation: six focused welcome/information navigation tests pass; TypeScript and production build pass. Gameplay controllers, paths, recovery, magnet, authority settings and telemetry are unchanged. Browser/physical-device layout verification was not performed in this update; oversized accessibility text may scroll within the information content while navigation stays visible.


## Rotation and Pairdriver diagrams

The PWA manifest now uses orientation=any instead of landscape. The rotate-to-landscape hint is removed. Existing viewport resize handling and responsive HUD remain in place. Device auto-rotation must be enabled; an older installed shortcut may need removal and reinstallation if its manifest remains cached.

Five compact SVG teaching illustrations show the shared-direction HUD colors, example authority changes, gold inclusion planning around an obstacle, and the correct authority/obstacle-feedback relationship. Research links remain on the final information page. Diagrams are schematic examples, not live telemetry.

Validation: nine focused PWA and welcome/information tests passed, including offline service-worker behavior; TypeScript and production build passed. Installed-app rotation and physical-device layouts still require hands-on checking.
