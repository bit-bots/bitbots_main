# Bit-Bots AutoReferee

Standalone referee infrastructure for a separately running MuJoCo simulation.
The implemented component sends the complete match state to a GameController
receiver using UDP unicast and receives the robot's return packets. Both teams
are always included, regardless of which team the receiving robot belongs to.
The existing `game_controller_hsl` binary schemas are reused unchanged.

## Scope

The launch file starts the referee process and composes the implemented match
state, opening sequence and UDP adapter. It does not start the simulator or
duplicate a robot's receiver. The opening sequence automatically advances from
INITIAL through READY and SET to PLAYING. The phase durations are defined in
`rules/startup.py`; READY and SET publish their remaining preparation time in
`secondary_time`. Transitions are sent immediately, in addition to the heartbeat.
Robot and ball positions and ball contacts are observed. The remaining half time
counts down using simulation timestamps only during PLAYING with `stopped=false`.
Fractional seconds accumulate across updates and are preserved across stops;
preparation and paused intervals are not charged. Clock resets or an external
remaining-time correction discard an old fractional remainder. The clock stops
at zero and records expiry, without automatically changing half or game phase.
Ball exits trigger goals or set plays and automatic ball placement. Motion in SET
and STOP, illegal positioning and leaving the field trigger player penalties
with simulation-time expiry.
Rules can explicitly request robot and ball teleports through the simulator command adapter.
After each completed physics step, the simulator publishes `/simulation/step`.
The AutoRef calls `rules/check_rules.py:RuleChecker.check_rules()` for each received
update, in every game phase and independently of `use_sim_time`. It updates the
playing clock, observes contacts, and checks motion penalties. A changed
return value is validated, stored and immediately sent through the UDP adapter.
The league and lineup mode determine the per-team player limit transmitted to the
robot; they do not yet place or spawn robots in the simulator.

## Build and launch

Use the repository's supported Linux Pixi environment. Build the new package
and its workspace dependencies:

```sh
pixi run -e default build --packages-up-to bitbots_auto_referee bitbots_mujoco_sim
```

This rebuilds the shared `bitbots_msgs` interfaces and both consumers. Restart
both the simulator and AutoRef after building: `/simulation/step` now uses
`bitbots_msgs/SimulationState` instead of the former clock-only message.

Start the referee independently of the already running simulation:

```sh
pixi run -e default ros2 launch bitbots_auto_referee auto_referee.launch.py
```

List all arguments and their defaults:

```sh
pixi run -e default ros2 launch bitbots_auto_referee auto_referee.launch.py --show-args
```

For a configured session, supply the values from your robot setup:

```sh
pixi run -e default ros2 launch bitbots_auto_referee auto_referee.launch.py \
  leagueSize:="$LEAGUE_SIZE" lineup_mode:="$LINEUP_MODE" \
  home_team_id:="$HOME_TEAM_ID" away_team_id:="$AWAY_TEAM_ID" \
  home_color:="$HOME_JERSEY_COLOR" away_color:="$AWAY_JERSEY_COLOR"
```

`config.py` is the canonical source of argument defaults and accepted named
values. Launch arguments and node parameter declarations share that source.
`leagueSize` intentionally preserves the requested spelling. It controls the
protocol competition category and, together with `lineup_mode`, the per-team
player limit. The canonical mapping is `PLAYERS_PER_TEAM` in `config.py`.
`players_per_team` is a derived property, not a launch or ROS parameter.
The loaded simulator field and robot population are unchanged.
Home is assigned the initial kick-off, and the first permitted player slot is
designated goalkeeper. Initially, all permitted slots are unpenalized; these are
protocol entries, not a claim that robots are connected or present on the field.
A later match snapshot may supply fewer player entries for either team, including
an empty team. Only a count above the match limit is rejected. Entries remain
ordered by player number; omitted trailing slots are transmitted as substitutes.
An empty team may omit its goalkeeper designation. The packet's `players_per_team`
always remains the match limit, irrespective of supplied entries or robot replies.
Team message budgets initially remain empty until a rule engine manages them.

| Parameters | Purpose |
| --- | --- |
| `leagueSize` | Competition size; combined with lineup mode determines the per-team player limit |
| `lineup_mode` | `foundation` or `advanced`; combined with league size determines the per-team player limit |
| `home_team_id`, `away_team_id` | Distinct protocol team identities |
| `robot_team_mapping` | JSON object mapping simulator robot indices to `home` or `away` |
| `robot_player_mapping` | Optional simulator-index to protocol player-number overrides |
| `penalty_area_length` | Penalty area depth used for motion-in-stop placement |
| `home_color`, `away_color` | Distinct field-player jersey colors |
| `home_goalkeeper_color`, `away_goalkeeper_color` | Independently selected goalkeeper colors |
| `target_host`, `target_port` | IPv4 unicast receiver endpoint |
| `bind_host`, `return_port` | Local interface and return packet port |
| `send_rate`, `response_timeout` | Wall-clock heartbeat and connection timeout |
| `use_sim_time` | Simulator clock for the opening sequence and referee decisions |
| `ui_enabled` | Launch the native read-only application alongside the referee |

Match and network settings are startup-only ROS parameters. Restart the process
to change them. Starting the AutoRef automatically prepares and starts play, which
can activate the connected robot's behavior when it receives the state changes.
`use_sim_time` is enabled by the launch file: preparation starts with the first
available nonzero simulator-clock sample, independently of the simulator's
absolute uptime. Pausing simulation pauses preparation. With `use_sim_time`
disabled, preparation uses monotonic wall time from node initialization.
A backward clock jump during preparation restarts the opening sequence. After
PLAYING is reached, the opening sequence stops modifying the match state.
The playing clock always uses the simulation timestamps in the observation
stream, even if the opening sequence uses wall time.

## Native read-only application

The launch file starts `auto_referee_ui` in its own process. Like the positioning
capsule's `debug_positioning.py`, it uses Matplotlib with a native Qt window.
The required Matplotlib and Qt packages are already part of the workspace's Pixi
environment. A graphical desktop session is required for the window.

The application displays team scores, match and game phases, remaining time,
stopped status, preparation time, kicking team, the last ball-contact team and
robot connectivity. The most recent decisions and events are shown below the
scoreboard. Long entries wrap; entries that do not fit in the current window
are indicated as older events outside the view. A bounded history is retained
by the referee and included in each snapshot. Ordinary clock ticks update the
display without flooding that history.

There are no match controls, sliders or plot toolbar. Closing the window ends
only the UI process; the referee continues. The application subscribes to the
relative ROS topic `dashboard`, which resolves to `/auto_referee/dashboard` under
the supplied launch file. Full snapshots include state and event history so a
newly opened window immediately receives the latest retained snapshot. Only the
referee publishes this topic; the UI never commands robots or changes match state.

For headless operation, launch with `ui_enabled:=false`. The referee keeps
publishing snapshots, so the window can be opened separately later:

```sh
pixi run -e default ros2 run bitbots_auto_referee auto_referee_ui \
  --ros-args -r __ns:=/auto_referee
```

Use the same ROS domain and namespace as the running referee. A missing or stale
snapshot is indicated without extrapolating the game clock. If simulation time
stops advancing, the display reports that it is waiting for simulation progress.
The former web server, HTML page and `ui_host`/`ui_port` parameters were removed.

## Robot receiver configuration

Use the existing `game_controller_hsl` receiver in the robot's ROS namespace.
Its `team_id` must match either configured team, and its `bot_id` must be inside
the permitted player-number range. Its `listen_port` must match `target_port`; `answer_port` must
match `return_port`. Its listen interface must accept the configured target
address. Match the receiver's simulation-clock setting to the robot stack.

The default network configuration is restricted to the local machine. For a
receiver on another host, select its target address and a reachable local bind
interface. The receiver sends replies to the source IP of the received packet
at its configured answer port. Only the referee binds that return port; a bind
conflict is an explicit startup failure rather than silently sharing replies
with another GameController.

This implementation targets a single receiver. Robot replies are indexed by
team and player identity, but sending to multiple receivers or broadcasting is
not enabled yet. Valid replies update `adapter.latest_responses`; connection
loss and recovery are logged. There is no ROS status publisher yet. Wire lengths
are converted to meters, while yaw remains in radians. The protocol's team-frame
robot pose and robot-relative ball frame are retained; no world-frame transform
is inferred.

No minimum number of robot replies is required to start or continue sending.
Either team may have fewer connected robots than the match limit, including
none. Never receiving a reply is reported as informational; a previously observed
connection timing out is reported without stopping the referee or changing the
match state. Connection status does not establish whether a robot is on the field;
that will require simulator observations.

UDP timers and reply timestamps use monotonic wall time, so heartbeat packets
continue during simulation pause or before the first simulator clock message.
Robot-reported observations do not provide referee ground truth.

## Architecture and extension points

```text
bitbots_auto_referee/
  config.py                 Shared launch defaults and startup validation
  node.py                   Process lifecycle and component composition
  core/state.py             Immutable complete match state
  adapters/game_controller.py  Wire encoding, UDP transport and robot replies
  adapters/simulation/      ROS simulation snapshot conversion
  core/observations.py      Copied world-frame positions and contacts
  events/                   Reserved for observation-to-event detectors
  rules/startup.py           Clock-driven opening sequence
  rules/check_rules.py       Rule evaluation entry point for simulation updates
  ui/dashboard.py           ROS state and event snapshot publisher
  ui/application.py         Native Matplotlib/Qt application
launch/
  auto_referee.launch.py     Central entry point for referee components
test/                       Offline regression tests
```

Future rules should create a new immutable `MatchState` and call
`GameControllerUDPAdapter.set_state()` on the owning executor thread. Validation
occurs before replacing the current snapshot. Socket operations are nonblocking;
receive work is bounded so a stream of packets cannot monopolize the executor.

Because the simulator is a separate process, its adapter consumes timestamped ROS
observations instead of accessing MuJoCo memory. Robot root and ball positions are
copied from free-joint coordinates after each physics step. Contacts come from
the completed physics solve, with ball geometry and robot ancestry resolved from
the MuJoCo model. Multiple contact points and robot links collapse into a per-robot
touch flag. Only touching, solver-active contacts are included, not proximity
contacts. AutoRef teleports are acknowledged and marked in the observation stream
to suppress artificial ball-contact callbacks. Other repositioning mechanisms,
including viewer dragging, do not yet carry these markers.

The simulation-step signal carries the timestamp, step counter, ball presence,
ball position, robot indices, root positions and ball-contact flags. It uses reliable, volatile KEEP_ALL QoS
on publisher and subscriber so pending updates are not deliberately replaced by
newer samples. Delivery remains subject to middleware resource limits. Evaluation
is asynchronous and does not wait for a referee response before the next physics
step. A late-joining referee processes future updates, not the simulation history;
start the AutoRef before the simulator to observe the opening step. The separate
`/clock` topic keeps its existing clock-synchronization purpose and settings.

### Rule checker state and ball contacts

`AutoReferee.rule_checker` owns the current observations as instance attributes:

- `robot_positions`: simulation robot index to world-frame position tuple.
- `ball_position`: world-frame position tuple, or `None` if the model has no ball.
- `simulation_time_ns` and `step_number`: the latest processed simulation step.
- `last_touch_team_id`: team passed to the most recent ball-contact callback.

Positions use meters and are replaced before callbacks execute. Removed robots
do not leave stale entries. Keeping attributes on each instance prevents separate
referees from sharing mutable state.

`on_robot_ball_contact(team_id)` runs when a robot starts touching the ball.
A continuing contact does not retrigger it; release followed by renewed contact
does. Simultaneous touching robots each receive a callback, ordered by simulator
index for determinism. That ordering does not establish physical precedence;
the stored last team reflects callback order for simultaneous touches. The default
handler records the team and does not yet change scores or penalties.

`robot_team_mapping` supplies explicit simulator-index assignments to `home` or
`away`; those names resolve to the configured team IDs. The default maps the first
simulated robot to home. Use the default in `config.py` as the JSON template for
additional assignments. The mapping does not create robots or imply that they
are connected; empty mappings and partial teams are allowed, up to the team limit.
Positions of unmapped robots are still recorded, but their contact callbacks are
skipped with a warning instead of guessing a team. A backwards timestamp or step
counter clears contact history when the simulator restarts.

Future event and rule components should be composed in `node.py`; components
requiring separate processes should be added to the central launch file. Empty
placeholder nodes are deliberately not launched.

### Teleports from rules

Inside `RuleChecker.check_rules()`, call:

```python
robot_result = self.teleport_robot(robot_id, x, y, yaw)
ball_result = self.teleport_ball(x, y, yaw)
```

`robot_id` is the simulator index used by `robot_positions`, not a team or
GameController player number. Coordinates are absolute world-frame meters;
`yaw` is in radians. Height and articulated joint positions are preserved, the
root is oriented upright with the requested yaw, and velocities of the target
are cleared. The ball's yaw controls its orientation, including its texture.

Both calls return a future containing `TeleportResult` with `success`, `message`
and `applied_step`. Do not block on `result()` inside a rule callback. Check
`done()` on subsequent updates before reading the result, and issue each command
once per decision rather than on every simulation update. Cancelling the returned
future does not cancel an already submitted teleport.

The `/simulation/teleport` service queues the command and acknowledges actual
application before physics in the simulation thread. While simulation is paused,
an accepted request waits until stepping resumes. An unavailable service or an
unknown target yields a failure; there is no automatic retry. The dashboard logs
execution results. Cached rule-checker positions update from subsequent simulation
observations, not optimistically when submitting a command.

Rebuild `bitbots_msgs` and both consumers and restart both processes after this
interface change. Manual viewer dragging can subsequently override placement;
teleports do not disable ongoing robot control or freeze the target in place.

## Review and tests

The offline tests cover independent wire fixtures, parameter validation,
packet rejection, counter wraparound and socket cleanup without real networking.
After building, they can be discovered with the standard-library test runner
inside Pixi:

```sh
pixi run -e default python -m unittest discover \
  -s src/bitbots_simulation/bitbots_auto_referee/test
```

Their presence does not imply they have been executed; see the accompanying
change report for validation performed.

## Ball exits and restarts

The referee detects complete ball exits over the outer edge of the field markings
while PLAYING and not stopped. It interpolates the first crossing between consecutive
observations, including height for a goal, and freezes the decision and last-touch
team at that instant. The decision delay and set-play duration are defined in
`rules/outside.py` and use simulation time. A ball already outside on the first
observation, or moved outside by teleport, does not create a fictitious crossing.

Goals increment the scoring team's score, give kickoff to the conceding team,
and place the ball at the center before repeating READY, SET and PLAYING using
the opening sequence's phase durations. Throw-ins go to the team opposite the
last touch, corners to the attacking team after a defending touch, and goal kicks
to the defending team after an attacking touch. Throw-ins use the crossing's
nearest touchline point; corners and goal kicks use the corresponding field or
goal-area corner. Unknown last-touch teams are reported without inventing a restart.

The match is stopped while waiting for ball placement acknowledgement and its
corresponding simulation observation. A placement failure leaves it stopped and
records an event. After successful placement, set plays remain PLAYING with
`stopped=false`, the awarded `kicking_team`, and a simulation-driven `secondary_time`.
The set play clears on expiry or on a new ball contact by the awarded team after
acknowledged placement. Opponent contacts and teleport contacts do not end it.
Further boundary crossings are still detected during a set play. Goals and set
plays are recorded in the native dashboard's decision history.

Geometry is explicitly configurable through launch arguments `field_length`,
`field_width`, `line_width`, `goal_width`, `goal_height`, `goal_area_length`,
`goal_area_width`, `penalty_area_length`, `penalty_area_width`,
`center_circle_radius`, and `ball_radius`, all in metres. Length and width refer to
line centers; goal dimensions refer to the clear opening. Defaults match the
current MuJoCo kid field and ball. These dimensions must match the loaded simulator
scene; `leagueSize` does not resize that scene or choose a different geometry.
`home_defends_negative_x` sets the home goal side in the first half. By default,
Home defends positive X and Away negative X; the sides reverse when `first_half`
changes. Robot placement remains unchanged.

The simulator uses the regular ROS executor so the asynchronous teleport service
can await completion by the physics thread without interrupting ROS processing.
Referee teleports release any active Viser drag for that object. Successful ball
placement is logged with the resumed state: set plays resume PLAYING immediately,
while goals enter their READY/SET preparation. Restart both simulator and referee
after rebuilding these changes. A manually paused viewer must be resumed in the
viewer before queued teleports can be applied.

## Motion penalties

`rules/motion.py` defines the grace interval, sustained-motion filter, numerical
motion thresholds and penalty durations. The referee evaluates each observed robot.
SET allows head motion and getting up, but penalizes other sustained motion.
STOP also checks the head and takes precedence over SET. Each mode transition,
newly observed robot, penalty expiry and robot teleport grants a settling interval.
Brief solver jitter does not immediately cause a penalty.

Getting up is approximated from torso inclination and height relative to the
model's initial pose. A tilted or low torso is exempt during SET, with a settling
interval after it becomes upright. This is a ground-truth heuristic, not an
animation classifier; crawling while low can also satisfy that exemption.
The simulator reports root linear/angular speed and maximum head/body joint speed.
Head hinges are identified by the model joint names containing `head`. Missing
or nonfinite motion telemetry is not interpreted as a movement violation.

Penalties apply to protocol players, not simulator indices. Within each team,
ascending configured robot indices map to ascending player numbers by default.
Use `robot_player_mapping` to override that mapping to match each receiver's
`player_id`. Numbers must be unique within their team and fit the roster.
Existing penalties are not overwritten. Only penalties issued by the motion or position
rules are cleared by their shared deadlines; repeated motion does not extend them.
Deadlines use simulation time even when the match clock is stopped. A simulation
clock reset clears owned motion penalties and restarts observation grace periods.

Motion in STOP teleports the robot to the nearest touchline at the front edge of
its own penalty area, facing toward the field center. Team side and half determine
the own-goal direction. Teleport failure is logged without clearing the penalty.
Expiry releases the penalty without teleporting the robot back onto the field.

`SimulationRobotState` now includes motion telemetry. Rebuild `bitbots_msgs` and
all consumers with the documented `--packages-up-to` build command, and restart
simulator and referee together; old and new generated messages are incompatible.

## Robot position rules

`robot_position_rules.check` runs after motion rules on each simulation update.
It uses world-aligned horizontal bounds covering the robot's collision geometry,
including limbs, rather than just the torso center. These bounds are conservative:
they can include empty space between limbs and around rotated shapes. Missing or
invalid bounds skip position decisions for that robot. The extended
`SimulationRobotState` requires rebuilding the shared messages and both consumers.

In SET, the complete robot must be in its own half and inside the field. The
kicking team may have one robot touching or intersecting the center circle,
including its marking, even when the rest of that robot crosses the halfway line.
If several candidates enter together, the closest torso to the center is selected,
with simulator index as tie breaker; the selection remains until that robot leaves
the circle or is no longer eligible. Other attacking robots in the circle are illegal.

During an active, non-stopped PLAYING set play, defenders must keep the center-circle
diameter as distance between their body bounds and the ball center. Moving away,
or moving tangentially without appreciable inward progress, is exempt. Standing
still and approaching are violations after a short continuous grace interval.
Robot displacement is measured in world coordinates so ball movement alone does
not count as retreat. For goal kicks, defenders must also be completely outside
the opponent penalty area, including the markings; this requirement remains even
while moving away from the ball. SET positioning also uses a continuous grace
interval to avoid penalties on a single observation.

Leaving-the-field detection runs in every phase once the complete bounds are
beyond `LEAVING_MARGIN` from the outer field markings. At corners it uses planar
distance to the field rectangle. A teleport observation itself is exempt; an
illegal position persisting afterward is evaluated normally.

Both rules use their protocol penalty names, the durations in
`rules/motion.py:PENALTY_SECONDS`, and the same own-penalty-area touchline placement
as motion in STOP. Existing player penalties are preserved and no duplicate
placement is requested. Deadlines use simulation time, and clock resets clear
owned penalties and position history. The center-circle radius and penalty-area
width are configurable launch parameters and must match the simulator field.

## Game flow rules

`game_flow_rules.check` runs after movement and position checks. Its durations,
movement tolerance and recovery thresholds are defined in `rules/game_flow.py`.
Local Gamestuck, Incapable Robot and Ball Holding share the existing penalty
expiry and sideline-placement mechanism; existing penalties are never replaced.

Stuck-play timers run only in non-stopped PLAYING without a set play. Meaningful
ball displacement resets them; small physics jitter does not. This deliberately
uses observed movement rather than assuming every touch moved the ball. Any real
movement is treated as progress, including movement with unknown attribution.
Ball teleports and clock resets discard old timers. Local Gamestuck additionally
requires an eligible, upright robot close to and facing the ball for the local
interval. On expiry, the nearest currently eligible robot able to play the ball
is penalized. This ability check is a pose heuristic, not a behavioral intention
signal from the robot.

Global Gamestuck starts a neutral restart with the protocol's no-team kickoff
value. The first observed pose and heading of each robot are retained as its
initial placement. Starts inside the field are projected to the nearest touchline;
existing sideline spawns are preserved. Robots currently present are returned to
those placements and the ball is placed at center. The referee waits for all
service acknowledgements and corresponding observations before READY, SET and
PLAYING. Preparation uses the existing phase durations. Scores and remaining match
time are preserved, as are penalties, whose normal simulation-time expiry continues.
Missing initial headings or failed teleports leave the restart stopped with a
logged reason. A simulator reset re-captures initial placements and retries an
interrupted global restart.

Incapable Robot tracks a fallen torso by inclination and relative height. A
partial rise followed by another fall counts as a failed attempt; stable upright
posture clears the history. Sustained failure to regain upright posture or repeated
failed attempts triggers the penalty. This is a hysteresis-based posture heuristic,
not an animation-state classifier. Teleported or already penalized robots do not
accumulate recovery failures.

Ball Holding is intentionally conservative. Only an upright robot enclosing the
ball's horizontal location is considered. The simulator samples horizontal rays
from the ball and reports how many hit that robot's collision geometry nearby.
Only near-complete, sustained enclosure of an essentially stationary ball during
free play qualifies. Incidental contacts, partial coverage and lying on the ball
are not holding violations. The ray test is a geometric approximation of blocked
access, not proof that another robot could or could not execute a specific kick.

Robot headings and ball blockage are new `SimulationRobotState` fields. Rebuild
the message package, simulator and referee together and restart both processes.

## Pushing

`pushing_check` runs after the game-flow rules. The simulator aggregates the force
magnitudes of active robot-to-robot contacts per robot pair and reports contact-point
approach speeds. The actor is inferred from one-sided approach, with observed root
displacement as an additional signal. Equal-and-opposite contact forces alone cannot
identify an actor. Ambiguous contacts without an identifiable initiator are not
penalized. An established initiator is retained through continuous pressure.

A sufficiently strong contact must also visibly destabilize a previously upright
victim: decreased upright projection, a fall in relative height, or increased
angular speed within the configured effect window. Weaker contact must persist
continuously for `SUSTAINED_NS` in `rules/pushing.py`. Contact gaps reset that timer.
Mutual approach and an upright, ball-facing duel with the ball centrally between
both robots are exempt. Teleports and simulation resets discard old episodes.
This remains a tunable physical heuristic, not proof of intent or causality.

Violations issue `PENALTY_PUSHING` using the shared penalty duration and sideline
placement. Existing penalties are preserved. Decision events include the actor,
victim, reason and peak force; current pair forces are also exposed in
`rule_checker.pushing_rules.contact_forces`. The complete force and approach
measurements are carried in `/simulation/step` for tuning even without a penalty.

All tuning arguments are declared and validated in `config.py` and passed through
by the launch script:

| Parameter | Meaning |
| --- | --- |
| `pushing_force_threshold` | Minimum force for a destabilizing single push, in newtons |
| `pushing_sustained_force_threshold` | Minimum force counted as sustained pressure, in newtons |
| `pushing_approach_speed` | Actor attribution threshold, in metres per second |
| `pushing_tilt_drop` | Minimum decrease in victim upright projection |
| `pushing_angular_speed` | Minimum increase in victim angular speed, in radians per second |
| `pushing_effect_window` | Time allowed between strong contact and destabilization |
| `pushing_ball_center_tolerance` | Allowed ball offset from the robot-pair midpoint |
| `pushing_ball_reach` | Maximum distance from each robot to the ball for the duel exemption |

These defaults are initial tuning values, not calibrated referee thresholds.
`SimulationRobotContact` is a new message embedded in `SimulationState`; rebuild
`bitbots_msgs` and both consumers, then restart simulator and referee together.

## Direct and indirect free kicks

`RuleChecker.award_direct_free_kick(state, team_id, x, y)` and
`award_indirect_free_kick(state, team_id, x, y)` award a restart during active
PLAYING and return the updated match snapshot. Callers must apply that returned
state. Both use the supplied incident location without relocating it to a field
marking, set `kicking_team`, and initialize `secondary_time` to the shared set-play
duration. They stop play while awaiting acknowledged ball placement, then resume
PLAYING and count down. The awarded team's first new ball contact ends `set_play`
and `secondary_time` as for the other set plays. Invalid teams/coordinates or an
outstanding placement are rejected. Double-touch detection uses this interface to award the opponent an indirect free kick at the ball position.

Pushing during active play awards a direct free kick to the victim's team in
addition to penalizing the actor. The simulator reports the strongest contact's
world location per pair; the rule retains the latest incident location even if
destabilization is detected after contact ends. If contact location is unavailable,
the pair midpoint is used as a fallback. Fouls detected outside active PLAYING
still incur their player penalty but do not interrupt preparation with a free kick.
If several fouls are detected in one update, the first recorded foul determines
the restart while all corresponding player penalties remain applied.

Indirect free kicks and throw-ins arm a separate goal restriction after placement.
The first touching robot from the awarded team is remembered by simulator index.
A second contact onset removes the restriction, including a renewed touch by the
same robot after releasing the ball. Continuous contact counts only once; contacts
by another robot count regardless of its team. Teleport contacts are excluded. Ending or timing out `set_play` does not remove the restriction.
Before it is lifted, a ball entering the opponent goal produces a goal kick;
a ball entering the restarting team's own goal produces a corner for the opponent.
Neither increments the score. The decision is captured at the goal-line crossing,
so later contacts during the outside-decision delay cannot turn it into a goal.

A new restart replaces this restriction; direct free kicks, goal kicks, corners
and kickoffs permit direct goals through the existing goal logic. Simulator resets,
external ball teleports and neutral global restarts discard stale restrictions.
The contact-location extension changes `SimulationRobotContact`: rebuild the shared
messages and both consumers and restart simulator and referee together.


### Double touch after restarts

All set plays and the transition from SET to PLAYING for a team kickoff arm the
restart taker's double-touch restriction. It persists after the set-play timer
ends, until another robot makes a significant contact or a new restart begins.
A repeated significant contact by the taker awards the opposing team an indirect
free kick at the current ball position, using the standard restart countdown.
No additional player penalty is imposed.

At each potential violation, the rule counts currently observed, unpenalized
players separately for each team. Only players on the field count; full robot
bounds are used when available, otherwise the root position must be inside the
field. Duplicate player slots do not increase the count. The executing team must
meet `MIN_ACTIVE_PLAYERS` in `rules/double_touch.py`; the configured roster size
and the opponent's player count do not determine eligibility.

Significant contact episodes must satisfy both `double_touch_min_force` and
`double_touch_min_impulse`. MuJoCo publishes summed robot-ball contact forces;
the referee integrates these over simulation time. `double_touch_release_time`
separates distinct episodes, avoiding repeated triggers from continuous contact
or short contact gaps. Teleport and stopped-play contacts are excluded. Without
force telemetry, contacts cannot qualify as significant. These thresholds are
ROS parameters and may require tuning for the robot model.

The indirect goal restriction still permits a renewed touch by the same robot,
but the double-touch rule takes precedence whenever its eligibility and
significance conditions are met. Rebuild the shared messages, simulator and
referee together before restarting them after this interface change.
