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
Simulation observations, event-based decisions, placement and referee UI are not
implemented yet. The remaining half time does not count down yet.
The league and lineup mode determine the per-team player limit transmitted to the
robot; they do not yet place or spawn robots in the simulator.

## Build and launch

Use the repository's supported Linux Pixi environment. Build the new package
and its workspace dependencies:

```sh
pixi run -e default build --packages-up-to bitbots_auto_referee
```

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
| `home_color`, `away_color` | Distinct field-player jersey colors |
| `home_goalkeeper_color`, `away_goalkeeper_color` | Independently selected goalkeeper colors |
| `target_host`, `target_port` | IPv4 unicast receiver endpoint |
| `bind_host`, `return_port` | Local interface and return packet port |
| `send_rate`, `response_timeout` | Wall-clock heartbeat and connection timeout |
| `use_sim_time` | Simulator clock for the opening sequence and referee decisions |

Match and network settings are startup-only ROS parameters. Restart the process
to change them. Starting the AutoRef automatically prepares and starts play, which
can activate the connected robot's behavior when it receives the state changes.
`use_sim_time` is enabled by the launch file: preparation starts with the first
available nonzero simulator-clock sample, independently of the simulator's
absolute uptime. Pausing simulation pauses preparation. With `use_sim_time`
disabled, preparation uses monotonic wall time from node initialization.
A backward clock jump during preparation restarts the opening sequence. After
PLAYING is reached, the opening sequence stops modifying the match state.

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
  adapters/simulation/      Reserved for ROS simulation observations/commands
  events/                   Reserved for observation-to-event detectors
  rules/startup.py           Clock-driven opening sequence
  ui/                       Reserved for referee status and operator commands
launch/
  auto_referee.launch.py     Central entry point for referee components
test/                       Offline regression tests
```

Future rules should create a new immutable `MatchState` and call
`GameControllerUDPAdapter.set_state()` on the owning executor thread. Validation
occurs before replacing the current snapshot. Socket operations are nonblocking;
receive work is bounded so a stream of packets cannot monopolize the executor.

Because the simulator is a separate process, its future adapter must consume
timestamped ROS observations instead of accessing MuJoCo memory. Contacts must
be captured inside the simulator at physics-step frequency and transferred
without silently dropping events. Placement commands need execution
acknowledgements and reset identifiers to avoid false boundary-crossing events.
Those simulator interfaces are a subsequent implementation step.

Future event and rule components should be composed in `node.py`; components
requiring separate processes should be added to the central launch file. Empty
placeholder nodes are deliberately not launched.

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
