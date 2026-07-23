# Examples

Minimal, standalone DDS scripts demonstrating the read/write pattern for
both the body and hands. "Standalone" means neither imports this
repository's `config.py` - every topic name, domain ID, and network
interface is declared explicitly at the top of the file instead, the same
way a real deployment script would. That's deliberate: it means each
example doubles as a template for a real-robot control script, and
pointing either one at hardware instead of simulation is a two-line
change, nothing more:

```python
DOMAIN_ID = 0            # was 1
INTERFACE = "enp2s0"     # was "lo" - your robot's actual network interface
```

Both scripts assume `unitree_mujoco.py` is already running in a separate
terminal on the same machine when used against the simulation.

## `read_states.py`

Read-only. Subscribes to `rt/lowstate` (body) and both
`rt/brainco/{left,right}/state` (hands), and prints body joint positions,
IMU orientation, and hand position/velocity to the terminal a few times a
second. Publishes nothing, so it's always safe to run - useful as a first
check that the DDS connection is actually delivering data before writing
any controller logic against it.

## `go_to_zero_and_hands.py`

Writes commands. Reads the robot's *current* body and hand state first,
then runs a single continuous control loop for the whole script's
lifetime: the body eases from its current pose to all-zero (3 seconds),
both hands close (2 seconds, hold for 1), then reopen (2 seconds) - after
which the script keeps holding that final pose, indefinitely, until
interrupted.

The loop never stops publishing partway through, which matters more than
it might look: a command-based low-level controller (real hardware or
this simulation) only holds a pose for as long as it keeps receiving
fresh targets. Stop publishing, even briefly, and the last-known target
goes stale - on real hardware this trips a command-timeout safety cutoff
that drops torque; in this simulation, `body_bridge.py` simply stops
refreshing `ctrl` once messages stop arriving. Either way, the fix is the
same discipline this script follows: always be publishing something,
and change the target, rather than stopping and starting a new loop for
each phase of a motion sequence.

Body gains (`KP`/`KD` at the top of the script) are reused from this
project's own G1 whole-body-control configuration rather than re-derived
- retune them if that configuration has since changed.

## Running

```
python3 examples/read_states.py
python3 examples/go_to_zero_and_hands.py
```