"""Tick sensor — time heartbeat for chat-mode autonomy.

Stateless: no parameters, no resource writes, no state across runs. Each
scheduled invocation just returns a marker that SensorRunner forwards to
the agent's sense_data channel as a `sensor:tick` source. The chat loop
dispatches on the source name and runs `_check_and_fire_concerns` against
the current wall clock — the sensor itself does NOT advance any internal
clock.
"""


def run(context):
    return {'status': 'ok', 'content': 'tick', 'metadata': {}}
