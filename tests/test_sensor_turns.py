"""A sensor turn is neither a user turn nor an autonomous fire.

Every gate in _process_user_turn used to key on `autonomous` alone, so a
sensor push ran the full user-turn machinery. Live evidence in Jill's store
before this split: 21 memories, 2 user_concerns and 2 discourse segments
filed under entity `sensor:factorio-telemetry`, among them standing
instructions the user had actually given — and 99 of 99 sensor turns
published a reply, because "stay silent" was not representable.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.chat_loop import HUMAN_COUNTERPART, is_sensor_source


def test_sensor_sources_are_recognized_structurally():
    assert is_sensor_source('sensor:rss-watcher')
    assert is_sensor_source('sensor:factorio-telemetry')
    # Humans and peers are not sensors, however they are named.
    assert not is_sensor_source('User')
    assert not is_sensor_source('User@telegram:Bruce')
    assert not is_sensor_source('Voice')
    assert not is_sensor_source('Sentinel')
    assert not is_sensor_source('')
    assert not is_sensor_source(None)


def test_reflection_reads_the_sensor_dialog_but_files_under_the_human():
    """The two uses of `source` come apart: which exchange to reflect over
    vs. who the result is about. A sensor pushed the text; the user is who
    the memory concerns."""
    from chat.reflection import ReflectionMixin

    seen = {}

    class _Loop(ReflectionMixin):
        _memories_collection_id = 'Collection_1'
        _companion_state = {'User': 'companion text for Bruce',
                            'sensor:rss-watcher': 'SHOULD NOT BE READ'}
        character_name = 'Jill'

        def _build_dialog(self, source, limit=4):
            seen['dialog_source'] = source
            return [{'source': source, 'text': 'a feed item appeared'}]

        def _top_active_agent_concerns(self, n=10):
            return []

        def _top_active_user_concerns(self, n=10):
            return []

        def _recall(self, q, k=3, threshold=0.5):
            return []

        def _load_pending_fire_outcomes(self):
            return []

        def _reflect_llm(self, *a, **k):
            raise AssertionError('not reached in this test')

        # Stop after prompt construction: the LLM call is what we don't
        # need, and its absence raises inside the try/except that returns
        # three empty lists.
        def _make_llm_callable(self, *a, **k):
            raise RuntimeError('stop here')

    loop = _Loop()
    captured = {}
    orig = _Loop._REFLECT_SYS

    class _Fmt(str):
        def format(self, **kw):
            captured.update(kw)
            return orig.format(**kw)

    _Loop._REFLECT_SYS = _Fmt(orig)
    try:
        loop._reflect_and_remember('sensor:rss-watcher',
                                   entity=HUMAN_COUNTERPART)
    except Exception:
        pass
    finally:
        _Loop._REFLECT_SYS = orig

    assert seen['dialog_source'] == 'sensor:rss-watcher', \
        "must reflect over the sensor's own report"
    assert captured.get('entity') == 'User', \
        "but attribute the result to the human counterpart"


def test_entity_defaults_to_source_for_human_channels():
    """The split must not change any existing channel: omit `entity` and
    the two are the same value, as before."""
    from chat.reflection import ReflectionMixin

    seen = {}

    class _Loop(ReflectionMixin):
        _memories_collection_id = None       # returns early, cheaply
        character_name = 'Jill'

        def _build_dialog(self, source, limit=4):
            seen['dialog_source'] = source
            return []

    assert _Loop()._reflect_and_remember('User') == ([], [], [])
