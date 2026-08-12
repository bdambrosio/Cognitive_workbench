"""The two probes that extend Sentinel past host posture: LAN membership
(baseline_diff arp) and egress (system_state connections).

Both exist because every other probe in the module answers "what can reach
this machine". These answer "who else is here" and "where is this machine
talking to", which is where a compromise shows itself.
"""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from chat import security as S  # noqa: E402


# ---------------------------------------------------------------- arp

_NEIGH = """\
192.168.68.63 dev wlp68s0 lladdr 94:dd:f8:11:e4:48 STALE
192.168.68.63 dev enp69s0 lladdr 94:DD:F8:11:E4:48 REACHABLE
192.168.68.1 dev enp69s0 lladdr 60:83:e7:71:d1:ec REACHABLE
192.168.68.99 dev enp69s0  FAILED
fe80::96dd:f8ff:fe11:e448 dev wlp68s0 lladdr 94:dd:f8:11:e4:48 DELAY
"""


def test_arp_baseline_reduces_to_device_identity():
    """Keyed by MAC because everything else churns: DHCP moves the IP,
    the state cycles REACHABLE/STALE/DELAY every few minutes, and a
    dual-homed host lists the same device once per interface. Any of the
    three would diff on every run and bury a real arrival."""
    out = S._canonicalize_baseline('arp', _NEIGH)
    assert out == ['60:83:e7:71:d1:ec', '94:dd:f8:11:e4:48'], out
    # One physical device seen on two interfaces and over IPv6 is one entry.
    assert len(out) == 2
    # An unresolved probe is not a device.
    assert not any('192.168.68.99' in x for x in out)


def test_arp_baseline_is_case_insensitive_on_mac():
    """ip prints lowercase, but a snapshot written by another tool or an
    older release must not read as a whole new device."""
    assert S._canonicalize_baseline('arp', "1.1.1.1 dev x lladdr AA:BB:CC:DD:EE:FF STALE\n") \
        == ['aa:bb:cc:dd:ee:ff']


# -------------------------------------------------------- connections

_SS = """\
Netid Recv-Q Send-Q Local Address:Port Peer Address:Port Process
tcp   0      0      192.168.68.76:50001 140.82.1.4:443   users:(("chrome",pid=1,fd=1))
tcp   0      0      192.168.68.76:50002 140.82.1.5:443   users:(("chrome",pid=1,fd=2))
tcp   0      0      192.168.68.76:50003 140.82.1.6:443   users:(("chrome",pid=1,fd=3))
tcp   0      0      192.168.68.76:50004 8.8.8.8:443      users:(("python3",pid=2,fd=1))
tcp   0      0      192.168.68.76:50005 8.8.8.8:443      users:(("python3",pid=2,fd=2))
tcp   0      0      127.0.0.1:50006     127.0.0.1:5000   users:(("python3",pid=2,fd=3))
"""


def _stub_ss(monkeypatch, text):
    monkeypatch.setattr(S, "_run_argv", lambda *a, **k: (text, None))


def test_connections_groups_by_process_and_counts_duplicates(monkeypatch):
    _stub_ss(monkeypatch, _SS)
    monkeypatch.setattr(S, "_CONNECTIONS_FANOUT", 12)
    out = S._tool_system_state('connections')
    assert out.startswith('OK:')
    assert 'python3:' in out and 'chrome:' in out
    assert '8.8.8.8:443' in out and 'x2' in out       # two sockets, one peer
    # Loopback is expected on this host and is counted, not listed.
    assert '127.0.0.1:5000' not in out
    assert '1 loopback connection(s) not listed' in out


def test_high_fanout_process_is_summarised_not_listed(monkeypatch):
    """A browser holds sockets to a hundred hosts. Listing them truncates
    the observation and buries the process that has three peers and
    should not."""
    _stub_ss(monkeypatch, _SS)
    monkeypatch.setattr(S, "_CONNECTIONS_FANOUT", 2)
    out = S._tool_system_state('connections')
    assert '3 distinct peers' in out and 'not listed' in out
    assert '140.82.1.4:443' not in out          # the fan-out itself is gone
    assert '8.8.8.8:443' in out                 # the reviewable one survives


def test_no_off_box_connections_is_empty_not_error(monkeypatch):
    _stub_ss(monkeypatch, "Netid Recv-Q Send-Q Local Peer Process\n"
                          "tcp 0 0 127.0.0.1:1 127.0.0.1:5000 users:((\"x\",pid=1,fd=1))\n")
    assert S._tool_system_state('connections').startswith('EMPTY:')


def test_unknown_category_names_connections_as_valid():
    assert 'connections' in S._tool_system_state('nope')
