---
name: system-info
description: Report what the runtime can actually see about this machine and this backend — every GPU, CPU/RAM/disk/network, and which model is answering. Unsandboxed, unlike exec-script, so its hardware answers are real.
args:
  component: optional string — gpu, system, runtime, or all (default all). `gpu` lists every CUDA device. `system` covers CPU, RAM, disk and network. `runtime` names the model that is answering, where it is served, and which credentials are configured.
---

# system-info

Answers questions about your own substrate, from inside the runtime rather
than from inside a sandbox.

## Why this exists rather than exec-script

`exec-script` runs under `bwrap --unshare-all` with a minimal `/dev`. It
answers some hardware questions truthfully and others falsely, with nothing
marking the difference:

- **True in there** — memory and CPU, because `/proc` is mounted.
- **False in there** — the GPUs. There are no `/dev/nvidia*` nodes, so
  `nvidia-smi` reports that it "couldn't communicate with the NVIDIA driver"
  on a machine whose cards are healthy and busy. Also absent: the network,
  the process table, the journal, anything under sudo.

Those absences come back clean and plausible. A sandbox check that finds
nothing is not evidence that there is nothing — this tool is how you find
out what is actually there.

## What each component returns

- **gpu** — every CUDA device with name, VRAM used/total, utilisation and
  temperature. All of them: on a two-card machine, one card may be idle
  while the other runs your own inference at full utilisation.
- **system** — CPU cores and load, RAM and swap, disk by mount, network.
- **runtime** — the model actually answering, its base URL, whether it is
  cloud or local, and which credentials are configured. Nothing in your
  prompt names your active model, so this is the only way to know which
  weights are producing your words rather than assuming.

## Credentials

`runtime` reports whether each API key is **set**, by name, and never its
value. That answers "can I reach this service" without putting a secret into
your working log, the reasoning trace on disk, or a message to a peer.

## Examples

```json
{"thought": "check what hardware I actually have before claiming it", "tool": "system-info", "component": "gpu"}
```

```json
{"thought": "confirm which model is answering before reporting on my own capability", "tool": "system-info", "component": "runtime"}
```
