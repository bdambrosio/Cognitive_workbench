# src/tools_jill — tools Jill writes

Jill writes her own tools here, through exec-script, which can write to this
directory and nowhere else in the repo except her scratch directory. Each
tool is a directory like those in `src/tools/`: a `Skill.md` with a `name`,
a `description` and `args`, and a `tool.py` with `react_invoke`.

The chat loop scans this directory once at startup (`tool_dirs` in
`scenarios/jill-chat.yaml`), after `src/tools/`. A name already used in
`src/tools/` is not replaced. So a new or changed tool here takes effect at
the next restart of her process.

**Before that restart, Bruce reads the diff** (`git status src/tools_jill`,
`git diff src/tools_jill`). A tool here runs in the chat process like any
other tool, with its API keys, network and file access; the sandbox applies
to exec-script's scripts, not to these tools. The read is for mistakes, such
as a tool that prints the environment or makes a network call it was not
meant to make. Agreed by Bruce and Jill, 2026-10-23.
