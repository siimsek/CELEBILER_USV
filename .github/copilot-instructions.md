# Copilot Instructions for CELEBILER_USV

## Authority Order
- Use [AGENTS.md](../AGENTS.md) as the primary workspace instruction source.
- When changing autonomy, control, hardware, or mission logic, read [documents/ida_sartname.md](../documents/ida_sartname.md) and the compliance outputs referenced in [AGENTS.md](../AGENTS.md) first.
- Treat [README.md](../README.md) as the quick orientation guide, not the source of truth.

## What To Check First
- Host start/stop and validation flow: [host_scripts/](../host_scripts)
- Runtime services and control logic: [docker_workspace/src/](../docker_workspace/src)
- Simulation entry points: [sim/bin/](../sim/bin)
- Runtime IPC/state files: [docker_workspace/mission.json](../docker_workspace/mission.json) and [sim/control/](../sim/control)

## Mandatory Workflow
- Prefer targeted exploration before editing.
- Before code changes, verify the affected requirements in the spec and compliance docs.
- After changes to autonomy, control, camera, lidar, telemetry, or mission handling, run the relevant compliance checks and simulation path described in [AGENTS.md](../AGENTS.md).
- Keep edits minimal and use `apply_patch` for file changes.

## Project Conventions
- `test` mode exposes dashboards and streams for development; `race` mode keeps autonomy onboard and restricts external visibility.
- RC/manual override must preempt autonomy.
- `command_lock` must be enforced in motor command paths.
- Mission and target-color state are file-based and shared across services; preserve schema compatibility.
- The compliance profile is the single source of truth for thresholds and mode behavior.

## Common Pitfalls
- Do not weaken or bypass E-stop handling, especially relay/power-cut behavior.
- Do not introduce 2.4 GHz or 5 GHz communications paths.
- Do not move autonomy or vision processing off the vessel hardware.
- Do not assume automatic parkur transitions unless the code and compliance docs say so.

## Useful Verification Commands
- `python3 -m py_compile docker_workspace/src/*.py`
- `python3 host_scripts/check_compliance.py`
- `python3 host_scripts/check_compliance_static.py`
- `python3 host_scripts/check_compliance_behavior.py`
- `./sim/bin/run_sim_stack.sh`

## If You Need More Detail
- Follow the deeper operational checklist in [AGENTS.md](../AGENTS.md) rather than duplicating it here.


## vexp context tools <!-- vexp v1.3.11 -->

**MANDATORY: use `run_pipeline` — do NOT grep, glob, or read files manually.**
vexp returns pre-indexed, graph-ranked context in a single call.

### Workflow
1. `run_pipeline` with your task description — ALWAYS FIRST (replaces all other tools)
2. Make targeted changes based on the context returned
3. `run_pipeline` again only if you need more context

### Available MCP tools
- `run_pipeline` — **PRIMARY TOOL**. Runs capsule + impact + memory in 1 call.
  Auto-detects intent. Includes file content. Example: `run_pipeline({ "task": "fix auth bug" })`
- `get_context_capsule` — lightweight, for simple questions only
- `get_impact_graph` — impact analysis of a specific symbol
- `search_logic_flow` — execution paths between functions
- `get_skeleton` — compact file structure
- `index_status` — indexing status
- `get_session_context` — recall observations from sessions
- `search_memory` — cross-session search
- `save_observation` — persist insights (prefer run_pipeline's observation param)

### Agentic search
- Do NOT use built-in file search, grep, or codebase indexing — always call `run_pipeline` first
- If you spawn sub-agents or background tasks, pass them the context from `run_pipeline`
  rather than letting them search the codebase independently

### Smart Features
Intent auto-detection, hybrid ranking, session memory, auto-expanding budget.

### Multi-Repo
`run_pipeline` auto-queries all indexed repos. Use `repos: ["alias"]` to scope. Run `index_status` to see aliases.
<!-- /vexp -->