---
title: command_processor — Serial Command Interface
type: entity
tags: [commands, serial, interface, debugging]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/command_processor.cpp, src/command_processor.h]
---

Text-based serial command interface for controlling and diagnosing the flight computer. Operates at 115200 baud.

## Available Commands

| Command | Description |
|---------|-------------|
| `arm` | Transition to ARMED state, enabling launch detection |
| `status_sensors` | Detailed sensor health report for all connected sensors |
| `calibrate` | Manual barometric calibration (set current altitude as ground) |
| `clear_errors` | Manual recovery from ERROR state |
| `help` | List all available commands with descriptions |
| `1`–`9` | Toggle debug flag subsystem outputs (see `src/debug_flags.h`) |

## Architecture

Each command is dispatched through a `SystemStatusContext` struct that provides access to all system state without global variable coupling:

```cpp
struct SystemStatusContext {
  // All sensors, state, and HAL references
  // Passed to every command handler function
};
```

## Adding Commands

1. Add handler: `void handleMyCommand(const SystemStatusContext& ctx)`
2. Register in dispatcher: `if (strcmp(cmd, "mycommand") == 0) handleMyCommand(ctx);`
3. Add help text to `printHelpText()`

## Debug Flags

`src/debug_flags.h` controls subsystem-specific verbose output. Toggle at runtime via single-digit commands without recompile.
