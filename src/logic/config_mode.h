#pragma once

/// Blocking serial configuration mode REPL.
///
/// Runs until user enters `exit` (returns to caller) or `reboot`
/// (triggers MCU restart).
void configModeRun();
