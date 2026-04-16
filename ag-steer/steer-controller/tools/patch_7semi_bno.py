from pathlib import Path

Import("env")


def patch_7semi_bno(source=None, target=None, env=None):
    pioenv = env.subst("$PIOENV")
    libdeps_dir = (
        Path(env.subst("$PROJECT_DIR"))
        / ".pio"
        / "libdeps"
        / pioenv
    )
    candidates = list(libdeps_dir.glob("*/src/BnoSPIBus.h"))
    if not candidates:
        print(f"7Semi BNO patch skipped: no BnoSPIBus.h found in {libdeps_dir}")
        return
    lib_file = candidates[0]

    original = lib_file.read_text(encoding="utf-8")
    patched = original.replace("    delay(3);\n", "    delayMicroseconds(1000);\n")

    if patched != original:
        lib_file.write_text(patched, encoding="utf-8")
        print("7Semi BNO patch applied: rx post-read delay 3ms -> 1000us")


patch_7semi_bno(env=env)
