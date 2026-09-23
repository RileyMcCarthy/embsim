# Default configuration for p2-softmmu
#
# The P2 has one board and no optional devices: hub RAM, eight cogs, and the
# pin bus. `CONFIG_P2_BOARD` is `default y` in hw/p2/Kconfig and depends on the
# target, so there is nothing to select here — but meson requires the file to
# exist.
