# Single J3 driver source

Snapshot taken on 2026-10-10 from the existing J3 driver working tree, based on
commit `57cbb14385479bf12254e0becb267a6edafa0598` plus its local single-axis changes.
The source was copied; the original deployment directory was not modified.

This repository builds only M18, EtherCAT/SOEM, CANopen, shared memory, utilities,
and the J3 CLI. J3 uses slave index 0, node 1, passages TX/RX/CFG 1/2/3 and shared
memory slot 3. The 30-slot, 480-byte ABI and existing guarded enable/move/disable
logic are retained. Direct arbitrary-node debug commands and other applications
were removed. The only network override is `JC_ETHERCAT_IFACE` (default `enp2s0`).

SOEM and cpp-readline licenses are retained in their respective `thirdparty`
directories. Both are built as static libraries; GNU readline comes from the
system. Build and operating instructions are in the repository's `USAGE.md`.
