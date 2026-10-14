# FIFO regression checks

These compile-time tests execute the production FIFO decoder on known 12-byte
records. They check byte order, negative values, signed extremes, gyro offsets,
record stride, and the FIFO enable mask. Constant evaluation also rejects reads
past the end of the exactly sized input arrays.

Run with any C++17 compiler, including the Pico ARM cross compiler:

```sh
arm-none-eabi-g++ -std=c++17 -Wall -Wextra -Werror -Iinclude -fsyntax-only tests/imu_fifo_test.cpp
```

A successful compiler exit means every assertion passed. This does not execute
firmware or verify I2C hardware. The test is also compiled by the normal CMake
build as `imu_fifo_regression`.

## Process host regression tests

With a host C++17 compiler and Bash, run `bash tests/run_host_tests.sh`.
The harness compiles the production `logger.cpp` and `state_machine.cpp` against
thread-safe queue, timer and GPIO substitutes in `tests/host`. It does not use
Pico hardware. The timeout is accelerated from 2000 ms to 20 ms only in the mock.

It checks source format rejection, processor compatibility, unchanged default
bytes, failed-start buffer recovery, empty/full/partial handoffs, a held sink
retaining exclusive buffer access, source starvation without silent drops,
processor failure and recycling, existing overflow reset policy, write/sync
errors, stop timeouts (including sync still in flight), and restart gating.
The fixtures intentionally live until process exit because the firmware worker
has no destruction/shutdown API. This harness does not validate SDK spinlocks,
interrupt timing, GPIO debounce, the real FIFO, or physical SD durability.

On Windows, a portable Zig compiler can also run the same test:

```powershell
zig c++ -std=c++17 -Wall -Wextra -Werror -pthread -Itests/host -Iinclude tests/process_host_test.cpp src/logger.cpp src/state_machine.cpp -o process_test.exe
.\process_test.exe
```

## Architecture after dependency injection

`Process` is the control interface; `Logger` is its two-core implementation.
`StateMachine` borrows `Process&`. `Logger` borrows `Inputs&`, `SampleProcessor&`,
`Outputs&`, and the event queue. Main owns `IMU`, `PassThroughProcessor`,
`SdStorage`, `Logger`, `Button`, and `StateMachine` for the entire firmware life.
`Button` implements `EventSource` and posts events; it knows nothing about the
state machine. The existing `Outputs` interface is the interchangeable sink.

Core 0 drains the source into one of four 6144-byte buffers. Core 1 dequeues a
buffer, processes it in place, writes it synchronously, then returns its handle.
`SampleFormat` names a byte layout and record size; `SampleBatch` borrows bytes.
A processor must preserve size/layout, never retain the pointer, and declare
format compatibility before the worker starts. No new timestamps or serialized
metadata are added. The default remains raw 12-byte big-endian accel/gyro data.

A processor error skips that batch's write, recycles its buffer, and posts
`PROCESS_ERROR`. Write errors retain the existing discard-and-report policy.
FIFO starvation still leaves samples in the source; overflow still posts an
event and resets the source on dispatch. A final drain that has no free buffer
still leaves unread records behind; this change does not promise lossless stop
under saturation. Event queue insertion remains best effort.

Shutdown now freezes acquisition before the final drain, then queues the sync
sentinel. This removes the previous acquisition-during-shutdown window. A third
queue carries completion and lifetime error counters back to core 0. On timeout,
`stop()` returns false and the state machine stays `STOPPING`; another button
press retries the same stop, without queueing another sentinel. No storage
report or restart is allowed until acknowledgement. After completion, `stop()`
returns true even if a write/sync failed: true means the worker is quiescent,
not a durability guarantee. Failures are reported in events and the final
snapshot; sync failure also prints directly. The file stays open across sessions.
The LED remains on in `STOPPING` until acknowledgement.

### IMU final-drain regression

`imu_stop_host_test.cpp` compiles the real IMU implementation against a small
register model. It checks that `Inputs::stop()` disables sensor writes
using register 0x23 while retaining USER_CTRL.FIFO_EN, so FIFO_R_W stays readable.
Clearing USER_CTRL.FIFO_EN would disable reads as well as writes (manufacturer
register map, section 4.28). The test also checks retained byte values, restart,
idempotent stop, and propagated I2C failure. The Bash runner includes this test;
on Windows compile `tests/imu_stop_host_test.cpp src/imu.cpp` with the same
host include paths and flags. This model is not a physical sensor timing test.
