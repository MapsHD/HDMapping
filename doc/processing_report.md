# Step 1 run analysis

The raylib Step 1 GUI and CLI add `run_analysis` (schema version 1) to
`HDMapping_results_*.json`. The completion dialog and console use the same
accounting object. Existing result metadata is retained.

## Timing boundaries

Durations use `std::chrono::steady_clock`. UTC timestamps identify events;
changing the system clock does not change measured durations.

- `loading`: input discovery/loading pass, including the initial lazy-loader range pass.
- `initialization`: IMU-based initial trajectory.
- `cache_preparation`: worker/chunk preparation, decimation, cache writes, and
  any deferred raw-cloud reads performed by lazy loading.
- `odometry`: the complete odometry call, including its cache reads, spill work,
  final bookkeeping and spill destruction.
- `export`: preparing, writing and closing point clouds, trajectories, poses,
  session metadata and parameters. A writer failure makes the export unsuccessful.
- `cleanup`: the actual cache removal attempt.
- `consistency`: optional post-processing, when invoked.

Stage wall times include actual pauses. `paused_seconds` measures waiting at
the processing pause boundary, not time since clicking Pause. The separate
`excluding_pauses_seconds` is wall time with those waits subtracted, not CPU time.
The progress ETA uses odometry time excluding pauses; it does not predict export.

Progress also shows `Raw recording span (IMU)`: the last minus first loaded raw
IMU timestamp, in seconds and minutes, including acquisition gaps. It uses the
already sorted input IMU vector, not the shorter optimized trajectory or file
modification times, and requires no additional data reads/scans. The same range
and source are saved under `run_analysis.input_recording`. These sensor
timestamps are not UTC/`timestampUnix`; missing/invalid ranges remain null.

`total_operation_wall_seconds` sums measured operation intervals. For the
automatic GUI flow this begins before input discovery and ends after export.
For CLI it also includes immediate cache cleanup. Gaps between stages inside
an operation appear as `between_stages_seconds`. Later interactive idle is
excluded. Re-export, consistency and deferred cleanup resume accounting as
additional operations of the same run; their intervals are recorded in
`operations`. They do not reset the odometry duration.

Cache write durations are **nested** within preparation/export. They are
reported separately by kind (`initial`, `export`) and must not be added to totals.
The final report's own serialization and replacement are outside measured
operation intervals; completion is shown only after that write returns.
Closing a stream does not imply an explicit physical-media flush.

The legacy `processing_info.elapsed_time_seconds` is odometry wall time,
including pauses, when run analysis is present. Its scope is labeled explicitly.
Historical files generated before this change have different boundaries.

## Resource statistics and cost

The sampler queries native process/system memory approximately once a second,
and available disk space every five seconds, independently of frame rate.
Stage transitions request an asynchronous refresh, with a minimum 100 ms
between samples to bound bursts. A stage boundary can carry the most recent
available sample of the same operation. Interactive idle is never extrapolated.

Each metric records first/last observations, time-weighted mean over valid
coverage, minimum, sampled maximum, observation count and coverage duration.
Missing values are null, not zero. Short peaks can fall between samples.
Per-stage statistics use the same definitions. No unbounded sample history or
periodic cache-directory traversal is retained.

Windows uses process working set for resident RAM, private commit for process
committed memory, and system commit/limit for context. Commit is **not actual
pagefile disk occupancy**; subtracting resident RAM from commit does not produce
that quantity. Active pagefile volumes are discovered at run start, alongside
input/cache, optional spill and output locations. Available disk space is not
a promise of additional commit capacity. Hover a volume in the GUI to see its roles.

GPU metrics use Windows GPU Process Memory PDH counters, with dedicated and
shared memory recorded per adapter LUID/physical index. Adapter sums may count
shared allocations more than once; they are not unique physical residency.
OS/driver limitations apply. The PDH query is reused between runs in the process.
Unsupported platforms/counters report unavailable values. Linux and macOS
provide resident RAM; Linux also provides available system RAM. Windows-specific
commit/GPU counters are not replaced with differently defined quantities.

`sampling` reports first/subsequent query wall cost, maximum query cost,
sampler-thread CPU time when supported, setup cost and cache-accounting cost.
A cold GPU performance-provider initialization can cost much more than later
queries. Queries run outside the report mutex on the sampling thread; their
wall duration is not equivalent to time blocked in the processing worker.
GUI snapshots only copy small aggregates and make no OS resource queries.

Cache accounting performs one size lookup per completed file, including export
chunks; overwrites replace the previous size. It records logical bytes/file
counts after preparation, historical peak and cleanup status. Spill sizes are
updated at reset/finish/destruction boundaries, never for each appended point.
Spill peak includes the overlapping old/new generation during reset. Disk-free
minima are sampled; cache peaks are derived from the application's file events.
Cache-file accounting covers this run's completed writes, not an inventory of
unmodified files left by an earlier crashed run. That scope is saved in the JSON.
After a failed cache removal, remaining bytes/file count are unknown (null).
Spill cleanup errors identify files that may remain outside the current stream.

## Completion and later updates

The GUI retains cache for consistency/editing/re-export and reports it as
`retained`. Closing the application removes it and updates all reports registered
for that run without repeating the export completion popup. Automatic processing
and manual re-export share a single completion-notification path. CLI removes
cache immediately and updates its report before the
final console summary. No result file is written by the sampler itself.

Reports are written through a temporary file and replaced only after the stream
has closed successfully. Error/cancellation status is retained during later
cleanup. Failed or cancelled runs write a diagnostic report when an output
directory is available; an unwritable destination cannot retain its own failure
report. Fatal process termination cannot guarantee finalization.

This change covers the raylib Step 1 workflow. Shared processing sources also
build for the older GLUT entry points, whose legacy UI is not redesigned here.
