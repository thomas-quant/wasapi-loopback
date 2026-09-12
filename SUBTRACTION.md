# `startEndpointMinusSelf`: endpoint loopback minus own-process loopback

Status: **implemented, compiles for Windows (`cargo check`), and passes the simulated-packet test suite on Linux. Not yet run on a real Windows box.** The on-box checks it still needs are listed at the end.

## What it does

The mix leg is `C`: plain loopback of one render endpoint, which is what the user hears. It is device-scoped, so a virtual cable that never reaches this endpoint stays out. The reference leg is `R`: process-loopback INCLUDE of the tree rooted at `rootPid`, i.e. GoofCord's own playback. Both run in **one native owner thread**. For each endpoint frame the output is one of:

| emitted | when |
|---|---|
| `C[i] − R[i + offset]` (unity gain, no filter, no adaptation) | `running`: the generation's integer offset is verified |
| `C[i]` unchanged | `aligning`, **only** once an earlier lock has proved the timing bias, and `R` is exactly zero, from real captured packets, across the whole timing-uncertainty window around the mapped frame |
| `0` (muted) | everything else: before the first lock, missing reference packets, a non-silent reference without a lock |

Nothing ever falls back to EXCLUDE capture or to the raw endpoint while the alignment is unknown.

```ts
type OnChunk = (err: unknown, chunk?: Buffer) => void; // 3840-byte (480-frame 48 kHz stereo f32) chunks; err ⇒ session ended
startEndpointMinusSelf(rootPid: number, deviceId: string | undefined | null, onChunk: OnChunk): number; // 0 ⇒ start refused
getSubtractionStatus(sessionId: number): SubtractionStatus | null;
getLastSubtractionStartError(): string | null; // why the last start returned 0
// stopSession / stopAll / getCaptureStats work as for every other session (stats describe the endpoint leg)
interface SubtractionStatus {
  state: "aligning" | "running" | "failed"; reason: string;
  offsetFrames: number; locked: boolean; bufferedFrames: number; generation: number; endpointId: string;
  coarseOffsetFrames?: number; candidateOffsetFrames?: number;
  gainLeft?: number; gainRight?: number; delayLeft?: number; delayRight?: number;
  endpointFrames: number; referenceFrames: number; subtractedFrames: number; passthroughFrames: number; mutedFrames: number;
  locks: number; rejectedCandidates: number; discontinuities: number; timelineGaps: number; droppedChunks: number;
}
```

Output frame `k` is always endpoint frame `k`, so the output stays endpoint-paced. An idle endpoint produces no chunks, the same as `startRenderEndpointLoopback`.

## Timeline model

- **Local frame indices.** Each leg counts the frames it has read, silent packets included. DevicePosition is not used: process loopback reports 0, and the endpoint's absolute position is irrelevant.
- **Packet timing.** Each packet's engine QPC (100 ns) is paired with the local index of the packet's first frame. When the engine QPC is flagged invalid, the host's read-time QPC is used instead, with a much wider tolerance.
- **Generation.** A stretch in which both legs are gap-free. One integer `offset = j − i` then holds for the whole generation. A generation ends on `DATA_DISCONTINUITY` on either leg, or when a packet's start time disagrees with the previous packet on its leg by more than 5 ms (engine QPC) or 40 ms (read time). Missing packets are never treated as silence. Frames that were still waiting are muted.
- **Coarse estimate.** Packet timing gives `offset ≈ (r0 − c0) + (qC − qR)·48000/10⁷`. It only centres the search. The difference between the verified offset and this estimate is the **tap bias**, recorded after the first lock.

## Lock procedure (per generation)

1. **Search.** Take a 0.5 s endpoint block at least 250 ms after a reference that carries energy. Run a normalized stereo cross-correlation over ±12000 frames (±250 ms) around the estimate, via FFT on a worker thread. The candidate is rejected if the peak is weak (< 0.05), sits at the window edge, or is ambiguous: another local maximum outside ±32 frames reaching 0.9 of the peak, which is what periodic audio produces.
2. **Held-out verification.** Check the candidate on later audio that doesn't overlap the calibration block, in 50 ms sub-blocks, separately per channel, at unity gain:
   - gain ratio `⟨C,R⟩/⟨R,R⟩`;
   - fractional-delay projection `⟨C−R, R′⟩/⟨R′,R′⟩`, which reads ≈ ±1 when the integer is off by one.

   Accept only when, for every channel with at least 8 valid sub-blocks, `|mean − target| + 2·se` lies inside the tolerance: gain within 1 ± 0.03, delay within 0 ± 0.05 samples. It's an equivalence-style test. Other apps are independent of the reference, so they only add noise, and noise can only delay acceptance, never cause it. A conclusive rejection drops the candidate. So does being still inconclusive after 200 sub-blocks (10 s).
3. **Running monitor.** The same two estimates keep running over non-overlapping 20-sub-block windows. Two consecutive windows that reject at `z = 4` fault the session with "lost lock". This catches clock slips, lock loss the timing didn't flag, and own audio that stops reaching the endpoint.
4. **Timeout.** After 20 s of own audio with no lock, the session **faults only if there were ≥ 2 conclusive mismatches**: unity rejected on held-out audio, a peak at the boundary, or the endpoint silent while the reference is active. Ambiguous or drowned-out evidence keeps it muted in `aligning` with an explanatory `reason`, because lack of evidence is not a mismatch.

A new lock whose tap bias disagrees with the previously proven bias by more than the passthrough window faults the session. Such a disagreement means an earlier silent-reference passthrough rested on a false assumption.

## What faults the session (callback error + `state: "failed"`)

- **Overflow.** More than 24000 endpoint frames (0.5 s) waiting for reference audio, i.e. the reference leg stalled.
- **Missing reference.** No reference packets for 2 s of endpoint audio.
- **Reference history overrun.**
- **Lost lock.** The monitor rejected.
- **Timeout with mismatches.** 20 s of own audio, no lock, and ≥ 2 conclusive mismatches.
- **Bias contradiction.** A new lock disagrees with the proven tap bias.
- **Bad samples.** Non-finite or malformed packets.
- **Route guard.** See below.
- **WASAPI errors.** Any WASAPI call error, e.g. `AUDCLNT_E_DEVICE_INVALIDATED` after a format change or unplug.

Start is refused (`0`, with a reason) for:

- an endpoint whose mix format isn't native 48 kHz / 2 channels / front-stereo mask. Each tap would run its own SRC or matrix, and two converters are not sample-identical;
- own-tree playback already active on another endpoint;
- process-loopback INCLUDE being unavailable on this system.

## Off-endpoint guard, and its limits

INCLUDE is device-agnostic. If GoofCord also plays to endpoint F, subtracting `R` would inject F's audio, inverted, into the share. Every 250 ms the owner:

- enumerates active sessions on every active render endpoint, plus a Toolhelp snapshot of the `rootPid` process tree, and fails if any own-tree process has an **active** session on another endpoint;
- when following the default endpoint, fails if the eConsole default changes.

Limits:

- **Polling.** Newly started off-endpoint playback can leak inverted audio for up to ~250 ms, plus the time until the monitor's gain deficit also triggers.
- **Session granularity.** A session carries no packet-level routing metadata.
- **PID reuse** can make the tree snapshot include an unrelated process.
- **Enumeration failure.** If any endpoint's sessions can't be enumerated, the session fails rather than guessing.

## Limits of the exactness claim

- The **lock gate** certifies a correct integer offset and a path within ±3 % gain and ±0.05 samples. Implication: those bounds alone would allow a residual as high as −30 dB (pure gain error) and −23.7 dB at 10 kHz / −43.7 dB at 1 kHz (pure delay error), relative to own audio. The actual null is whatever the tap identity delivers. It was measured bit-exact once, on HDMI only (mono-collapsed pink noise, 17 s). Nothing here claims −60 dB on arbitrary devices.
- **Off by one is worse than no subtraction.** At 10 kHz a one-frame error leaves `2|sin(π·10000/48000)| ≈ 1.22`, i.e. +1.7 dB. That's why the delay projection is part of the gate, not only gain.
- **Clock drift.** 1 ppm slips ~2.9 frames/min. The monitor's delay estimate catches a slip before or when it happens, but a slow fractional drift below 0.05 samples is tolerated until then.
- **Nonlinear effects.** Endpoint enhancements such as a limiter or loudness EQ are not detected via `IAudioEffectsManager`. They're only caught by the gain/delay tests, which are statistical and pass near-linear chains, e.g. the Realtek APO measured flat within ±0.04 dB.
- **Listen-to-this-device.** With it on, mic/VAC audio genuinely is on the endpoint and stays. That's out of scope by construction.
- **Lock speed** falls as other apps get louder relative to own audio. Verification noise scales with that amplitude ratio divided by √(effective samples). In simulation, +6 dB louder other audio locks within 14 s. At +12 dB it may not lock in 8 s, but it never mislocks or faults.

## The HDMI "+1024" shift

Measured: HDMI PL2470H gave `C − R = 0` exactly at a **local-index** lag of 1024. On Realtek the lag changed on every start (−480, 960, 1440, −868); CABLE Input gave 6720. A local-index lag is the start-order offset between the two legs plus the tap bias, and the two can't be separated from those runs, because the harness at the time didn't record per-packet QPC. Design consequences:

- **Never round to the 480-frame period.** 1024 and −868 aren't multiples of it.
- **Never hardcode 1024.** Search per generation.
- **Integer lags can be exact, so verify at integer precision.** The engine searches ±12000 frames around the packet-timing estimate, so a constant bias of up to 250 ms is covered, CABLE's 6720 included.
- **Tests** cover a content offset 1024 frames away from what packet timing says (not a period multiple), plus −868, 0, 1, 1023 and 6720, with mismatched packet sizes (441/480/519 vs 480).
- **Open question.** If the HDMI 1024 is a constant tap bias, e.g. an HDMI endpoint period/buffer of 1024 frames = 21.33 ms, then `offsetFrames − coarseOffsetFrames` will read ≈ 1024 on every start. If it was start-order noise, it will read ≈ 0. **Log both across several starts on HDMI to settle it.**

## Tests (Linux, no Windows crate)

```sh
cargo test --manifest-path native/wasapi-loopback/subtract-core/Cargo.toml
```

The unit tests cover:

- FFT against a direct DFT;
- FFT correlation against a direct sum;
- ring bounds, zero-run counting and non-finite rejection;
- equivalence verdicts;
- the process-tree and route decision;
- the search at offsets 1024/−868/6720/1;
- periodic ambiguity;
- absent own audio.

The packet-level simulation (`tests/paired_capture.rs`) checks, in every scenario, that **each output frame is either muted or exactly the other-apps signal**: own audio never leaks, is never injected inverted, and the raw endpoint never passes while own audio is present. Scenarios:

- 1024 HDMI-style shift;
- isolated own audio cancels to bit-exact zero;
- arbitrary offsets with mismatched packet sizes;
- read-time-only timestamps;
- +6 dB and +12 dB louder independent other audio;
- reference silent after lock (other apps pass bit-exactly);
- reference silent from the start (stays muted, no raw passthrough);
- periodic own audio (ambiguous, never locks);
- 0.8 path gain (rejected, faults);
- swapped channels (never lock);
- an unflagged reference gap and a flagged endpoint discontinuity (new generation, relock);
- reference stall (overflow fault);
- reference never delivering (fault);
- an undetected one-frame slip ("lost lock" fault);
- endpoint idle gap with a proven bias (silent-reference passthrough, then relock).

## Needs verification on a real box

1. **Continuous INCLUDE packets.** Does INCLUDE of the Electron tree deliver packets (silent-flagged) continuously while GoofCord renders nothing? If it stops, the session faults with "overflow" or "no packets" by design, since missing packets are not silence, and that behaviour would need a product decision.
2. **Packet timing across legs.** Are engine QPCs present and comparable between the endpoint and process legs? Read `coarseOffsetFrames` against `offsetFrames` over several starts, HDMI and Realtek. If the tap bias exceeds ±250 ms anywhere, the search radius must grow.
3. **Lock on real audio.** Lock time and the gain/delay estimates on real call audio and the Discord stream-start chime, with and without other audio playing.
4. **Route guard.** No false fault from GoofCord's own loopback capture sessions. Also check the result of moving GoofCord's output device mid-share.
5. **Long runs.** A 30-minute run with no "lost lock" on a single-clock endpoint.
