//! Portable core of `startEndpointMinusSelf`: endpoint loopback minus the process-loopback
//! INCLUDE capture of GoofCord's own tree, at one verified integer frame offset and unity gain.
//!
//! Nothing here touches Windows. The native owner feeds every WASAPI packet of both legs — local
//! frame index (implied by order), frame count, flags, engine QPC and the host's read time — into
//! [`Engine::push`] *before* any rechunking, and pops 480-frame stereo f32 chunks back out. The
//! correlation search is a pure [`run_align_job`] so the owner can run it off the capture thread.
//!
//! What the engine guarantees, and what it does not, is written down in `../SUBTRACTION.md`.

mod align;
mod engine;
mod fft;
mod ring;
pub mod route;
mod stats;

pub use align::{run_align_job, AlignJob, AlignOutcome, AlignParams, AlignResult};
pub use engine::{
    Config, Engine, Fault, Leg, PacketInfo, Phase, Status, CHANNELS, CHUNK_FRAMES, SAMPLE_RATE,
};
