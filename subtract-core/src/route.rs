//! Off-endpoint guard logic (pure; the Windows owner supplies the snapshots).
//!
//! Process-loopback INCLUDE is device-agnostic: it contains GoofCord's playback *wherever* it is
//! routed. Subtracting it from endpoint E while some of that playback goes to endpoint F injects an
//! inverted copy of F's share into the output — audio that was never on E. The owner therefore
//! polls the audio sessions of every active render endpoint and fails the session as soon as any
//! process in the own tree has an *active* session on an endpoint other than E.
//!
//! Limits (also in SUBTRACTION.md): this is polling, not packet-level routing metadata. A session
//! that turns active between polls can leak for up to one poll interval, and a single session
//! cannot tell us which of its samples went where. The engine's running gain/delay monitor is the
//! second, content-based line: own audio that stops arriving at E shows up as a gain deficit.

/// One audio session as seen on one render endpoint.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SessionInfo {
    pub endpoint_id: String,
    pub pid: u32,
    pub active: bool,
}

/// `root` plus every descendant in a `(pid, parent_pid)` process snapshot. Cycles (pid reuse can
/// make a snapshot look cyclic) and pid 0 are ignored.
pub fn process_tree(root: u32, entries: &[(u32, u32)]) -> Vec<u32> {
    let mut tree = vec![root];
    let mut i = 0;
    while i < tree.len() {
        let parent = tree[i];
        for &(pid, ppid) in entries {
            if ppid == parent && pid != 0 && pid != ppid && !tree.contains(&pid) {
                tree.push(pid);
            }
        }
        i += 1;
    }
    tree
}

/// `Err(reason)` when any active session owned by the own tree is on an endpoint other than
/// `target_endpoint`. Inactive sessions (created but not currently rendering) are allowed.
pub fn check_own_routes(target_endpoint: &str, own: &[u32], sessions: &[SessionInfo]) -> Result<(), String> {
    for s in sessions {
        if s.active && own.contains(&s.pid) && !s.endpoint_id.eq_ignore_ascii_case(target_endpoint) {
            return Err(format!(
                "own process {} is playing on another endpoint ({}); the process reference would \
                 inject that audio inverted, so multi-endpoint own playback is unsupported",
                s.pid, s.endpoint_id
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn process_tree_collects_descendants_and_survives_cycles() {
        let entries = [(10, 1), (20, 10), (30, 20), (40, 99), (10, 30), (0, 10), (50, 50)];
        let mut t = process_tree(10, &entries);
        t.sort();
        assert_eq!(t, vec![10, 20, 30]);
        assert_eq!(process_tree(50, &entries), vec![50]);
    }

    #[test]
    fn only_active_own_sessions_off_the_target_endpoint_fail() {
        let s = |ep: &str, pid, active| SessionInfo { endpoint_id: ep.into(), pid, active };
        let own = [10, 20];
        assert!(check_own_routes("{E}", &own, &[s("{e}", 20, true), s("{F}", 10, false), s("{F}", 77, true)]).is_ok());
        let err = check_own_routes("{E}", &own, &[s("{E}", 20, true), s("{F}", 20, true)]).unwrap_err();
        assert!(err.contains("another endpoint"), "{err}");
    }
}
