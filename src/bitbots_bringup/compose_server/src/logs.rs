//! In-memory log buffers of the supervised processes.

use std::{
    collections::{HashMap, VecDeque},
    sync::Mutex,
};

use crate::api::{LogLine, LogStream, Logs};

const ALL_CAPACITY: usize = 20_000;
const COMPONENT_CAPACITY: usize = 5_000;

#[derive(Default)]
struct Buffer {
    lines: VecDeque<LogLine>,
    /// Sequence number of the newest line that was dropped to make room
    evicted_up_to: u64,
}

impl Buffer {
    fn push(&mut self, line: LogLine, capacity: usize) {
        if self.lines.len() >= capacity {
            if let Some(old) = self.lines.pop_front() {
                self.evicted_up_to = old.seq;
            }
        }
        self.lines.push_back(line);
    }

    fn since(&self, since: u64, limit: usize) -> (Vec<LogLine>, bool) {
        let start = self.lines.partition_point(|l| l.seq <= since);
        let lines = self.lines.range(start..).take(limit).cloned().collect();
        (lines, self.evicted_up_to > since)
    }
}

#[derive(Default)]
struct Inner {
    latest_seq: u64,
    all: Buffer,
    per_component: HashMap<String, Buffer>,
}

/// Ring buffers of all output lines, both combined and per component.
pub struct LogStore {
    instance_id: u64,
    inner: Mutex<Inner>,
}

impl Default for LogStore {
    fn default() -> Self {
        Self::new()
    }
}

impl LogStore {
    pub fn new() -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock is after 1970");
        Self {
            instance_id: now.as_nanos() as u64 ^ u64::from(std::process::id()),
            inner: Mutex::new(Inner::default()),
        }
    }

    pub fn push(&self, component: &str, component_name: &str, stream: LogStream, text: &str) {
        let mut inner = self.inner.lock().unwrap();
        inner.latest_seq += 1;
        let line = LogLine {
            seq: inner.latest_seq,
            component: component.to_string(),
            component_name: component_name.to_string(),
            stream,
            text: text.to_string(),
        };
        inner
            .per_component
            .entry(component.to_string())
            .or_default()
            .push(line.clone(), COMPONENT_CAPACITY);
        inner.all.push(line, ALL_CAPACITY);
    }

    /// Lines newer than `since`, oldest first, optionally restricted to one component.
    pub fn since(&self, since: u64, component: Option<&str>, limit: usize) -> Logs {
        let inner = self.inner.lock().unwrap();
        let (lines, truncated) = match component {
            None => inner.all.since(since, limit),
            Some(key) => inner
                .per_component
                .get(key)
                .map(|b| b.since(since, limit))
                .unwrap_or_default(),
        };
        Logs {
            lines,
            truncated,
            latest_seq: inner.latest_seq,
            instance_id: self.instance_id,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn since_returns_newer_lines_per_component() {
        let store = LogStore::new();
        store.push("a", "A", LogStream::Stdout, "a1");
        store.push("b", "B", LogStream::Stderr, "b1");
        store.push("a", "A", LogStream::Stdout, "a2");

        let all = store.since(1, None, usize::MAX);
        let texts: Vec<_> = all.lines.iter().map(|l| l.text.as_str()).collect();
        assert_eq!(texts, ["b1", "a2"]);
        assert_eq!(all.latest_seq, 3);
        assert!(!all.truncated);

        let a = store.since(0, Some("a"), usize::MAX);
        let texts: Vec<_> = a.lines.iter().map(|l| l.text.as_str()).collect();
        assert_eq!(texts, ["a1", "a2"]);
        assert!(store.since(0, Some("unknown"), usize::MAX).lines.is_empty());
    }

    #[test]
    fn limit_returns_oldest_first() {
        let store = LogStore::new();
        for i in 0..5 {
            store.push("a", "A", LogStream::Stdout, &i.to_string());
        }
        let logs = store.since(0, None, 2);
        let seqs: Vec<_> = logs.lines.iter().map(|l| l.seq).collect();
        assert_eq!(seqs, [1, 2]);
    }

    #[test]
    fn eviction_is_reported_as_truncation() {
        let store = LogStore::new();
        for i in 0..COMPONENT_CAPACITY + 1 {
            store.push("a", "A", LogStream::Stdout, &i.to_string());
        }
        assert!(store.since(0, Some("a"), usize::MAX).truncated);
        assert!(!store.since(1, Some("a"), usize::MAX).truncated);
    }
}
