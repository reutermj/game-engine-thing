//! Just enough JSON to write views: an interface crate depends only on
//! `engine_api` (defs.bzl), so no serde here.

/// An object, written as fields are added.
pub(crate) struct Object {
    out: String,
}

impl Object {
    pub(crate) fn new() -> Object {
        Object { out: "{".into() }
    }

    fn key(&mut self, key: &str) {
        if self.out.len() > 1 {
            self.out.push(',');
        }
        string_to(&mut self.out, key);
        self.out.push(':');
    }

    pub(crate) fn string(&mut self, key: &str, value: &str) {
        self.key(key);
        string_to(&mut self.out, value);
    }

    /// An integer, or a float written as f64 prints it.
    pub(crate) fn number(&mut self, key: &str, value: f64) {
        self.key(key);
        self.out += &number(value);
    }

    /// A float as f32 prints it, the shortest that reads back the same
    /// (0.1, not 0.10000000149011612).
    pub(crate) fn float(&mut self, key: &str, value: f32) {
        self.key(key);
        self.out += &float(value);
    }

    /// `value` as it is: an array or object already written.
    pub(crate) fn raw(&mut self, key: &str, value: &str) {
        self.key(key);
        self.out += value;
    }

    pub(crate) fn finish(mut self) -> String {
        self.out.push('}');
        self.out
    }
}

pub(crate) fn array(items: impl Iterator<Item = String>) -> String {
    format!("[{}]", items.collect::<Vec<_>>().join(","))
}

/// JSON has no NaN or infinity: they're written `null`, which a reader
/// can't mistake for a number.
pub(crate) fn number(value: f64) -> String {
    if value.is_finite() { value.to_string() } else { "null".into() }
}

pub(crate) fn float(value: f32) -> String {
    if value.is_finite() { value.to_string() } else { "null".into() }
}

pub(crate) fn string(value: &str) -> String {
    let mut out = String::new();
    string_to(&mut out, value);
    out
}

fn string_to(out: &mut String, value: &str) {
    out.push('"');
    for c in value.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}
