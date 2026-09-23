//! Build-time tool behind `engine_mod`: digests a mod's interface and records
//! the interfaces it was built against, as a `rustc_env_files` file the mod's
//! `export_mod!` bakes into the library.
//!
//! ```text
//! mod_links --out-env <file> --out-digest <file> [--src <file>]... [--dep <mod>=<digest file>]...
//! ```
//!
//! The digest covers the interface's source text and the digests of the
//! interfaces it depends on, so it changes whenever anything a dependent
//! compiled against could have. That includes comment-only edits: a false
//! positive costs a game reload, a false negative a dependent reading the
//! wrong layout.

use std::fs;

/// FNV-1a: stable across Rust versions and platforms, unlike `DefaultHasher`,
/// and change detection doesn't need a cryptographic hash.
fn fnv1a(hash: &mut u64, bytes: &[u8]) {
    for &b in bytes {
        *hash ^= b as u64;
        *hash = hash.wrapping_mul(0x100000001b3);
    }
}

/// The digest of an interface made of `srcs` (path and contents) that depends
/// on the interfaces `deps` (name and digest). No sources means no interface:
/// the digest is empty, and no mod can depend on this one.
fn interface_digest(srcs: &[(String, Vec<u8>)], deps: &[(String, String)]) -> String {
    if srcs.is_empty() {
        return String::new();
    }
    let mut hash = 0xcbf29ce484222325;
    let parts = srcs
        .iter()
        .flat_map(|(path, bytes)| [path.as_bytes(), bytes.as_slice()])
        .chain(deps.iter().flat_map(|(name, digest)| [name.as_bytes(), digest.as_bytes()]));
    for part in parts {
        // Length-prefixed, so no two different inputs hash the same bytes.
        fnv1a(&mut hash, &(part.len() as u64).to_le_bytes());
        fnv1a(&mut hash, part);
    }
    format!("{hash:016x}")
}

fn main() {
    let mut args = std::env::args().skip(1);
    let (mut out_env, mut out_digest) = (None, None);
    let (mut srcs, mut deps) = (Vec::new(), Vec::new());
    while let Some(flag) = args.next() {
        let value = args.next().unwrap_or_else(|| panic!("{flag} needs a value"));
        match flag.as_str() {
            "--out-env" => out_env = Some(value),
            "--out-digest" => out_digest = Some(value),
            "--src" => srcs.push(value),
            "--dep" => {
                let (name, file) = value.split_once('=').expect("--dep takes <mod>=<file>");
                let digest = fs::read_to_string(file).expect("reading a dep digest");
                deps.push((name.to_string(), digest.trim().to_string()));
            }
            _ => panic!("unknown flag {flag}"),
        }
    }

    let srcs: Vec<(String, Vec<u8>)> = srcs
        .into_iter()
        .map(|src| {
            let bytes = fs::read(&src).expect("reading an interface source");
            (src, bytes)
        })
        .collect();
    let digest = interface_digest(&srcs, &deps);

    let deps: Vec<String> = deps.iter().map(|(name, digest)| format!("{name}:{digest}")).collect();
    let env = format!("ENGINE_INTERFACE_DIGEST={digest}\nENGINE_MOD_DEPS={}\n", deps.join(","));
    fs::write(out_env.expect("--out-env"), env).expect("writing the env file");
    fs::write(out_digest.expect("--out-digest"), digest).expect("writing the digest");
}

#[cfg(test)]
mod tests {
    use super::interface_digest;

    fn src(path: &str, text: &str) -> (String, Vec<u8>) {
        (path.into(), text.as_bytes().to_vec())
    }

    #[test]
    fn the_digest_changes_with_anything_a_dependent_compiled_against() {
        let base = interface_digest(&[src("a.rs", "struct A;")], &[]);
        assert_eq!(base.len(), 16, "{base}");
        assert_eq!(base, interface_digest(&[src("a.rs", "struct A;")], &[]), "must be deterministic");
        // An edit to the same file: the case a real reload hits.
        assert_ne!(base, interface_digest(&[src("a.rs", "struct A(u8);")], &[]));
        assert_ne!(base, interface_digest(&[src("b.rs", "struct A;")], &[]));
        assert_ne!(base, interface_digest(&[src("a.rs", "struct A;"), src("b.rs", "")], &[]));
        // A dependency's interface changing changes this one, since its types
        // can appear in this interface's.
        let dep = |digest: &str| [("physics".to_string(), digest.to_string())];
        assert_ne!(base, interface_digest(&[src("a.rs", "struct A;")], &dep("01")));
        assert_ne!(
            interface_digest(&[src("a.rs", "struct A;")], &dep("01")),
            interface_digest(&[src("a.rs", "struct A;")], &dep("02"))
        );
    }

    #[test]
    fn moving_bytes_between_files_changes_the_digest() {
        // Without length prefixes these would hash the same concatenation.
        assert_ne!(
            interface_digest(&[src("a.rs", "xy"), src("b.rs", "z")], &[]),
            interface_digest(&[src("a.rs", "x"), src("b.rs", "yz")], &[])
        );
    }

    #[test]
    fn no_sources_means_no_interface() {
        assert_eq!(interface_digest(&[], &[("physics".into(), "01".into())]), "");
    }
}
