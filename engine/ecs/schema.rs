//! Schemas: the field-by-field description of a component or a mod's state
//! that lets values migrate between builds whose layouts differ. Shared by
//! the world's components and by mod state (the loader), which migrate by
//! the same rules (docs/architecture/ecs.md, "Layout changes").

use crate::component::{DefaultFn, DropFn, FieldDesc, FieldKind};

/// A field of a schema, copied out of the mod's `FieldDesc` because that
/// points into the mod's memory. Its drop function is kept apart, with the
/// build it belongs to.
#[derive(PartialEq, Debug)]
pub struct Field {
    name: String,
    kind: FieldKind,
    offset: usize,
    size: usize,
    fingerprint: u64,
}

impl Field {
    fn describe(&self) -> String {
        if self.kind == FieldKind::OPAQUE {
            format!("type {:016x}", self.fingerprint)
        } else {
            kind_name(self.kind).into()
        }
    }
}

/// Whether a value in field `o` can become one in field `f`: moved, if they
/// have the same type, or converted between numeric kinds.
pub fn carries(o: &Field, f: &Field) -> bool {
    if o.kind == FieldKind::OPAQUE || f.kind == FieldKind::OPAQUE {
        o.kind == f.kind && o.fingerprint == f.fingerprint && o.size == f.size
    } else {
        o.kind == f.kind || (is_numeric(o.kind) && is_numeric(f.kind))
    }
}

/// The fields `fields[..count]` describe, with their drop functions, or
/// `None` if they (or `size` and `align`) don't describe a valid layout.
///
/// # Safety
/// `fields` must point to `count` descs (or be anything, with `count` 0).
pub unsafe fn read_fields(
    fields: *const FieldDesc,
    count: usize,
    size: usize,
    align: usize,
) -> Option<(Vec<Field>, Vec<Option<DropFn>>)> {
    if !align.is_power_of_two() || size % align != 0 {
        return None;
    }
    let descs = match count {
        0 => &[][..],
        n => unsafe { std::slice::from_raw_parts(fields, n) },
    };
    descs
        .iter()
        .map(|f| {
            // A scalar's size is fixed by its kind; anything else is opaque.
            if f.kind != FieldKind::OPAQUE && kind_size(f.kind)? != f.size {
                return None;
            }
            if f.offset.checked_add(f.size)? > size {
                return None;
            }
            let name = unsafe { std::slice::from_raw_parts(f.name, f.name_len) };
            let name = String::from_utf8(name.to_vec()).ok()?;
            let field = Field { name, kind: f.kind, offset: f.offset, size: f.size, fingerprint: f.fingerprint };
            Some((field, f.drop))
        })
        .collect::<Option<Vec<_>>>()
        .map(|pairs| pairs.into_iter().unzip())
}

fn kind_size(kind: FieldKind) -> Option<usize> {
    Some(match kind {
        FieldKind::U8 | FieldKind::I8 | FieldKind::BOOL => 1,
        FieldKind::U16 | FieldKind::I16 => 2,
        FieldKind::U32 | FieldKind::I32 | FieldKind::F32 => 4,
        FieldKind::U64 | FieldKind::I64 | FieldKind::F64 | FieldKind::ENTITY => 8,
        _ => return None,
    })
}

pub fn kind_name(kind: FieldKind) -> &'static str {
    match kind {
        FieldKind::U8 => "u8",
        FieldKind::U16 => "u16",
        FieldKind::U32 => "u32",
        FieldKind::U64 => "u64",
        FieldKind::I8 => "i8",
        FieldKind::I16 => "i16",
        FieldKind::I32 => "i32",
        FieldKind::I64 => "i64",
        FieldKind::F32 => "f32",
        FieldKind::F64 => "f64",
        FieldKind::BOOL => "bool",
        FieldKind::ENTITY => "Entity",
        FieldKind::OPAQUE => "opaque",
        _ => "?",
    }
}

fn is_numeric(kind: FieldKind) -> bool {
    !matches!(kind, FieldKind::BOOL | FieldKind::ENTITY | FieldKind::OPAQUE)
}

/// A numeric value wide enough to hold any numeric field exactly.
enum Num {
    Int(i128),
    Float(f64),
}

/// Moves one field from the old layout into the new: byte for byte when the
/// kinds match (`size` bytes; the caller has checked an opaque field's type),
/// and between numeric kinds with `as` semantics (floats saturate into
/// integers, integers wrap into narrower ones). Leaves `dst` untouched, so
/// holding the default, when the kinds can't convert.
///
/// # Safety
/// `src` and `dst` must point to fields of the given kinds; both may be
/// unaligned.
pub unsafe fn convert(from: FieldKind, src: *const u8, size: usize, to: FieldKind, dst: *mut u8) {
    if from == to {
        unsafe { std::ptr::copy_nonoverlapping(src, dst, size) };
        return;
    }
    if !is_numeric(from) || !is_numeric(to) {
        return;
    }
    let value = unsafe {
        match from {
            FieldKind::U8 => Num::Int(src.read() as i128),
            FieldKind::U16 => Num::Int((src as *const u16).read_unaligned() as i128),
            FieldKind::U32 => Num::Int((src as *const u32).read_unaligned() as i128),
            FieldKind::U64 => Num::Int((src as *const u64).read_unaligned() as i128),
            FieldKind::I8 => Num::Int((src as *const i8).read() as i128),
            FieldKind::I16 => Num::Int((src as *const i16).read_unaligned() as i128),
            FieldKind::I32 => Num::Int((src as *const i32).read_unaligned() as i128),
            FieldKind::I64 => Num::Int((src as *const i64).read_unaligned() as i128),
            FieldKind::F32 => Num::Float((src as *const f32).read_unaligned() as f64),
            FieldKind::F64 => Num::Float((src as *const f64).read_unaligned()),
            _ => return,
        }
    };
    macro_rules! write {
        ($ty:ty) => {
            unsafe {
                (dst as *mut $ty).write_unaligned(match value {
                    Num::Int(v) => v as $ty,
                    Num::Float(v) => v as $ty,
                })
            }
        };
    }
    match to {
        FieldKind::U8 => write!(u8),
        FieldKind::U16 => write!(u16),
        FieldKind::U32 => write!(u32),
        FieldKind::U64 => write!(u64),
        FieldKind::I8 => write!(i8),
        FieldKind::I16 => write!(i16),
        FieldKind::I32 => write!(i32),
        FieldKind::I64 => write!(i64),
        FieldKind::F32 => write!(f32),
        FieldKind::F64 => write!(f64),
        _ => {}
    }
}


/// Rewrites one value from an old layout into a new one, matching fields by
/// name: the new value starts as the new build's `Default`, takes every old
/// field that `carries` over (the default's field dropped first), and every
/// old field that doesn't carry over is dropped with the old build's code.
/// Afterwards the old value has been fully moved out or dropped.
///
/// # Safety
/// `old` must be a live value laid out per its fields, whose drops belong to
/// code that is still mapped; `new` must be uninitialized memory for the new
/// layout.
pub unsafe fn migrate(
    old: (*mut u8, &[Field], &[Option<DropFn>]),
    new: (*mut u8, &[Field], &[Option<DropFn>], DefaultFn),
) {
    let (old, old_fields, old_drops) = old;
    let (new, new_fields, new_drops, default) = new;
    let mut carried = vec![false; old_fields.len()];
    unsafe {
        default(new);
        for (f, drop_new) in new_fields.iter().zip(new_drops) {
            let Some(i) = old_fields.iter().position(|o| o.name == f.name) else { continue };
            let o = &old_fields[i];
            if !carries(o, f) {
                continue;
            }
            // The default's field is overwritten, so drop it first.
            if let Some(drop) = drop_new {
                drop(new.add(f.offset));
            }
            convert(o.kind, old.add(o.offset), o.size, f.kind, new.add(f.offset));
            carried[i] = true;
        }
        for ((o, drop_old), carried) in old_fields.iter().zip(old_drops).zip(carried) {
            if let (false, Some(drop)) = (carried, drop_old) {
                drop(old.add(o.offset));
            }
        }
    }
}

/// What `migrate` does to each field, for the log: "kept x, added z (default)".
pub fn report(old: &[Field], new: &[Field]) -> String {
    let mut report = Vec::new();
    for f in new {
        report.push(match old.iter().find(|o| o.name == f.name) {
            Some(o) if carries(o, f) && o.kind == f.kind => format!("kept {}", f.name),
            Some(o) if carries(o, f) => {
                format!("converted {} {} -> {}", f.name, kind_name(o.kind), kind_name(f.kind))
            }
            Some(o) => format!("reset {} ({} -> {} can't convert)", f.name, o.describe(), f.describe()),
            None => format!("added {} (default)", f.name),
        });
    }
    for o in old {
        if !new.iter().any(|f| f.name == o.name) {
            report.push(format!("dropped {}", o.name));
        }
    }
    report.join(", ")
}
