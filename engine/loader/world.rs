//! The world behind `Host::world`: entities and component storage that outlive
//! every mod build.
//!
//! Components are untyped here: the loader knows each one's name, size and
//! alignment and nothing else, so no code from a mod is ever stored and a
//! reload can't leave the world pointing into an unmapped library. Storage is a
//! sparse set per component: values packed densely for queries, plus an
//! entity-index-to-slot map for lookups.

use std::alloc::Layout;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use engine_api::{Column, ComponentDesc, ComponentId, Entity, FieldKind, ModContext};

use crate::engine::{engine_of, name_of};

const EMPTY: u32 = u32::MAX;

#[derive(Default)]
pub struct WorldStorage {
    components: Vec<Storage>,
    by_name: HashMap<String, u32>,
    generations: Vec<u32>,
    alive: Vec<bool>,
    free: Vec<u32>,
}

struct Storage {
    name: String,
    size: usize,
    align: usize,
    version: u32,
    /// Empty when the component has no schema, which makes a layout change
    /// clear its values instead of migrating them.
    fields: Vec<Field>,
    /// `loaded_at` of the build that set the current layout. A build loaded
    /// later with a different layout replaces it; one loaded earlier is stale.
    layout_from: u64,
    /// Builds already told they're stale, so the warning isn't per frame.
    warned: HashSet<(String, u64)>,
    entities: Vec<Entity>,
    data: Values,
    /// Entity index to slot in `entities`/`data`, or `EMPTY`.
    slots: Vec<u32>,
}

impl WorldStorage {
    /// One line for `modctl list`: entity count, then each component's count.
    pub fn summary(&self) -> String {
        let entities = self.alive.iter().filter(|&&a| a).count();
        let components: Vec<String> =
            self.components.iter().map(|s| format!("{} x{}", s.name, s.entities.len())).collect();
        format!("world: {entities} entities; {}", components.join(", "))
    }

    fn is_alive(&self, e: Entity) -> bool {
        let i = e.index as usize;
        i < self.alive.len() && self.alive[i] && self.generations[i] == e.generation
    }

    fn register(&mut self, desc: &ComponentDesc, mod_name: &str, loaded_at: u64) -> ComponentId {
        let name = unsafe {
            std::str::from_utf8_unchecked(std::slice::from_raw_parts(desc.name, desc.name_len))
        };
        // Rust types always pass; this guards against a hand-written desc,
        // since a bad layout or an out-of-bounds field would panic or corrupt
        // memory later, inside an extern "C" fn.
        let Some(fields) = read_fields(desc) else {
            eprintln!("[engine] {mod_name}: component {name} has an invalid layout");
            return ComponentId::INVALID;
        };
        let Some(&id) = self.by_name.get(name) else {
            let id = self.components.len() as u32;
            self.components.push(Storage {
                name: name.into(),
                size: desc.size,
                align: desc.align,
                version: desc.version,
                fields,
                layout_from: loaded_at,
                warned: HashSet::new(),
                entities: Vec::new(),
                data: Values::new(desc.size, desc.align),
                slots: Vec::new(),
            });
            self.by_name.insert(name.into(), id);
            return ComponentId(id);
        };

        let s = &mut self.components[id as usize];
        if (s.size, s.align, s.version) == (desc.size, desc.align, desc.version) && s.fields == fields {
            return ComponentId(id);
        }
        if loaded_at > s.layout_from {
            // The newest build is the one the developer just changed, so it
            // wins. Its schema says where each surviving field now lives; a
            // version bump says the old values mean something else, so they
            // go.
            let migrate = s.version == desc.version && !s.fields.is_empty() && !fields.is_empty();
            if migrate {
                let default = unsafe { std::slice::from_raw_parts(desc.default, desc.size) };
                let report = s.migrate(desc.size, desc.align, &fields, default);
                println!(
                    "[engine] migrated {} value(s) of {name} to {mod_name}'s layout: {report}",
                    s.entities.len()
                );
            } else {
                let why = if s.version != desc.version { "its version changed" } else { "it has no schema" };
                println!(
                    "[engine] component {name} changed layout in {mod_name} and {why}; cleared {} value(s)",
                    s.entities.len()
                );
                for e in s.entities.drain(..) {
                    s.slots[e.index as usize] = EMPTY;
                }
                s.data = Values::new(desc.size, desc.align);
            }
            (s.size, s.align, s.version, s.fields) = (desc.size, desc.align, desc.version, fields);
            s.layout_from = loaded_at;
            return ComponentId(id);
        }
        if s.warned.insert((mod_name.into(), loaded_at)) {
            eprintln!(
                "[engine] {mod_name} was built with an older layout of {name}; \
                 its access to {name} is disabled until it's reloaded"
            );
        }
        ComponentId::INVALID
    }

    fn spawn(&mut self) -> Entity {
        if let Some(index) = self.free.pop() {
            self.alive[index as usize] = true;
            return Entity { index, generation: self.generations[index as usize] };
        }
        let index = self.alive.len() as u32;
        self.alive.push(true);
        self.generations.push(0);
        Entity { index, generation: 0 }
    }

    fn despawn(&mut self, e: Entity) -> bool {
        if !self.is_alive(e) {
            return false;
        }
        for s in &mut self.components {
            s.remove(e);
        }
        let i = e.index as usize;
        self.alive[i] = false;
        self.generations[i] = self.generations[i].wrapping_add(1);
        self.free.push(e.index);
        true
    }

    fn storage(&mut self, e: Entity, id: ComponentId) -> Option<&mut Storage> {
        if !self.is_alive(e) {
            return None;
        }
        self.components.get_mut(id.0 as usize)
    }
}

impl Storage {
    /// Rewrites every value into the new layout, matching fields by name.
    /// Returns what happened to each field, for the log.
    fn migrate(&mut self, size: usize, align: usize, fields: &[Field], default: &[u8]) -> String {
        let mut data = Values::new(size, align);
        for slot in 0..self.entities.len() {
            data.push_uninit();
            let (old, new) = (self.data.at(slot), data.at(slot));
            unsafe {
                std::ptr::copy_nonoverlapping(default.as_ptr(), new, size);
                for f in fields {
                    if let Some(o) = self.fields.iter().find(|o| o.name == f.name) {
                        convert(o.kind, old.add(o.offset), f.kind, new.add(f.offset));
                    }
                }
            }
        }
        self.data = data;

        let mut report = Vec::new();
        for f in fields {
            report.push(match self.fields.iter().find(|o| o.name == f.name) {
                Some(o) if o.kind == f.kind => format!("kept {}", f.name),
                Some(o) if is_numeric(o.kind) && is_numeric(f.kind) => {
                    format!("converted {} {} -> {}", f.name, kind_name(o.kind), kind_name(f.kind))
                }
                Some(o) => format!(
                    "reset {} ({} -> {} can't convert)",
                    f.name,
                    kind_name(o.kind),
                    kind_name(f.kind)
                ),
                None => format!("added {} (default)", f.name),
            });
        }
        for o in &self.fields {
            if !fields.iter().any(|f| f.name == o.name) {
                report.push(format!("dropped {}", o.name));
            }
        }
        report.join(", ")
    }

    fn slot(&self, e: Entity) -> Option<usize> {
        match self.slots.get(e.index as usize) {
            Some(&slot) if slot != EMPTY => Some(slot as usize),
            _ => None,
        }
    }

    fn insert(&mut self, e: Entity, value: *const u8) {
        let slot = match self.slot(e) {
            Some(slot) => slot,
            None => {
                let slot = self.entities.len();
                self.entities.push(e);
                self.data.push_uninit();
                if self.slots.len() <= e.index as usize {
                    self.slots.resize(e.index as usize + 1, EMPTY);
                }
                self.slots[e.index as usize] = slot as u32;
                slot
            }
        };
        unsafe { std::ptr::copy_nonoverlapping(value, self.data.at(slot), self.size) };
    }

    fn remove(&mut self, e: Entity) -> bool {
        let Some(slot) = self.slot(e) else { return false };
        let last = self.entities.len() - 1;
        if slot != last {
            let moved = self.entities[last];
            unsafe { std::ptr::copy_nonoverlapping(self.data.at(last), self.data.at(slot), self.size) };
            self.entities[slot] = moved;
            self.slots[moved.index as usize] = slot as u32;
        }
        self.entities.pop();
        self.data.len -= 1;
        self.slots[e.index as usize] = EMPTY;
        true
    }
}

/// A field of a component's schema, copied out of the mod's `FieldDesc`
/// because that points into the mod's memory.
#[derive(PartialEq)]
struct Field {
    name: String,
    kind: FieldKind,
    offset: usize,
}

/// The desc's fields, or `None` if the desc doesn't describe a valid layout.
fn read_fields(desc: &ComponentDesc) -> Option<Vec<Field>> {
    if !desc.align.is_power_of_two() || desc.size % desc.align != 0 {
        return None;
    }
    let descs = match desc.field_count {
        0 => &[][..],
        n => unsafe { std::slice::from_raw_parts(desc.fields, n) },
    };
    descs
        .iter()
        .map(|f| {
            let size = kind_size(f.kind)?;
            if f.offset.checked_add(size)? > desc.size {
                return None;
            }
            let name = unsafe { std::slice::from_raw_parts(f.name, f.name_len) };
            Some(Field { name: String::from_utf8(name.to_vec()).ok()?, kind: f.kind, offset: f.offset })
        })
        .collect()
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

fn kind_name(kind: FieldKind) -> &'static str {
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
        _ => "?",
    }
}

fn is_numeric(kind: FieldKind) -> bool {
    !matches!(kind, FieldKind::BOOL | FieldKind::ENTITY)
}

/// A numeric value wide enough to hold any numeric field exactly.
enum Num {
    Int(i128),
    Float(f64),
}

/// Copies one field from the old layout into the new, converting between
/// numeric kinds with `as` semantics (floats saturate into integers, integers
/// wrap into narrower ones). Leaves `dst` untouched, so holding the default,
/// when the kinds can't convert.
///
/// # Safety
/// `src` and `dst` must point to fields of the given kinds; both may be
/// unaligned.
unsafe fn convert(from: FieldKind, src: *const u8, to: FieldKind, dst: *mut u8) {
    if from == to {
        let size = kind_size(from).unwrap_or(0);
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

/// Packed, aligned bytes for one component's values. `Vec<u8>` can't be used:
/// its buffer is only byte-aligned.
struct Values {
    ptr: *mut u8,
    len: usize,
    cap: usize,
    size: usize,
    align: usize,
}

impl Values {
    fn new(size: usize, align: usize) -> Values {
        // A dangling but aligned pointer: what zero-sized values need, and what
        // `Column` hands out before the first allocation.
        let ptr = std::ptr::without_provenance_mut(align);
        Values { ptr, len: 0, cap: if size == 0 { usize::MAX } else { 0 }, size, align }
    }

    fn layout(&self, cap: usize) -> Layout {
        Layout::from_size_align(self.size * cap, self.align).expect("component layout")
    }

    fn push_uninit(&mut self) {
        if self.len == self.cap {
            let cap = (self.cap * 2).max(4);
            let ptr = unsafe {
                if self.cap == 0 {
                    std::alloc::alloc(self.layout(cap))
                } else {
                    std::alloc::realloc(self.ptr, self.layout(self.cap), self.size * cap)
                }
            };
            if ptr.is_null() {
                std::alloc::handle_alloc_error(self.layout(cap));
            }
            (self.ptr, self.cap) = (ptr, cap);
        }
        self.len += 1;
    }

    fn at(&self, slot: usize) -> *mut u8 {
        unsafe { self.ptr.add(slot * self.size) }
    }
}

impl Drop for Values {
    fn drop(&mut self) {
        if self.size != 0 && self.cap != 0 {
            unsafe { std::alloc::dealloc(self.ptr, self.layout(self.cap)) };
        }
    }
}

// Host callbacks. None of them may panic: unwinding out of an extern "C" fn
// aborts. The store is borrowed only for the length of one call, so a mod
// holding query results can still call back in.

fn with_world<R>(ctx: *const ModContext, fallback: R, f: impl FnOnce(&mut WorldStorage) -> R) -> R {
    let world: &RefCell<WorldStorage> = unsafe { engine_of(ctx) }.world();
    match world.try_borrow_mut() {
        Ok(mut world) => f(&mut world),
        Err(_) => fallback,
    }
}

pub unsafe extern "C" fn register(ctx: *const ModContext, desc: *const ComponentDesc) -> ComponentId {
    let (name, loaded_at) = unsafe { (name_of(ctx), (*ctx).loaded_at) };
    with_world(ctx, ComponentId::INVALID, |w| w.register(unsafe { &*desc }, name, loaded_at))
}

pub unsafe extern "C" fn spawn(ctx: *const ModContext) -> Entity {
    with_world(ctx, Entity::DEAD, |w| w.spawn())
}

pub unsafe extern "C" fn despawn(ctx: *const ModContext, e: Entity) -> bool {
    with_world(ctx, false, |w| w.despawn(e))
}

pub unsafe extern "C" fn insert(ctx: *const ModContext, e: Entity, id: ComponentId, value: *const u8) -> bool {
    with_world(ctx, false, |w| match w.storage(e, id) {
        Some(s) => {
            s.insert(e, value);
            true
        }
        None => false,
    })
}

pub unsafe extern "C" fn remove(ctx: *const ModContext, e: Entity, id: ComponentId) -> bool {
    with_world(ctx, false, |w| w.storage(e, id).is_some_and(|s| s.remove(e)))
}

pub unsafe extern "C" fn get(ctx: *const ModContext, e: Entity, id: ComponentId) -> *mut u8 {
    with_world(ctx, std::ptr::null_mut(), |w| match w.storage(e, id) {
        Some(s) => s.slot(e).map_or(std::ptr::null_mut(), |slot| s.data.at(slot)),
        None => std::ptr::null_mut(),
    })
}

pub unsafe extern "C" fn column(ctx: *const ModContext, id: ComponentId) -> Column {
    let empty = Column { entities: std::ptr::null(), data: std::ptr::null_mut(), len: 0 };
    with_world(ctx, empty, |w| match w.components.get(id.0 as usize) {
        Some(s) => Column { entities: s.entities.as_ptr(), data: s.data.ptr, len: s.entities.len() },
        None => Column { entities: std::ptr::null(), data: std::ptr::null_mut(), len: 0 },
    })
}
