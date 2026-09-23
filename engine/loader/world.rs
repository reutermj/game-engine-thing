//! The world behind `Host::world`: entities and component storage that outlive
//! every mod build.
//!
//! The loader knows each component by its layout and schema, never its Rust
//! type. What it can't do with bytes alone (drop a `String`, make a `Default`)
//! it does with code the registering build handed over, and it keeps that
//! build's library mapped for as long as values may need it: until a newer
//! build registers the component, or the world is dropped. So values survive
//! their mod being reloaded or unloaded. Storage is a sparse set per
//! component: values packed densely for queries, plus an entity-index-to-slot
//! map for lookups.

use std::alloc::Layout;
use std::any::Any;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use engine_api::{Column, Component, ComponentDesc, ComponentId, DefaultFn, DropFn, Entity, ModContext};

use crate::engine::{engine_of, name_of};
use crate::schema::{self, Field};

const EMPTY: u32 = u32::MAX;

/// Keeps mapped the library a build's code lives in. The world only holds it.
pub type Keepalive = Rc<dyn Any>;

#[derive(Default)]
pub struct WorldStorage {
    components: Vec<Storage>,
    by_name: HashMap<String, u32>,
    generations: Vec<u32>,
    alive: Vec<bool>,
    free: Vec<u32>,
}

/// The code that manages one component's values: the newest registered
/// build's, which has the storage's current layout.
struct Code {
    /// `ModContext::loaded_at` of that build. A build loaded later replaces
    /// this code (and, with another layout, the layout); one loaded earlier
    /// with another layout is stale.
    loaded_at: u64,
    drop: Option<DropFn>,
    default: DefaultFn,
    /// Parallel to `Storage::fields`.
    field_drops: Vec<Option<DropFn>>,
    /// The library the functions above are in. `None` only in unit tests,
    /// where they are in the test binary.
    _library: Option<Keepalive>,
}

struct Storage {
    name: String,
    size: usize,
    align: usize,
    version: u32,
    /// Empty when the component has no schema, which makes a layout change
    /// clear its values instead of migrating them.
    fields: Vec<Field>,
    code: Code,
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

    /// A clone of every value of `T`, for code running in the loader's own
    /// process (tests, and later tooling); mods go through `WorldApi`. `None`
    /// if `T` isn't registered or doesn't match the stored layout exactly, so
    /// a caller can't misread a layout it wasn't built for.
    pub fn values<T: Component + Clone>(&self) -> Option<Vec<(Entity, T)>> {
        let s = &self.components[*self.by_name.get(T::NAME)? as usize];
        let desc = ComponentDesc::of::<T>();
        let (fields, _) = read_fields(&desc)?;
        if (s.size, s.align, s.version) != (desc.size, desc.align, desc.version) || fields != s.fields {
            return None;
        }
        let values = (0..s.entities.len())
            .map(|slot| (s.entities[slot], unsafe { (*(s.data.at(slot) as *const T)).clone() }))
            .collect();
        Some(values)
    }

    fn is_alive(&self, e: Entity) -> bool {
        let i = e.index as usize;
        i < self.alive.len() && self.alive[i] && self.generations[i] == e.generation
    }

    /// `library` is only called when this build's code will be kept, which is
    /// rarely: registration runs on every typed access.
    fn register(
        &mut self,
        desc: &ComponentDesc,
        mod_name: &str,
        loaded_at: u64,
        library: impl FnOnce() -> Option<Keepalive>,
    ) -> ComponentId {
        let name = unsafe {
            std::str::from_utf8_unchecked(std::slice::from_raw_parts(desc.name, desc.name_len))
        };
        // Rust types always pass; this guards against a hand-written desc,
        // since a bad layout or an out-of-bounds field would panic or corrupt
        // memory later, inside an extern "C" fn.
        let Some((fields, field_drops)) = read_fields(desc) else {
            eprintln!("[engine] {mod_name}: component {name} has an invalid layout");
            return ComponentId::INVALID;
        };
        let code = |field_drops| Code {
            loaded_at,
            drop: desc.drop,
            default: desc.default,
            field_drops,
            _library: library(),
        };
        let Some(&id) = self.by_name.get(name) else {
            let id = self.components.len() as u32;
            self.components.push(Storage {
                name: name.into(),
                size: desc.size,
                align: desc.align,
                version: desc.version,
                fields,
                code: code(field_drops),
                warned: HashSet::new(),
                entities: Vec::new(),
                data: Values::new(desc.size, desc.align),
                slots: Vec::new(),
            });
            self.by_name.insert(name.into(), id);
            return ComponentId(id);
        };

        let s = &mut self.components[id as usize];
        let same_layout =
            (s.size, s.align, s.version) == (desc.size, desc.align, desc.version) && s.fields == fields;
        if loaded_at <= s.code.loaded_at && !same_layout {
            if s.warned.insert((mod_name.into(), loaded_at)) {
                eprintln!(
                    "[engine] {mod_name} was built with an older layout of {name}; \
                     its access to {name} is disabled until it's reloaded"
                );
            }
            return ComponentId::INVALID;
        }
        if loaded_at <= s.code.loaded_at {
            return ComponentId(id);
        }
        // A newer build than the one whose code the storage holds. With the
        // same layout, it takes over the code, and the older build's library
        // can be unmapped once nothing else holds it.
        let code = code(field_drops);
        if !same_layout {
            // The newest build is the one the developer just changed, so it
            // wins. Its schema says where each surviving field now lives; a
            // version bump says the old values mean something else, so they
            // go.
            let migrate = s.version == desc.version && !s.fields.is_empty() && !fields.is_empty();
            if migrate {
                let report = s.migrate(desc.size, desc.align, &fields, &code);
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
                s.clear();
                s.data = Values::new(desc.size, desc.align);
            }
            (s.size, s.align, s.version, s.fields) = (desc.size, desc.align, desc.version, fields);
        }
        s.code = code;
        ComponentId(id)
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
    /// Rewrites every value into the new layout (see `schema::migrate`).
    /// Returns what happened to each field, for the log.
    fn migrate(&mut self, size: usize, align: usize, fields: &[Field], code: &Code) -> String {
        let mut data = Values::new(size, align);
        for slot in 0..self.entities.len() {
            data.push_uninit();
            let (old, new) = (&self.fields[..], &self.code.field_drops[..]);
            unsafe {
                schema::migrate(
                    (self.data.at(slot), old, new),
                    (data.at(slot), fields, &code.field_drops, code.default),
                )
            };
        }
        // Every old value has been moved out or dropped, field by field, so
        // the old buffer is freed without dropping anything.
        self.data = data;
        schema::report(&self.fields, fields)
    }

    /// Drops every value.
    fn clear(&mut self) {
        for slot in 0..self.entities.len() {
            if let Some(drop) = self.code.drop {
                unsafe { drop(self.data.at(slot)) };
            }
        }
        for e in self.entities.drain(..) {
            self.slots[e.index as usize] = EMPTY;
        }
        self.data.len = 0;
    }

    fn slot(&self, e: Entity) -> Option<usize> {
        match self.slots.get(e.index as usize) {
            Some(&slot) if slot != EMPTY => Some(slot as usize),
            _ => None,
        }
    }

    /// Moves the value at `value` in, dropping the one it replaces.
    fn insert(&mut self, e: Entity, value: *const u8) {
        let slot = match self.slot(e) {
            Some(slot) => {
                if let Some(drop) = self.code.drop {
                    unsafe { drop(self.data.at(slot)) };
                }
                slot
            }
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
        if let Some(drop) = self.code.drop {
            unsafe { drop(self.data.at(slot)) };
        }
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

impl Drop for Storage {
    /// Drops the values while `code`'s library is still mapped: fields drop
    /// after this body runs.
    fn drop(&mut self) {
        self.clear();
    }
}

/// The desc's fields and their drop functions, or `None` if the desc doesn't
/// describe a valid layout.
fn read_fields(desc: &ComponentDesc) -> Option<(Vec<Field>, Vec<Option<DropFn>>)> {
    unsafe { schema::read_fields(desc.fields, desc.field_count, desc.size, desc.align) }
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
    let library = || unsafe { engine_of(ctx) }.library_of(ctx);
    with_world(ctx, ComponentId::INVALID, |w| w.register(unsafe { &*desc }, name, loaded_at, library))
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

#[cfg(test)]
mod tests {
    use engine_api::{Component, ComponentDesc, ComponentId, Entity, component};

    use super::WorldStorage;

    // Two builds of one component, as two mods (or two builds of one mod)
    // would define it. v2 reorders, widens `y`, drops `w`, retypes `flag` to a
    // kind that can't convert, and adds `z` with a non-zero default.
    mod v1 {
        engine_api::component! {
            #[derive(Debug, Default, PartialEq, Copy)]
            pub struct Pos: "test::Pos" {
                pub x: f32,
                pub y: f32,
                pub w: u32,
                pub flag: f32,
            }
        }
    }

    mod v2 {
        engine_api::component! {
            #[derive(Debug, PartialEq, Copy)]
            pub struct Pos: "test::Pos" {
                pub y: f64,
                pub x: f32,
                pub flag: bool,
                pub z: f32,
            }
        }

        impl Default for Pos {
            fn default() -> Self {
                Pos { y: 0.0, x: 0.0, flag: true, z: 7.0 }
            }
        }
    }

    /// v1 with a version bump: same fields, different meaning.
    mod v1_bumped {
        engine_api::component! {
            #[derive(Debug, Default, PartialEq, Copy)]
            pub struct Pos: "test::Pos", version = 1 {
                pub x: f32,
                pub y: f32,
                pub w: u32,
                pub flag: f32,
            }
        }
    }

    component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        struct Tag: "test::Tag" {}
    }

    mod u64x1 {
        engine_api::component! {
            #[derive(Default)]
            pub struct Wide: "test::Wide" {
                pub x: u64,
            }
        }
    }

    fn register<T: Component>(w: &mut WorldStorage, loaded_at: u64) -> ComponentId {
        w.register(&ComponentDesc::of::<T>(), "test", loaded_at, || None)
    }

    /// Moves `value` in, as `World::insert` does.
    fn insert<T: Component>(w: &mut WorldStorage, e: Entity, value: T, loaded_at: u64) {
        let id = register::<T>(w, loaded_at);
        let s = w.storage(e, id).expect("entity is alive and the component registered");
        let value = std::mem::ManuallyDrop::new(value);
        s.insert(e, &*value as *const T as *const u8);
    }

    fn values<T: Component + Clone>(w: &WorldStorage) -> Vec<(Entity, T)> {
        w.values::<T>().unwrap_or_else(|| panic!("{} isn't stored with this layout", T::NAME))
    }

    fn pos1(x: f32) -> v1::Pos {
        v1::Pos { x, y: x * 2.0, w: 3, flag: 1.0 }
    }

    #[test]
    fn a_despawned_handle_never_reaches_the_entity_that_reuses_its_index() {
        let mut w = WorldStorage::default();
        let old = w.spawn();
        insert(&mut w, old, pos1(1.0), 1);
        assert!(w.despawn(old));

        let new = w.spawn();
        assert_eq!(new.index, old.index, "the index should be reused");
        assert_ne!(new.generation, old.generation);
        insert(&mut w, new, pos1(2.0), 1);

        let id = register::<v1::Pos>(&mut w, 1);
        assert!(w.storage(old, id).is_none(), "the stale handle must not resolve");
        assert!(!w.despawn(old), "despawning a stale handle must not kill the new entity");
        assert_eq!(values::<v1::Pos>(&w), [(new, pos1(2.0))]);
    }

    #[test]
    fn despawn_removes_every_component() {
        let mut w = WorldStorage::default();
        let e = w.spawn();
        insert(&mut w, e, pos1(1.0), 1);
        insert(&mut w, e, Tag {}, 1);
        w.despawn(e);
        assert_eq!(values::<v1::Pos>(&w), []);
        assert_eq!(values::<Tag>(&w), []);
    }

    #[test]
    fn removal_keeps_every_other_value_reachable() {
        let mut w = WorldStorage::default();
        let es: Vec<Entity> = (0..4).map(|_| w.spawn()).collect();
        for (i, &e) in es.iter().enumerate() {
            insert(&mut w, e, pos1(i as f32), 1);
        }
        // Removing from the middle swaps the last value into the gap.
        let id = register::<v1::Pos>(&mut w, 1);
        assert!(w.storage(es[1], id).unwrap().remove(es[1]));
        assert!(!w.storage(es[1], id).unwrap().remove(es[1]), "already removed");

        let mut got = values::<v1::Pos>(&w);
        got.sort_by_key(|(e, _)| e.index);
        assert_eq!(got, [(es[0], pos1(0.0)), (es[2], pos1(2.0)), (es[3], pos1(3.0))]);
        // And each survivor's slot points back at it. Checking the value alone
        // isn't enough: a stale slot can still find the right bytes, since
        // removal doesn't overwrite them.
        for i in [0, 2, 3] {
            let s = w.storage(es[i], id).unwrap();
            let slot = s.slot(es[i]).unwrap();
            assert!(slot < s.entities.len() && s.entities[slot] == es[i], "slot of {:?} is stale", es[i]);
            assert_eq!(unsafe { *(s.data.at(slot) as *const v1::Pos) }, pos1(i as f32));
        }
    }

    #[test]
    fn inserting_again_replaces_the_value() {
        let mut w = WorldStorage::default();
        let e = w.spawn();
        insert(&mut w, e, pos1(1.0), 1);
        insert(&mut w, e, pos1(5.0), 1);
        assert_eq!(values::<v1::Pos>(&w), [(e, pos1(5.0))]);
    }

    #[test]
    fn zero_sized_components_are_stored() {
        let mut w = WorldStorage::default();
        let es: Vec<Entity> = (0..3).map(|_| w.spawn()).collect();
        for &e in &es {
            insert(&mut w, e, Tag {}, 1);
        }
        assert_eq!(values::<Tag>(&w).len(), 3);
    }

    #[test]
    fn a_newer_build_migrates_every_field_by_name() {
        let mut w = WorldStorage::default();
        let a = w.spawn();
        let b = w.spawn();
        insert(&mut w, a, v1::Pos { x: 1.5, y: 2.5, w: 9, flag: 1.0 }, 1);
        insert(&mut w, b, v1::Pos { x: -3.0, y: 4.25, w: 9, flag: 0.0 }, 1);

        assert_ne!(register::<v2::Pos>(&mut w, 2), ComponentId::INVALID);
        assert_eq!(
            values::<v2::Pos>(&w),
            [
                // x kept, y widened, w dropped, flag reset to v2's default
                // (f32 -> bool doesn't convert), z from v2's default.
                (a, v2::Pos { y: 2.5, x: 1.5, flag: true, z: 7.0 }),
                (b, v2::Pos { y: 4.25, x: -3.0, flag: true, z: 7.0 }),
            ]
        );
    }

    #[test]
    fn an_older_build_is_cut_off_until_it_is_reloaded() {
        let mut w = WorldStorage::default();
        let e = w.spawn();
        insert(&mut w, e, pos1(1.0), 1);
        register::<v2::Pos>(&mut w, 2);

        // Still-loaded build from before the change: not allowed to read or
        // write v2's bytes as v1's.
        assert_eq!(register::<v1::Pos>(&mut w, 1), ComponentId::INVALID);
        assert_eq!(w.components[0].warned.len(), 1);
        register::<v1::Pos>(&mut w, 1);
        assert_eq!(w.components[0].warned.len(), 1, "warn once per build, not per access");

        // The same v1 source reloaded later is the newest build, so it wins.
        assert_ne!(register::<v1::Pos>(&mut w, 3), ComponentId::INVALID);
        assert_eq!(values::<v1::Pos>(&w), [(e, v1::Pos { x: 1.0, y: 2.0, w: 0, flag: 0.0 })]);
    }

    #[test]
    fn a_version_bump_clears_instead_of_migrating() {
        let mut w = WorldStorage::default();
        let e = w.spawn();
        insert(&mut w, e, pos1(1.0), 1);
        register::<v1_bumped::Pos>(&mut w, 2);
        assert_eq!(values::<v1_bumped::Pos>(&w), []);
        assert!(w.storage(e, ComponentId(0)).unwrap().slot(e).is_none(), "slots must be cleared too");
    }

    #[test]
    fn a_component_without_a_schema_clears_on_layout_change() {
        #[derive(Clone, Copy, Default)]
        struct Raw(u32);
        // SAFETY: plain data.
        unsafe impl Component for Raw {
            const NAME: &'static str = "test::Raw";
        }
        #[derive(Clone, Copy, Default)]
        struct RawWider(u64);
        unsafe impl Component for RawWider {
            const NAME: &'static str = "test::Raw";
        }

        let mut w = WorldStorage::default();
        let es: Vec<Entity> = (0..3).map(|_| w.spawn()).collect();
        insert(&mut w, es[0], Raw(5), 1);
        assert_eq!(values::<Raw>(&w)[0].1.0, 5);
        register::<RawWider>(&mut w, 2);
        assert_eq!(values::<RawWider>(&w).len(), 0);
        // Several values: with a buffer still laid out for `Raw`, wider values
        // would overlap each other.
        for (i, &e) in es.iter().enumerate() {
            insert(&mut w, e, RawWider(u64::MAX - i as u64), 2);
        }
        let got: Vec<u64> = values::<RawWider>(&w).iter().map(|(_, v)| v.0).collect();
        assert_eq!(got, [u64::MAX, u64::MAX - 1, u64::MAX - 2], "the new layout is in use");
    }

    #[test]
    fn numeric_conversion_follows_as_semantics() {
        use crate::schema::convert;
        use engine_api::FieldKind as K;
        fn run<A: Copy, B: Copy + Default>(from: K, value: A, to: K) -> B {
            let mut out = B::default();
            unsafe {
                convert(from, &value as *const A as *const u8, size_of::<A>(), to, &mut out as *mut B as *mut u8)
            };
            out
        }
        assert_eq!(run::<f32, f64>(K::F32, 2.5, K::F64), 2.5);
        assert_eq!(run::<f64, i32>(K::F64, 1e20, K::I32), i32::MAX, "floats saturate");
        assert_eq!(run::<f32, u8>(K::F32, -4.0, K::U8), 0, "floats saturate at zero");
        assert_eq!(run::<f32, i32>(K::F32, f32::NAN, K::I32), 0);
        assert_eq!(run::<i32, u8>(K::I32, 300, K::U8), 44, "integers wrap");
        assert_eq!(run::<i64, f32>(K::I64, -3, K::F32), -3.0);
        assert_eq!(run::<u64, i64>(K::U64, u64::MAX, K::I64), -1);
        // Across kinds nothing is written, so the destination keeps its default.
        assert_eq!(run::<f32, u8>(K::F32, 1.0, K::BOOL), 0);
        assert_eq!(run::<bool, u32>(K::BOOL, true, K::U32), 0);
    }

    #[test]
    fn a_desc_that_does_not_describe_its_type_is_rejected() {
        use engine_api::{FieldDesc, FieldKind};
        let mut w = WorldStorage::default();
        let good = || ComponentDesc {
            name: "test::Bad".as_ptr(),
            name_len: "test::Bad".len(),
            fields: std::ptr::null(),
            field_count: 0,
            ..ComponentDesc::of::<u64x1::Wide>()
        };
        let field = |kind, offset, size| FieldDesc { kind, offset, size, ..FieldDesc::new::<u64>("x", 0) };
        let out_of_bounds = [field(FieldKind::U64, 4, 8)];
        let unknown_kind = [field(FieldKind(99), 0, 8)];
        let wrong_size = [field(FieldKind::U32, 0, 8)];
        let opaque_out_of_bounds = [field(FieldKind::OPAQUE, 0, 16)];
        let cases = [
            ComponentDesc { align: 3, ..good() },
            ComponentDesc { size: 12, ..good() },
            ComponentDesc { fields: out_of_bounds.as_ptr(), field_count: 1, ..good() },
            ComponentDesc { fields: unknown_kind.as_ptr(), field_count: 1, ..good() },
            ComponentDesc { fields: wrong_size.as_ptr(), field_count: 1, ..good() },
            ComponentDesc { fields: opaque_out_of_bounds.as_ptr(), field_count: 1, ..good() },
        ];
        for desc in cases {
            assert_eq!(w.register(&desc, "test", 1, || None), ComponentId::INVALID);
        }
        assert!(w.components.is_empty(), "nothing should have been registered");
        assert_ne!(w.register(&good(), "test", 1, || None), ComponentId::INVALID);
    }

    // Heap-owning components. `Tracked` counts its drops, so tests can check
    // that every value is dropped exactly once, by whichever path.

    use std::cell::Cell;
    use std::rc::Rc;

    #[derive(Clone, Default)]
    struct Tracked(Rc<Cell<u32>>);

    impl Drop for Tracked {
        fn drop(&mut self) {
            self.0.set(self.0.get() + 1);
        }
    }

    // SAFETY: an `Rc` to heap memory; nothing in the test binary's image.
    unsafe impl engine_api::Crossing for Tracked {}
    unsafe impl engine_api::FieldType for Tracked {
        const KIND: engine_api::FieldKind = engine_api::FieldKind::OPAQUE;
        const FINGERPRINT: u64 = engine_api::__fingerprint("Tracked", &[]);
    }

    component! {
        #[derive(Default)]
        struct Owner: "test::Owner" {
            name: String,
            tracked: Tracked,
        }
    }

    fn owner(name: &str, drops: &Rc<Cell<u32>>) -> Owner {
        Owner { name: name.into(), tracked: Tracked(drops.clone()) }
    }

    #[test]
    fn every_heap_value_is_dropped_exactly_once() {
        let drops = Rc::new(Cell::new(0));
        let mut w = WorldStorage::default();
        let es: Vec<Entity> = (0..4).map(|_| w.spawn()).collect();
        for &e in &es {
            insert(&mut w, e, owner("x", &drops), 1);
        }
        insert(&mut w, es[0], owner("replacement", &drops), 1);
        assert_eq!(drops.get(), 1, "replacing drops the old value");
        let id = register::<Owner>(&mut w, 1);
        w.storage(es[1], id).unwrap().remove(es[1]);
        assert_eq!(drops.get(), 2, "remove drops");
        w.despawn(es[2]);
        assert_eq!(drops.get(), 3, "despawn drops");

        // The two left have their own heap data, intact.
        let mut names: Vec<String> = values::<Owner>(&w).into_iter().map(|(_, o)| o.name).collect();
        names.sort();
        assert_eq!(names, ["replacement", "x"]);
        // `values` clones, and the clones drop when the test drops them.
        let dropped = drops.get();

        drop(w);
        assert_eq!(drops.get(), dropped + 2, "the world drops what's left");
    }

    #[test]
    fn a_version_bump_drops_the_old_values() {
        mod bumped {
            engine_api::component! {
                #[derive(Default)]
                pub struct Owner: "test::Owner", version = 1 {
                    pub name: String,
                    pub tracked: super::Tracked,
                }
            }
        }
        let drops = Rc::new(Cell::new(0));
        let mut w = WorldStorage::default();
        for _ in 0..3 {
            let e = w.spawn();
            insert(&mut w, e, owner("x", &drops), 1);
        }
        register::<bumped::Owner>(&mut w, 2);
        assert_eq!(drops.get(), 3);
        assert_eq!(values::<bumped::Owner>(&w).len(), 0);
    }

    mod heap_v1 {
        engine_api::field_struct! {
            #[derive(Debug, PartialEq)]
            pub struct Item {
                pub id: u32,
            }
        }

        engine_api::component! {
            #[derive(Default)]
            pub struct Bag: "test::Bag" {
                pub label: String,
                pub counts: Vec<u32>,
                pub items: Vec<Item>,
                pub gone: super::Tracked,
            }
        }
    }

    // Reordered; `counts` retyped to Vec<u64> (can't convert); `Item` gained
    // a field (so `items` is a different type); `gone` removed; `extra` added
    // with a default that owns heap memory.
    mod heap_v2 {
        engine_api::field_struct! {
            #[derive(Debug, PartialEq)]
            pub struct Item {
                pub id: u32,
                pub weight: u32,
            }
        }

        engine_api::component! {
            pub struct Bag: "test::Bag" {
                pub extra: Vec<u8>,
                pub counts: Vec<u64>,
                pub items: Vec<Item>,
                pub label: String,
            }
        }

        impl Default for Bag {
            fn default() -> Self {
                Bag { extra: vec![1, 2, 3], counts: vec![], items: vec![], label: String::new() }
            }
        }
    }

    #[test]
    fn heap_fields_move_intact_and_changed_ones_drop_and_reset() {
        let drops = Rc::new(Cell::new(0));
        let mut w = WorldStorage::default();
        let es: Vec<Entity> = (0..2).map(|_| w.spawn()).collect();
        for (i, &e) in es.iter().enumerate() {
            let bag = heap_v1::Bag {
                label: format!("bag {i}"),
                counts: vec![7; 3],
                items: vec![heap_v1::Item { id: i as u32 }],
                gone: Tracked(drops.clone()),
            };
            insert(&mut w, e, bag, 1);
        }

        register::<heap_v2::Bag>(&mut w, 2);
        assert_eq!(drops.get(), 2, "the removed field was dropped, once per value");
        let mut got = values::<heap_v2::Bag>(&w);
        let labels: Vec<&str> = got.iter().map(|(_, b)| b.label.as_str()).collect();
        assert_eq!(labels, ["bag 0", "bag 1"], "an unchanged heap field moves intact");
        for (_, bag) in &got {
            assert!(bag.counts.is_empty(), "Vec<u32> -> Vec<u64> can't convert, so it resets");
            assert!(bag.items.is_empty(), "Item changed, so Vec<Item> is a new type and resets");
            assert_eq!(bag.extra, [1, 2, 3], "a new field comes from the new Default");
        }

        // Each value got its own default, not a copy of one buffer: changing
        // one leaves the other alone (and dropping both doesn't free twice).
        let id = register::<heap_v2::Bag>(&mut w, 2);
        let s = w.storage(es[0], id).unwrap();
        let slot = s.slot(es[0]).unwrap();
        unsafe { (*(s.data.at(slot) as *mut heap_v2::Bag)).extra.push(4) };
        got = values::<heap_v2::Bag>(&w);
        assert_eq!(got[0].1.extra, [1, 2, 3, 4]);
        assert_eq!(got[1].1.extra, [1, 2, 3]);
    }

    #[test]
    fn a_nested_struct_change_changes_the_fingerprint() {
        use engine_api::FieldType;
        assert_ne!(heap_v1::Item::FINGERPRINT, heap_v2::Item::FINGERPRINT);
        assert_ne!(<Vec<heap_v1::Item>>::FINGERPRINT, <Vec<heap_v2::Item>>::FINGERPRINT);
        assert_ne!(<Vec<u32>>::FINGERPRINT, <Vec<u64>>::FINGERPRINT);
        assert_ne!(<Vec<u32>>::FINGERPRINT, <Option<u32>>::FINGERPRINT);
        assert_eq!(<Vec<heap_v1::Item>>::FINGERPRINT, <Vec<heap_v1::Item>>::FINGERPRINT);
    }

    #[test]
    fn the_newest_builds_library_is_kept_and_older_ones_released() {
        let mut w = WorldStorage::default();
        let e = w.spawn();
        let (older, newer): (Rc<()>, Rc<()>) = (Rc::new(()), Rc::new(()));
        let keep = |lib: &Rc<()>| {
            let lib = lib.clone();
            move || Some(lib as super::Keepalive)
        };
        let desc = ComponentDesc::of::<Owner>();
        w.register(&desc, "a", 1, keep(&older));
        let drops = Rc::new(Cell::new(0));
        insert(&mut w, e, owner("x", &drops), 1);
        assert_eq!(Rc::strong_count(&older), 2, "the world holds the build it has code from");

        // The same build again, or an older one, changes nothing.
        w.register(&desc, "a", 1, keep(&newer));
        assert_eq!(Rc::strong_count(&newer), 1);

        // A newer build takes over the code; the older library is released.
        w.register(&desc, "a", 2, keep(&newer));
        assert_eq!((Rc::strong_count(&older), Rc::strong_count(&newer)), (1, 2));
        drop(w);
        assert_eq!(Rc::strong_count(&newer), 1, "dropping the world releases it");
        assert_eq!(drops.get(), 1, "after dropping the value with its code");
    }
}
