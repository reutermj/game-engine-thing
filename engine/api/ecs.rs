//! The world: an entity/component store owned by the loader and shared by
//! every mod. Components may hold heap data (`String`, `Vec`, ...): each
//! registration carries the code to drop and default them, and the loader
//! keeps that code's library mapped for as long as values need it.
//!
//! This is what makes reloads cheap to write for. Systems are mod code and get
//! swapped freely; components are data in the loader and never see a reload.
//! A mod's own state (`Mod`) still exists for the few things that are
//! genuinely private to it.
//!
//! The store is reached only through [`WorldApi`] function pointers. Mods never
//! see its Rust types, so a mod built against a different version of the store
//! can't misread its memory: the C ABI and `API_VERSION` are the whole contract.
//! See docs/architecture/ecs.md.

use std::marker::PhantomData;

use crate::ModContext;

/// Generational handle: a despawned entity's index is reused with a new
/// generation, so a stale handle never reaches a newer entity.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Entity {
    pub index: u32,
    pub generation: u32,
}

impl Entity {
    /// Never alive. Returned by `spawn` when the store couldn't be reached.
    pub const DEAD: Entity = Entity { index: u32::MAX, generation: u32::MAX };
}

/// `DEAD`: a component's `Entity` field defaults to referring to nothing,
/// which is also what a migration gives a newly added one.
impl Default for Entity {
    fn default() -> Self {
        Entity::DEAD
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ComponentId(pub u32);

impl ComponentId {
    /// The component can't be used by this build: the store has a newer layout
    /// for it. Every operation with it is a no-op.
    pub const INVALID: ComponentId = ComponentId(u32::MAX);
}

/// Frees the value (or field) at the pointer in place. Compiled into each
/// mod, so the loader can drop data whose type only mods know.
pub type DropFn = unsafe extern "C" fn(value: *mut u8);
/// Writes a `Default` value to the (uninitialized) pointer.
pub type DefaultFn = unsafe extern "C" fn(out: *mut u8);

/// What a mod believes a component looks like, and the code to manage its
/// values. Components are matched by name, and a mismatch in anything else is
/// a layout change.
#[repr(C)]
pub struct ComponentDesc {
    pub name: *const u8,
    pub name_len: usize,
    pub size: usize,
    pub align: usize,
    pub version: u32,
    /// The layout field by field, so the loader can migrate values to a new
    /// layout. Empty means no schema, and a layout change clears the values.
    pub fields: *const FieldDesc,
    pub field_count: usize,
    /// Drops a whole value; `None` if the type has nothing to drop.
    pub drop: Option<DropFn>,
    /// Makes this build's `Default`: the starting point of every migrated
    /// value, so fields the old layout lacks get their default.
    pub default: DefaultFn,
}

impl ComponentDesc {
    pub fn of<T: Component>() -> ComponentDesc {
        ComponentDesc {
            name: T::NAME.as_ptr(),
            name_len: T::NAME.len(),
            size: size_of::<T>(),
            align: align_of::<T>(),
            version: T::VERSION,
            fields: T::FIELDS.as_ptr(),
            field_count: T::FIELDS.len(),
            drop: __drop_fn::<T>(),
            default: __write_default::<T>,
        }
    }
}

#[doc(hidden)]
pub const fn __drop_fn<T>() -> Option<DropFn> {
    if std::mem::needs_drop::<T>() { Some(__drop::<T>) } else { None }
}

#[doc(hidden)]
pub unsafe extern "C" fn __drop<T>(value: *mut u8) {
    // A panicking drop leaks rather than unwinding into the loader.
    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe {
        std::ptr::drop_in_place(value as *mut T)
    }));
}

#[doc(hidden)]
pub unsafe extern "C" fn __write_default<T: Default>(out: *mut u8) {
    unsafe { (out as *mut T).write(T::default()) }
}

/// The kinds of field the loader can migrate. A plain integer for the same
/// reason as `Op`.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FieldKind(pub u32);

impl FieldKind {
    pub const U8: FieldKind = FieldKind(0);
    pub const U16: FieldKind = FieldKind(1);
    pub const U32: FieldKind = FieldKind(2);
    pub const U64: FieldKind = FieldKind(3);
    pub const I8: FieldKind = FieldKind(4);
    pub const I16: FieldKind = FieldKind(5);
    pub const I32: FieldKind = FieldKind(6);
    pub const I64: FieldKind = FieldKind(7);
    pub const F32: FieldKind = FieldKind(8);
    pub const F64: FieldKind = FieldKind(9);
    pub const BOOL: FieldKind = FieldKind(10);
    pub const ENTITY: FieldKind = FieldKind(11);
    /// Anything else (`String`, `Vec<T>`, a nested struct): the loader can't
    /// convert it, only move it into a field with the same `fingerprint`.
    pub const OPAQUE: FieldKind = FieldKind(12);
}

/// One field of a component's schema. Built at compile time by
/// [`component!`]; the loader copies what it keeps, since `name` points into
/// the mod.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FieldDesc {
    pub name: *const u8,
    pub name_len: usize,
    pub kind: FieldKind,
    pub offset: usize,
    pub size: usize,
    /// The field type's layout, recursively: equal fingerprints mean a value
    /// of one can be moved into the other byte for byte.
    pub fingerprint: u64,
    /// Drops just this field; `None` if it has nothing to drop.
    pub drop: Option<DropFn>,
}

impl FieldDesc {
    pub const fn new<T: FieldType>(name: &'static str, offset: usize) -> FieldDesc {
        FieldDesc {
            name: name.as_ptr(),
            name_len: name.len(),
            kind: T::KIND,
            offset,
            size: size_of::<T>(),
            fingerprint: T::FINGERPRINT,
            drop: __drop_fn::<T>(),
        }
    }
}

/// A type a [`component!`] field may have.
///
/// # Safety
///
/// A scalar `KIND` must describe the type's representation exactly: the
/// loader reads and writes the field's bytes as that kind. `FINGERPRINT` must
/// differ between any two types whose values can't be moved into each other
/// byte for byte. And the type must hold nothing that points into a mod's
/// image (code, statics, string literals): values outlive the build that made
/// them. Heap memory is fine, since every mod and the loader share one
/// allocator.
pub unsafe trait FieldType: Crossing + Clone + 'static {
    const KIND: FieldKind;
    const FINGERPRINT: u64;
}

/// A type that may be passed by value to or returned from a call between mods
/// (see `service!`): the other side may keep it, so it has to be safe to
/// outlive the build it came from. Every [`FieldType`] and every component is;
/// so is any reference, since a borrow can't outlive the call and both builds
/// are mapped while it runs.
///
/// A supertrait of `FieldType` rather than a blanket impl over it, which would
/// conflict with the impls for references.
///
/// # Safety
///
/// As for [`FieldType`], if the type isn't a reference: it must hold nothing
/// that points into a mod's image.
pub unsafe trait Crossing {}

unsafe impl<T: ?Sized> Crossing for &T {}
unsafe impl<T: ?Sized> Crossing for &mut T {}
unsafe impl Crossing for () {}

#[doc(hidden)]
pub const fn __fnv(mut hash: u64, bytes: &[u8]) -> u64 {
    let mut i = 0;
    while i < bytes.len() {
        hash = (hash ^ bytes[i] as u64).wrapping_mul(0x100000001b3);
        i += 1;
    }
    hash
}

const FNV_START: u64 = 0xcbf29ce484222325;

/// A fingerprint built from a name and the fingerprints it's made of.
#[doc(hidden)]
pub const fn __fingerprint(name: &str, parts: &[u64]) -> u64 {
    let mut hash = __fnv(FNV_START, name.as_bytes());
    let mut i = 0;
    while i < parts.len() {
        hash = __fnv(hash, &parts[i].to_le_bytes());
        i += 1;
    }
    hash
}

/// A struct's fingerprint: each field's name, offset and fingerprint.
#[doc(hidden)]
pub const fn __fingerprint_struct(fields: &[(&str, usize, u64)]) -> u64 {
    let mut hash = FNV_START;
    let mut i = 0;
    while i < fields.len() {
        hash = __fnv(hash, fields[i].0.as_bytes());
        hash = __fnv(hash, &(fields[i].1 as u64).to_le_bytes());
        hash = __fnv(hash, &fields[i].2.to_le_bytes());
        i += 1;
    }
    hash
}

macro_rules! scalar_field_types {
    ($($ty:ident => $kind:ident),* $(,)?) => {
        $(unsafe impl FieldType for $ty {
            const KIND: FieldKind = FieldKind::$kind;
            const FINGERPRINT: u64 = __fingerprint(stringify!($ty), &[]);
        }
        unsafe impl Crossing for $ty {})*
    };
}

scalar_field_types! {
    u8 => U8, u16 => U16, u32 => U32, u64 => U64,
    i8 => I8, i16 => I16, i32 => I32, i64 => I64,
    f32 => F32, f64 => F64, bool => BOOL, Entity => ENTITY,
}

// The std containers are laid out however this rustc lays them out; that is
// sound because every mod and the loader come from one toolchain, which the
// loader checks at load time.
// Plain data: seconds and nanoseconds, no pointers.
unsafe impl Crossing for std::time::Instant {}
unsafe impl FieldType for std::time::Instant {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Instant", &[]);
}

unsafe impl Crossing for std::time::Duration {}
unsafe impl FieldType for std::time::Duration {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Duration", &[]);
}

unsafe impl Crossing for String {}
unsafe impl FieldType for String {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("String", &[]);
}

unsafe impl<T: FieldType> Crossing for Vec<T> {}
unsafe impl<T: FieldType> FieldType for Vec<T> {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Vec", &[T::FINGERPRINT]);
}

unsafe impl<T: FieldType> Crossing for Option<T> {}
unsafe impl<T: FieldType> FieldType for Option<T> {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Option", &[T::FINGERPRINT]);
}

unsafe impl<T: FieldType> Crossing for Box<T> {}
unsafe impl<T: FieldType> FieldType for Box<T> {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Box", &[T::FINGERPRINT]);
}

unsafe impl<T: FieldType, const N: usize> Crossing for [T; N] {}
unsafe impl<T: FieldType, const N: usize> FieldType for [T; N] {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("Array", &[T::FINGERPRINT, N as u64]);
}

unsafe impl<K: FieldType + Eq + std::hash::Hash, V: FieldType> Crossing for std::collections::HashMap<K, V> {}
unsafe impl<K: FieldType + Eq + std::hash::Hash, V: FieldType> FieldType for std::collections::HashMap<K, V> {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("HashMap", &[K::FINGERPRINT, V::FINGERPRINT]);
}

unsafe impl<K: FieldType + Ord, V: FieldType> Crossing for std::collections::BTreeMap<K, V> {}
unsafe impl<K: FieldType + Ord, V: FieldType> FieldType for std::collections::BTreeMap<K, V> {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = __fingerprint("BTreeMap", &[K::FINGERPRINT, V::FINGERPRINT]);
}

macro_rules! tuple_field_types {
    ($(($($t:ident),+)),* $(,)?) => {
        $(unsafe impl<$($t: FieldType),+> Crossing for ($($t,)+) {}
        unsafe impl<$($t: FieldType),+> FieldType for ($($t,)+) {
            const KIND: FieldKind = FieldKind::OPAQUE;
            const FINGERPRINT: u64 = __fingerprint("Tuple", &[$($t::FINGERPRINT),+]);
        })*
    };
}

tuple_field_types! { (A, B), (A, B, C), (A, B, C, D) }

/// Declares a component whose values survive layout changes.
///
/// ```ignore
/// engine_api::component! {
///     #[derive(Debug, Default)]
///     pub struct Inventory: "rpg::Inventory" {
///         pub items: Vec<Item>,
///         pub owner: String,
///     }
/// }
/// ```
///
/// Adds `Clone` (add `Copy` yourself if every field is), and implements
/// [`Component`] with a schema, so adding, removing, reordering or retyping
/// (between numeric kinds) a field keeps every value's other fields. New
/// fields take their value from the type's `Default`; a field whose non-scalar
/// type changed (`Vec<u32>` to `Vec<u64>`) is reset to it. Every field must be
/// a [`FieldType`]: scalars, `Entity`, `String`, `Vec`, `Option`, `Box`,
/// arrays, `HashMap`, `BTreeMap`, and structs declared with [`field_struct!`].
///
/// When a change keeps the layout valid but not the meaning (a field now in
/// different units, say), bump the version so values are cleared instead:
/// `pub struct Position: "game::Position", version = 1 { ... }`.
#[macro_export]
macro_rules! component {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident : $id:literal $(, version = $version:literal)? {
            $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone)]
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        // SAFETY: every field is a `FieldType`, which rules out pointers into
        // the mod, and `FIELDS` is generated from the struct itself.
        unsafe impl $crate::Crossing for $name {}
        unsafe impl $crate::Component for $name {
            const NAME: &'static str = $id;
            $(const VERSION: u32 = $version;)?
            const FIELDS: &'static [$crate::FieldDesc] = &[
                $($crate::FieldDesc::new::<$ty>(stringify!($field), ::std::mem::offset_of!($name, $field))),*
            ];
        }
    };
}

/// Declares a mod's state: the data the loader carries from one build to the
/// next, so it follows the same rule as a component's fields (every field a
/// [`FieldType`]) and migrates the same way when its layout changes. Anything
/// that doesn't fit (a closure, a trait object, a crate's handle) goes in
/// [`Mod::Transient`](crate::Mod::Transient), which each build makes for itself.
///
/// ```ignore
/// engine_api::mod_state! {
///     #[derive(Default)]
///     struct Counter {
///         frames: u64,
///         count: u64,
///     }
/// }
/// ```
///
/// Bump the version (`struct Counter, version = 1 { ... }`) when a field's
/// meaning changes, to reset the state instead of migrating it.
#[macro_export]
macro_rules! mod_state {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident $(, version = $version:literal)? {
            $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)?
        }
    ) => {
        $(#[$meta])*
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        // SAFETY: every field is a `FieldType`, which rules out pointers into
        // the mod, and `FIELDS` is generated from the struct itself.
        unsafe impl $crate::ModState for $name {
            $(const VERSION: u32 = $version;)?
            const FIELDS: &'static [$crate::FieldDesc] = &[
                $($crate::FieldDesc::new::<$ty>(stringify!($field), ::std::mem::offset_of!($name, $field))),*
            ];
        }
    };
}

/// Declares a struct that can be a component's field (or a `Vec`'s element,
/// and so on): `Vec<Item>` in a component needs `Item` declared this way. Its
/// fingerprint covers its fields, so changing `Item` counts as changing the
/// type of every field that holds one.
#[macro_export]
macro_rules! field_struct {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone)]
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        // SAFETY: every field is a `FieldType`, and the fingerprint covers
        // each one's name, offset and fingerprint.
        unsafe impl $crate::Crossing for $name {}
        unsafe impl $crate::FieldType for $name {
            const KIND: $crate::FieldKind = $crate::FieldKind::OPAQUE;
            const FINGERPRINT: u64 = $crate::__fingerprint_struct(&[
                $((
                    stringify!($field),
                    ::std::mem::offset_of!($name, $field),
                    <$ty as $crate::FieldType>::FINGERPRINT,
                )),*
            ]);
        }
    };
}

/// One component's storage, densely packed: `data` holds `len` values in the
/// same order as `entities`.
#[repr(C)]
pub struct Column {
    pub entities: *const Entity,
    pub data: *mut u8,
    pub len: usize,
}

#[repr(C)]
pub struct WorldApi {
    /// Returns the component's id, creating its storage on first use. Called
    /// on every typed access, so it doubles as the layout check.
    pub register:
        unsafe extern "C" fn(ctx: *const ModContext, desc: *const ComponentDesc) -> ComponentId,
    pub spawn: unsafe extern "C" fn(ctx: *const ModContext) -> Entity,
    /// Also removes (and drops) all of the entity's components.
    pub despawn: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity) -> bool,
    /// Moves the value at `value` into the world, dropping any existing one.
    /// On `true` the world owns it; on `false` the caller still does.
    pub insert: unsafe extern "C" fn(
        ctx: *const ModContext,
        entity: Entity,
        id: ComponentId,
        value: *const u8,
    ) -> bool,
    /// Drops the entity's value.
    pub remove: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity, id: ComponentId) -> bool,
    /// Null if the entity is dead or lacks the component. `write` says
    /// whether the caller will write through it, which a running system must
    /// have declared.
    pub get: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity, id: ComponentId, write: bool) -> *mut u8,
    /// Valid until the next `insert`, `remove` or `despawn`.
    pub column: unsafe extern "C" fn(ctx: *const ModContext, id: ComponentId, write: bool) -> Column,
    /// Queues moving the value at `value` onto `entity`, applied at the end
    /// of the phase. On `true` the world owns it; on `false` the caller does.
    pub defer_insert: unsafe extern "C" fn(
        ctx: *const ModContext,
        entity: Entity,
        id: ComponentId,
        value: *const u8,
    ) -> bool,
    pub defer_remove: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity, id: ComponentId),
    pub defer_despawn: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity),
    /// Like `register`, for an event type. Events have their own ids.
    pub register_event:
        unsafe extern "C" fn(ctx: *const ModContext, desc: *const ComponentDesc) -> ComponentId,
    /// Moves the event at `value` into the world, visible from the next
    /// phase boundary. On `false` the caller still owns it.
    pub send_event: unsafe extern "C" fn(ctx: *const ModContext, id: ComponentId, value: *const u8) -> bool,
    /// The events the running system hasn't read yet, marking them read.
    /// Valid until the end of the phase. Empty outside a system.
    pub read_events: unsafe extern "C" fn(ctx: *const ModContext, id: ComponentId) -> EventSlice,
}

/// Events of one type, packed.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct EventSlice {
    pub data: *const u8,
    pub len: usize,
}

/// A component type. Normally implemented by [`component!`].
///
/// # Safety
///
/// Values move into loader memory and outlive the build that wrote them, so a
/// component must hold nothing that points into a mod's image: no references,
/// function pointers or trait objects (their vtables are in the mod), and no
/// `&'static str` (the string is in the mod). Heap data is fine. `FIELDS`, if
/// not empty, must describe every field exactly.
pub unsafe trait Component: Default + 'static {
    /// Stable identity across builds and mods. Two mods that name the same
    /// component share its storage.
    const NAME: &'static str;
    /// Bump when a field's meaning changes, whether or not the layout does.
    /// A version change clears values rather than migrating them.
    const VERSION: u32 = 0;
    const FIELDS: &'static [FieldDesc] = &[];
}

/// Typed access to the world for the duration of one mod callback.
///
/// Structural changes take `&mut self`, and queries borrow `self` mutably for
/// as long as they run, so the borrow checker keeps a query's references from
/// outliving the storage they point into.
pub struct World<'a> {
    ctx: *const ModContext,
    _cx: PhantomData<&'a mut ModContext>,
}

impl<'a> World<'a> {
    pub(crate) fn new(ctx: &'a mut ModContext) -> World<'a> {
        World { ctx, _cx: PhantomData }
    }

    fn api(&self) -> &WorldApi {
        unsafe { &(*(*self.ctx).host).world }
    }

    fn id<T: Component>(&self) -> ComponentId {
        unsafe { (self.api().register)(self.ctx, &ComponentDesc::of::<T>()) }
    }

    pub fn spawn(&mut self) -> Entity {
        unsafe { (self.api().spawn)(self.ctx) }
    }

    pub fn despawn(&mut self, entity: Entity) -> bool {
        unsafe { (self.api().despawn)(self.ctx, entity) }
    }

    /// Moves `value` into the world, replacing (and dropping) any existing one.
    /// If the entity is dead or the component unusable, `value` is dropped here.
    pub fn insert<T: Component>(&mut self, entity: Entity, value: T) -> bool {
        let id = self.id::<T>();
        let value = std::mem::ManuallyDrop::new(value);
        let moved = unsafe { (self.api().insert)(self.ctx, entity, id, &*value as *const T as *const u8) };
        if !moved {
            drop(std::mem::ManuallyDrop::into_inner(value));
        }
        moved
    }

    pub fn remove<T: Component>(&mut self, entity: Entity) -> bool {
        let id = self.id::<T>();
        unsafe { (self.api().remove)(self.ctx, entity, id) }
    }

    pub fn get<T: Component>(&self, entity: Entity) -> Option<&T> {
        let id = self.id::<T>();
        unsafe { ((self.api().get)(self.ctx, entity, id, false) as *const T).as_ref() }
    }

    pub fn get_mut<T: Component>(&mut self, entity: Entity) -> Option<&mut T> {
        let id = self.id::<T>();
        unsafe { ((self.api().get)(self.ctx, entity, id, true) as *mut T).as_mut() }
    }

    /// Every entity with a `T`.
    pub fn query<T: Component>(&mut self) -> impl Iterator<Item = (Entity, &mut T)> {
        let (entities, values) = unsafe { self.column::<T>() };
        entities.iter().copied().zip(values.iter_mut())
    }

    /// Every entity with both an `A` and a `B`, walking `A`'s storage. Put the
    /// rarer component first.
    pub fn query2<A: Component, B: Component>(
        &mut self,
    ) -> impl Iterator<Item = (Entity, &mut A, &mut B)> {
        // Same storage twice would hand out two `&mut` to one value.
        assert_ne!(A::NAME, B::NAME, "query2 needs two different components");
        let (entities, values) = unsafe { self.column::<A>() };
        let (ctx, get, b) = (self.ctx, self.api().get, self.id::<B>());
        entities.iter().copied().zip(values.iter_mut()).filter_map(move |(e, a)| {
            let b = unsafe { (get(ctx, e, b, true) as *mut B).as_mut()? };
            Some((e, a, b))
        })
    }

    /// # Safety
    /// The slices alias loader memory; callers must hold `&mut self` for as
    /// long as they use them, which every public caller does.
    unsafe fn column<T: Component>(&self) -> (&'a [Entity], &'a mut [T]) {
        let id = self.id::<T>();
        let column = unsafe { (self.api().column)(self.ctx, id, true) };
        if column.len == 0 {
            return (&[], &mut []);
        }
        unsafe {
            (
                std::slice::from_raw_parts(column.entities, column.len),
                std::slice::from_raw_parts_mut(column.data as *mut T, column.len),
            )
        }
    }
}
