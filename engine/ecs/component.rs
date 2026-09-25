//! Components and their schemas: what a component is, as mods declare it,
//! and the field-by-field description that lets values migrate between
//! builds whose layouts differ. See docs/architecture/ecs.md.


/// Generational handle: a despawned entity's index is reused with a new
/// generation, so a stale handle never reaches a newer entity.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
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

/// Frees the value (or field) at the pointer in place. Compiled into each
/// mod, so the loader can drop data whose type only mods know.
pub type DropFn = unsafe extern "C" fn(value: *mut u8);
/// Writes a `Default` value to the (uninitialized) pointer.
pub type DefaultFn = unsafe extern "C" fn(out: *mut u8);

/// How a component's values are stored: packed in archetype tables (the
/// default), or in a set of their own, for components added and removed
/// often. See docs/architecture/storage.md.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Storage {
    Table,
    Sparse,
}

/// What a mod believes a component looks like, and the code to manage its
/// values. Components are matched by name, and a mismatch in anything else is
/// a layout change. Made only by [`ComponentDesc::of`], so it always
/// describes a real `Component`.
#[derive(Clone, Copy)]
pub struct ComponentDesc {
    pub name: &'static str,
    pub storage: Storage,
    /// The schema's fingerprint: equal ones mean equal layouts and meaning.
    pub fingerprint: u64,
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
    /// For a spatial key, its extent and bounds glue.
    pub spatial: Option<crate::spatial::SpatialDesc>,
    /// For an ordered key, its key glue.
    pub order: Option<crate::ordered::OrderDesc>,
}

impl ComponentDesc {
    pub fn of<T: Component>() -> ComponentDesc {
        ComponentDesc {
            name: T::NAME,
            storage: T::STORAGE,
            fingerprint: T::FINGERPRINT,
            size: size_of::<T>(),
            align: align_of::<T>(),
            version: T::VERSION,
            fields: T::FIELDS.as_ptr(),
            field_count: T::FIELDS.len(),
            drop: __drop_fn::<T>(),
            default: __write_default::<T>,
            spatial: T::SPATIAL,
            order: T::ORDER,
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

/// A component's fingerprint: its name, version, layout and every field's
/// name, offset, kind and fingerprint. Two builds agree on a component's
/// values exactly when their fingerprints are equal.
#[doc(hidden)]
pub const fn __component_fingerprint(name: &str, version: u32, size: usize, align: usize, fields: &[FieldDesc]) -> u64 {
    let mut hash = __fnv(FNV_START, name.as_bytes());
    hash = __fnv(hash, &(version as u64).to_le_bytes());
    hash = __fnv(hash, &(size as u64).to_le_bytes());
    hash = __fnv(hash, &(align as u64).to_le_bytes());
    let mut i = 0;
    while i < fields.len() {
        let f = &fields[i];
        // SAFETY: a `FieldDesc`'s name is always a `&'static str`'s bytes.
        let field_name = unsafe { std::slice::from_raw_parts(f.name, f.name_len) };
        hash = __fnv(hash, field_name);
        hash = __fnv(hash, &(f.offset as u64).to_le_bytes());
        hash = __fnv(hash, &(f.kind.0 as u64).to_le_bytes());
        hash = __fnv(hash, &f.fingerprint.to_le_bytes());
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
/// `pub struct Position: "game::Position", version = 1 { ... }`. A component
/// added and removed often is better sparse: `pub struct Burning:
/// "game::Burning", storage = sparse { ... }`. A position whose tables should
/// be kept in spatial order says `order = spatial`, and implements
/// [`SpatialKey`](crate::spatial::SpatialKey). One whose tables should be
/// kept sorted by it says `order = key`, and implements
/// [`OrderKey`](crate::ordered::OrderKey).
#[macro_export]
macro_rules! component {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident : $id:literal $(, version = $version:literal)? $(, storage = $storage:ident)? $(, order = $order:ident)? {
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
            $(const STORAGE: $crate::Storage = $crate::__storage!($storage);)?
            $(
                const SPATIAL: ::std::option::Option<$crate::spatial::SpatialDesc> = $crate::__spatial!($order, $name);
                const ORDER: ::std::option::Option<$crate::ordered::OrderDesc> = $crate::__keyed!($order, $name);
            )?
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


/// A component type. Normally implemented by [`component!`].
///
/// # Safety
///
/// Values move into loader memory and outlive the build that wrote them, so a
/// component must hold nothing that points into a mod's image: no references,
/// function pointers or trait objects (their vtables are in the mod), and no
/// `&'static str` (the string is in the mod). Heap data is fine. `FIELDS`, if
/// not empty, must describe every field exactly.
pub unsafe trait Component: Default + Send + Sync + 'static {
    /// Stable identity across builds and mods. Two mods that name the same
    /// component share its storage.
    const NAME: &'static str;
    /// Bump when a field's meaning changes, whether or not the layout does.
    /// A version change clears values rather than migrating them.
    const VERSION: u32 = 0;
    const FIELDS: &'static [FieldDesc] = &[];
    const STORAGE: Storage = Storage::Table;
    /// For a spatial key (`order = spatial`, with `SpatialKey`
    /// implemented), what keeps its tables in spatial order.
    const SPATIAL: Option<crate::spatial::SpatialDesc> = None;
    /// For an ordered key (`order = key`, with `OrderKey` implemented),
    /// what keeps its tables sorted.
    const ORDER: Option<crate::ordered::OrderDesc> = None;
    /// Checked on every typed access to stored values, so a build can't
    /// read a layout it wasn't compiled for.
    const FINGERPRINT: u64 =
        __component_fingerprint(Self::NAME, Self::VERSION, size_of::<Self>(), align_of::<Self>(), Self::FIELDS);
}


#[doc(hidden)]
#[macro_export]
macro_rules! __spatial {
    (spatial, $name:ident) => {
        ::std::option::Option::Some($crate::spatial::SpatialDesc::of::<$name>())
    };
    (key, $name:ident) => {
        ::std::option::Option::None
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __keyed {
    (spatial, $name:ident) => {
        ::std::option::Option::None
    };
    (key, $name:ident) => {
        ::std::option::Option::Some($crate::ordered::OrderDesc::of::<$name>())
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __storage {
    (table) => {
        $crate::Storage::Table
    };
    (sparse) => {
        $crate::Storage::Sparse
    };
}
