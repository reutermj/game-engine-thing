//! The world: an entity/component store owned by the loader and shared by
//! every mod.
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

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ComponentId(pub u32);

impl ComponentId {
    /// The component can't be used by this build: the store has a newer layout
    /// for it. Every operation with it is a no-op.
    pub const INVALID: ComponentId = ComponentId(u32::MAX);
}

/// What a mod believes a component looks like. Components are matched by
/// name, and a mismatch in anything else is a layout change.
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
    /// `size` bytes of this build's `Default`, the starting point of every
    /// migrated value: fields the old layout lacks keep these bytes.
    pub default: *const u8,
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
}

impl FieldDesc {
    pub const fn new(name: &'static str, kind: FieldKind, offset: usize) -> FieldDesc {
        FieldDesc { name: name.as_ptr(), name_len: name.len(), kind, offset }
    }
}

/// A type a [`component!`] field may have.
///
/// # Safety
///
/// `KIND` must describe the type's in-memory representation exactly: the
/// loader reads and writes the field's bytes as that kind.
pub unsafe trait FieldType: Copy {
    const KIND: FieldKind;
}

macro_rules! field_types {
    ($($ty:ty => $kind:ident),* $(,)?) => {
        $(unsafe impl FieldType for $ty { const KIND: FieldKind = FieldKind::$kind; })*
    };
}

field_types! {
    u8 => U8, u16 => U16, u32 => U32, u64 => U64,
    i8 => I8, i16 => I16, i32 => I32, i64 => I64,
    f32 => F32, f64 => F64, bool => BOOL, Entity => ENTITY,
}

/// Declares a component whose values survive layout changes.
///
/// ```ignore
/// engine_api::component! {
///     #[derive(Debug, Default)]
///     pub struct Position: "game::Position" {
///         pub x: f32,
///         pub y: f32,
///     }
/// }
/// ```
///
/// Adds `Clone` and `Copy`, and implements [`Component`] with a schema, so
/// adding, removing, reordering or retyping (between numeric kinds) a field
/// keeps every value's other fields. New fields take their value from the
/// type's `Default`. Every field must be a [`FieldType`].
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
        #[derive(Clone, Copy)]
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        // SAFETY: every field is a `FieldType`, and those are plain data.
        unsafe impl $crate::Component for $name {
            const NAME: &'static str = $id;
            $(const VERSION: u32 = $version;)?
            const FIELDS: &'static [$crate::FieldDesc] = &[
                $($crate::FieldDesc::new(
                    stringify!($field),
                    <$ty as $crate::FieldType>::KIND,
                    ::std::mem::offset_of!($name, $field),
                )),*
            ];
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
    /// Also removes all of the entity's components.
    pub despawn: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity) -> bool,
    /// Copies `size` bytes from `value`, replacing any existing value.
    pub insert: unsafe extern "C" fn(
        ctx: *const ModContext,
        entity: Entity,
        id: ComponentId,
        value: *const u8,
    ) -> bool,
    pub remove: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity, id: ComponentId) -> bool,
    /// Null if the entity is dead or lacks the component.
    pub get: unsafe extern "C" fn(ctx: *const ModContext, entity: Entity, id: ComponentId) -> *mut u8,
    /// Valid until the next `insert`, `remove` or `despawn`.
    pub column: unsafe extern "C" fn(ctx: *const ModContext, id: ComponentId) -> Column,
}

/// A component type. Normally implemented by [`component!`].
///
/// # Safety
///
/// The value is copied into loader memory and outlives the build that wrote
/// it, so it must be plain data: no references, raw pointers or function
/// pointers into a mod (a `&'static str` or `fn` points into code a reload
/// unmaps), and nothing whose meaning depends on a mod's statics. `Copy`
/// rules out drop glue, which would also be code in an unmapped library.
/// `FIELDS`, if not empty, must describe every field exactly.
pub unsafe trait Component: Copy + Default + 'static {
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
        let default = T::default();
        let desc = ComponentDesc {
            name: T::NAME.as_ptr(),
            name_len: T::NAME.len(),
            size: size_of::<T>(),
            align: align_of::<T>(),
            version: T::VERSION,
            fields: T::FIELDS.as_ptr(),
            field_count: T::FIELDS.len(),
            default: &default as *const T as *const u8,
        };
        unsafe { (self.api().register)(self.ctx, &desc) }
    }

    pub fn spawn(&mut self) -> Entity {
        unsafe { (self.api().spawn)(self.ctx) }
    }

    pub fn despawn(&mut self, entity: Entity) -> bool {
        unsafe { (self.api().despawn)(self.ctx, entity) }
    }

    pub fn insert<T: Component>(&mut self, entity: Entity, value: T) -> bool {
        let id = self.id::<T>();
        unsafe { (self.api().insert)(self.ctx, entity, id, &value as *const T as *const u8) }
    }

    pub fn remove<T: Component>(&mut self, entity: Entity) -> bool {
        let id = self.id::<T>();
        unsafe { (self.api().remove)(self.ctx, entity, id) }
    }

    pub fn get<T: Component>(&self, entity: Entity) -> Option<&T> {
        let id = self.id::<T>();
        unsafe { ((self.api().get)(self.ctx, entity, id) as *const T).as_ref() }
    }

    pub fn get_mut<T: Component>(&mut self, entity: Entity) -> Option<&mut T> {
        let id = self.id::<T>();
        unsafe { ((self.api().get)(self.ctx, entity, id) as *mut T).as_mut() }
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
            let b = unsafe { (get(ctx, e, b) as *mut B).as_mut()? };
            Some((e, a, b))
        })
    }

    /// # Safety
    /// The slices alias loader memory; callers must hold `&mut self` for as
    /// long as they use them, which every public caller does.
    unsafe fn column<T: Component>(&self) -> (&'a [Entity], &'a mut [T]) {
        let id = self.id::<T>();
        let column = unsafe { (self.api().column)(self.ctx, id) };
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
