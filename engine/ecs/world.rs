//! The world: entities, archetype tables whose rows live in pages, sparse
//! sets, and event queues. Safe Rust over `ErasedColumn`, except for the
//! schema migration, which is `schema.rs`'s.
//!
//! Every piece of shared state a structural change touches has its own guard,
//! so changes to disjoint tables proceed at once:
//!
//! - a table's rows (the entities, by page) and each of its columns are
//!   separate `RwLock`s;
//! - each sparse component's set, and each event queue, is one `RwLock`;
//! - entity locations and generations are atomics, one per entity;
//! - components, tables and queues live in arenas that only grow, and
//!   nothing in them moves once made, so they're found without a lock.
//!
//! Guards are taken with `try_*`: the scheduler only starts work whose
//! guards are free, so a guard that isn't is a scheduler bug, and panics.
//!
//! Components are registered in two steps. `intern` gives a name its id
//! when a build declares a system (a load not yet committed); `install`
//! gives it a layout and the build's code when the load commits, migrating
//! stored values if the layout changed. Both happen between frames.

use std::any::Any;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard, TryLockError};

use crate::component::{ComponentDesc, DefaultFn, DropFn, Entity, Storage};
use crate::erased::{ErasedColumn, ValueType, drop_value};
use crate::events::EventQueue;
use crate::par::Executor;
use crate::schema::{self, Field};
use crate::ordered::{self, KeyOrder, OrderDesc};
use crate::spatial::{Resort, SPATIAL_PAGE_ROWS, SpatialDesc, SpatialPages};

/// Rows per page: the unit of borrowing for data parallelism, and where a
/// table grows.
pub const PAGE_ROWS: usize = 256;
const MAX_COMPONENTS: usize = 4096;
const MAX_TABLES: usize = 4096;
pub(crate) const MAX_EVENTS: usize = 1024;
/// Entities are allocated in segments of this many, on demand.
const SEGMENT: usize = 4096;
const SEGMENTS: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ComponentId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TableId(pub u32);

/// Keeps mapped the library a build's code lives in. The world only holds it.
pub type Keepalive = Arc<dyn Any + Send + Sync>;

/// Who is registering: a build, for its code's sake. `loaded_at` orders
/// builds (a newer one's layout wins); `keepalive` keeps its code mapped
/// while stored values need it.
#[derive(Clone, Default)]
pub struct Build {
    pub name: String,
    pub loaded_at: u64,
    pub keepalive: Option<Keepalive>,
}

/// Values fixed in a slot the first time it's filled, found without a lock.
pub(crate) struct Arena<T> {
    slots: Box<[OnceLock<T>]>,
    count: AtomicUsize,
}

impl<T> Arena<T> {
    fn new(capacity: usize) -> Arena<T> {
        Arena { slots: (0..capacity).map(|_| OnceLock::new()).collect(), count: AtomicUsize::new(0) }
    }

    /// Callers serialize pushes (each arena has a mutex beside it).
    pub(crate) fn push(&self, value: T) -> usize {
        let i = self.count.load(Ordering::Acquire);
        assert!(i < self.slots.len(), "arena full");
        assert!(self.slots[i].set(value).is_ok(), "arena slot reused");
        // Published after it's made, so a reader never sees a slot unset.
        self.count.store(i + 1, Ordering::Release);
        i
    }

    pub(crate) fn get(&self, i: usize) -> &T {
        self.slots[i].get().expect("an arena index that was pushed")
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &T> {
        let n = self.count.load(Ordering::Acquire);
        self.slots[..n].iter().map(|s| s.get().expect("counted slots are set"))
    }
}

/// Where an entity's row is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Location {
    pub table: TableId,
    pub page: u32,
    pub row: u32,
}

const DEAD: u64 = u64::MAX;
/// Spawned by a system, not yet placed in a table by its apply node.
const RESERVED: u64 = u64::MAX - 1;

impl Location {
    fn pack(self) -> u64 {
        (self.table.0 as u64) << 40 | (self.page as u64) << 16 | self.row as u64
    }
    fn unpack(bits: u64) -> Option<Location> {
        (bits < RESERVED).then(|| Location {
            table: TableId((bits >> 40) as u32),
            page: ((bits >> 16) & 0xff_ffff) as u32,
            row: (bits & 0xffff) as u32,
        })
    }
}

struct Segment {
    generations: Box<[AtomicU32]>,
    locations: Box<[AtomicU64]>,
}

/// Entity ids, generations and locations. Each entity's slot is its own
/// atomic, so changes to different entities never contend.
pub struct Entities {
    segments: Box<[OnceLock<Segment>]>,
    next: AtomicU32,
    free: Mutex<Vec<u32>>,
    alive: AtomicUsize,
}

impl Entities {
    fn new() -> Entities {
        Entities {
            segments: (0..SEGMENTS).map(|_| OnceLock::new()).collect(),
            next: AtomicU32::new(0),
            free: Mutex::new(Vec::new()),
            alive: AtomicUsize::new(0),
        }
    }

    fn slot(&self, index: u32) -> Option<(&AtomicU32, &AtomicU64)> {
        let (s, i) = (index as usize / SEGMENT, index as usize % SEGMENT);
        let segment = self.segments.get(s)?.get()?;
        Some((&segment.generations[i], &segment.locations[i]))
    }

    /// A new id, not yet in any table.
    pub fn reserve(&self) -> Entity {
        let index = self.free.lock().unwrap().pop().unwrap_or_else(|| {
            let index = self.next.fetch_add(1, Ordering::Relaxed);
            let s = index as usize / SEGMENT;
            assert!(s < SEGMENTS, "out of entities");
            self.segments[s].get_or_init(|| Segment {
                generations: (0..SEGMENT).map(|_| AtomicU32::new(0)).collect(),
                locations: (0..SEGMENT).map(|_| AtomicU64::new(DEAD)).collect(),
            });
            index
        });
        let (generation, location) = self.slot(index).expect("a reserved slot exists");
        location.store(RESERVED, Ordering::Release);
        self.alive.fetch_add(1, Ordering::Relaxed);
        Entity { index, generation: generation.load(Ordering::Acquire) }
    }

    pub fn is_alive(&self, e: Entity) -> bool {
        self.slot(e.index).is_some_and(|(g, l)| {
            g.load(Ordering::Acquire) == e.generation && l.load(Ordering::Acquire) != DEAD
        })
    }

    pub fn location(&self, e: Entity) -> Option<Location> {
        let (g, l) = self.slot(e.index)?;
        if g.load(Ordering::Acquire) != e.generation {
            return None;
        }
        Location::unpack(l.load(Ordering::Acquire))
    }

    pub fn count(&self) -> usize {
        self.alive.load(Ordering::Relaxed)
    }

    pub(crate) fn place(&self, e: Entity, at: Location) {
        self.slot(e.index).expect("a placed entity's slot").1.store(at.pack(), Ordering::Release);
    }

    /// Kills `e`; its slot is reusable once the caller hands it to `free`.
    fn kill(&self, e: Entity) {
        let (g, l) = self.slot(e.index).expect("a killed entity's slot");
        l.store(DEAD, Ordering::Release);
        g.fetch_add(1, Ordering::AcqRel);
        self.alive.fetch_sub(1, Ordering::Relaxed);
    }

    fn free(&self, slots: &mut Vec<u32>) {
        if !slots.is_empty() {
            self.free.lock().unwrap().append(slots);
        }
    }
}

/// A component as stored: its name, storage, and (once installed) the
/// layout and code of the newest build that registered it.
pub struct ComponentInfo {
    pub name: String,
    pub storage: Storage,
    /// Whether this component keeps its tables in spatial order: fixed at
    /// its first declaration, like its storage.
    pub spatial: bool,
    /// Whether this component keeps its tables sorted by it: fixed like
    /// `spatial`.
    pub ordered: bool,
    /// The sparse set, for a sparse component, made when first installed.
    /// Before `installed`, so its values drop while the keepalive there
    /// still maps their code.
    sparse: OnceLock<RwLock<SparseSet>>,
    installed: RwLock<Option<Installed>>,
}

struct Installed {
    ty: ValueType,
    version: u32,
    fields: Vec<Field>,
    field_drops: Vec<Option<DropFn>>,
    default: DefaultFn,
    /// A spatial key's glue, from the same build as the layout.
    spatial: Option<SpatialDesc>,
    /// An ordered key's glue, likewise.
    order: Option<OrderDesc>,
    loaded_at: u64,
    _keepalive: Option<Keepalive>,
}

/// Every entity with exactly one set of table components. Rows live in
/// pages; every column has the same pages, and the same rows in each.
pub struct Table {
    pub id: TableId,
    /// Sorted.
    pub components: Vec<ComponentId>,
    pub rows: RwLock<Vec<Vec<Entity>>>,
    /// Parallel to `components`.
    pub columns: Vec<RwLock<Vec<ErasedColumn>>>,
    /// Rows a page holds: fewer in a spatial table, whose pages are
    /// neighborhoods.
    pub page_rows: usize,
    /// For a table holding a spatial key: the key, and the order. Locked
    /// with `rows`: whoever reads rows by page reads their order too.
    pub spatial: Option<SpatialTable>,
    /// For a table holding an ordered key (and no spatial one, which wins):
    /// the key, and each row's. Locked with `rows`, like `spatial`.
    pub ordered: Option<OrderedTable>,
    /// The ticks a row last arrived in this table (spawned, or moved from
    /// another) and last left it (despawned, or moved to another): what
    /// `Query::arrived_since` and `left_since` read. A row that's gone
    /// leaves no value to look at, and a count of rows can't tell one gone
    /// from one gone and another come; which rows arrived is in their
    /// values' ticks, and this says in a look whether to walk for them.
    /// Written under the rows' write guard and read under their read
    /// guard, so the lock orders them.
    pub arrived: AtomicU32,
    pub left: AtomicU32,
}

pub struct OrderedTable {
    pub key: ComponentId,
    pub order: RwLock<KeyOrder>,
}

pub struct SpatialTable {
    pub key: ComponentId,
    pub pages: RwLock<SpatialPages>,
}

impl Table {
    pub fn column_index(&self, c: ComponentId) -> Option<usize> {
        self.components.binary_search(&c).ok()
    }

    pub fn len(&self) -> usize {
        self.rows.read().unwrap_or_else(PoisonError::into_inner).iter().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// An unused slot in a sparse set's index.
const EMPTY: u32 = u32::MAX;

/// A sparse component's values, packed, with an index by entity. Entries
/// for entities that have died are left in place, invisible (their
/// generation no longer matches), and purged when the set is next written:
/// so despawning never has to touch every sparse set.
pub struct SparseSet {
    values: ErasedColumn,
    entities: Vec<Entity>,
    /// By entity index: the slot in `entities`/`values`, or `EMPTY`.
    slots: Vec<u32>,
}

impl SparseSet {
    fn new(ty: ValueType) -> SparseSet {
        SparseSet { values: ErasedColumn::new(ty), entities: Vec::new(), slots: Vec::new() }
    }

    pub fn len(&self) -> usize {
        self.entities.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entities.is_empty()
    }

    pub fn contains(&self, e: Entity) -> bool {
        self.slot(e).is_some()
    }

    /// Every entry's entity, including dead ones'; filter with
    /// `Entities::is_alive`.
    pub fn entities(&self) -> &[Entity] {
        &self.entities
    }

    fn slot(&self, e: Entity) -> Option<usize> {
        let slot = *self.slots.get(e.index as usize)?;
        (slot != EMPTY && self.entities[slot as usize] == e).then_some(slot as usize)
    }

    pub fn get<T: crate::Component>(&self, e: Entity) -> Option<&T> {
        Some(&self.values.as_slice::<T>()[self.slot(e)?])
    }

    pub fn get_mut<T: crate::Component>(&mut self, e: Entity) -> Option<&mut T> {
        let slot = self.slot(e)?;
        Some(&mut self.values.as_mut_slice::<T>()[slot])
    }

    /// `e`'s value and its tick, for a write at tick `now` that records
    /// itself.
    pub fn get_mut_ticked<T: crate::Component>(&mut self, e: Entity, now: u32) -> Option<(&mut T, &mut u32)> {
        let slot = self.slot(e)?;
        let (values, ticks) = self.values.as_mut_slice_ticked::<T>(now);
        Some((&mut values[slot], &mut ticks[slot]))
    }

    fn insert<T: crate::Component>(&mut self, e: Entity, value: T) {
        match self.slots.get(e.index as usize).copied().filter(|&s| s != EMPTY).map(|s| s as usize) {
            // The same entity replaces its value; a dead one's stale entry is
            // reused by the index's new owner.
            Some(slot) => {
                self.values.replace(slot, value);
                self.entities[slot] = e;
            }
            None => {
                let i = e.index as usize;
                if self.slots.len() <= i {
                    self.slots.resize(i + 1, EMPTY);
                }
                self.slots[i] = self.entities.len() as u32;
                self.entities.push(e);
                self.values.push(value);
            }
        }
    }

    fn remove_slot(&mut self, slot: usize) {
        self.values.swap_remove_drop(slot);
        let removed = self.entities.swap_remove(slot);
        self.slots[removed.index as usize] = EMPTY;
        if let Some(&moved) = self.entities.get(slot) {
            self.slots[moved.index as usize] = slot as u32;
        }
    }

    fn remove(&mut self, e: Entity) {
        if let Some(slot) = self.slot(e) {
            self.remove_slot(slot);
        }
    }

    fn purge_dead(&mut self, entities: &Entities) {
        let mut slot = 0;
        while slot < self.entities.len() {
            if entities.is_alive(self.entities[slot]) {
                slot += 1;
            } else {
                self.remove_slot(slot);
            }
        }
    }
}

pub struct World {
    // Fields drop in order: the values in tables and events first, then the
    // components, whose keepalives map the code that drops those values.
    tables: Arena<Table>,
    by_set: Mutex<HashMap<Vec<ComponentId>, TableId>>,
    pub(crate) events: Arena<RwLock<EventQueue>>,
    pub(crate) events_by_name: Mutex<HashMap<String, usize>>,
    components: Arena<ComponentInfo>,
    by_name: Mutex<HashMap<String, ComponentId>>,
    /// The names of spatial keys' extents: writing one moves rows too.
    extents: Mutex<Vec<String>>,
    pub entities: Entities,
    frame: AtomicU64,
    frame_open: AtomicBool,
    /// Stamped on values when written: each query, and each write between
    /// frames, takes the next.
    change_tick: AtomicU32,
    /// The threads systems (`Workers`) and apply nodes split work over:
    /// the host's, set between frames, so no mod owns a thread.
    executor: RwLock<Option<Arc<dyn Executor>>>,
}

impl Default for World {
    fn default() -> World {
        World::new()
    }
}

impl World {
    pub fn new() -> World {
        let world = World {
            components: Arena::new(MAX_COMPONENTS),
            by_name: Mutex::new(HashMap::new()),
            extents: Mutex::new(Vec::new()),
            tables: Arena::new(MAX_TABLES),
            by_set: Mutex::new(HashMap::new()),
            events: Arena::new(MAX_EVENTS),
            events_by_name: Mutex::new(HashMap::new()),
            entities: Entities::new(),
            frame: AtomicU64::new(0),
            frame_open: AtomicBool::new(false),
            change_tick: AtomicU32::new(1),
            executor: RwLock::new(None),
        };
        world.table_for(&[]);
        world
    }

    // ---- Components ----

    /// `desc`'s component's id, made if the name is new. Only the name and
    /// storage are fixed here; the layout comes with `install`.
    pub fn intern(&self, desc: &ComponentDesc) -> Result<ComponentId, String> {
        let mut by_name = self.by_name.lock().unwrap();
        if let Some(&id) = by_name.get(desc.name) {
            let info = self.component(id);
            if info.storage != desc.storage {
                return Err(format!(
                    "{} is stored as {:?}, and this build stores it as {:?}; restart the engine to change a component's storage",
                    desc.name, info.storage, desc.storage
                ));
            }
            if info.ordered != desc.order.is_some() {
                return Err(format!(
                    "{} is {}an ordered key, and this build says otherwise; restart the engine to change it",
                    desc.name,
                    if info.ordered { "" } else { "not " }
                ));
            }
            if info.spatial != desc.spatial.is_some() {
                return Err(format!(
                    "{} is {}kept in spatial order, and this build says otherwise; restart the engine to change it",
                    desc.name,
                    if info.spatial { "" } else { "not " }
                ));
            }
            return Ok(id);
        }
        if desc.spatial.is_some() && desc.storage == Storage::Sparse {
            // Tables are what's kept in order, and a sparse component isn't
            // in one.
            return Err(format!("{} can't be both sparse and a spatial key", desc.name));
        }
        if desc.order.is_some() && desc.storage == Storage::Sparse {
            return Err(format!("{} can't be both sparse and an ordered key", desc.name));
        }
        let info = ComponentInfo {
            name: desc.name.into(),
            storage: desc.storage,
            spatial: desc.spatial.is_some(),
            ordered: desc.order.is_some(),
            installed: RwLock::new(None),
            sparse: OnceLock::new(),
        };
        if let Some(s) = desc.spatial {
            self.extents.lock().unwrap().push(s.extent.to_string());
        }
        let id = ComponentId(self.components.push(info) as u32);
        by_name.insert(desc.name.into(), id);
        Ok(id)
    }

    /// Gives `desc`'s component `build`'s layout and code. A newer build than
    /// the one installed takes over: with the same layout, its code replaces
    /// the older build's; with another, stored values migrate field by field,
    /// or, if the version changed or there's no schema, reset to its default.
    /// An older build with another layout is refused. Returns what happened
    /// to stored values, if anything did. Between frames only.
    pub fn install(&self, desc: &ComponentDesc, build: &Build) -> Result<Option<String>, String> {
        assert!(!self.frame_open.load(Ordering::Acquire), "components are installed between frames");
        let id = self.intern(desc)?;
        let info = self.component(id);
        let (fields, field_drops) = unsafe { schema::read_fields(desc.fields, desc.field_count, desc.size, desc.align) }
            .ok_or_else(|| format!("{}: its schema doesn't describe its layout", desc.name))?;
        let ty = ValueType::from_desc(desc);
        let new = Installed {
            ty,
            version: desc.version,
            fields,
            field_drops,
            default: desc.default,
            spatial: desc.spatial,
            order: desc.order,
            loaded_at: build.loaded_at,
            _keepalive: build.keepalive.clone(),
        };
        let mut installed = info.installed.write().unwrap();
        let Some(current) = installed.as_ref() else {
            if info.storage == Storage::Sparse {
                let _ = info.sparse.set(RwLock::new(SparseSet::new(ty)));
            }
            *installed = Some(new);
            return Ok(None);
        };
        if build.loaded_at <= current.loaded_at {
            return if current.ty.same_values(&ty) {
                Ok(None)
            } else {
                Err(format!(
                    "{} was built with an older layout of {}; reload it with the rest of the game",
                    build.name, desc.name
                ))
            };
        }
        let report = if current.ty.same_values(&ty) {
            // The newer build's code takes over, so the older one can go.
            self.each_column(id, |c| c.adopt(ty));
            None
        } else {
            let migrate = current.version == new.version && !current.fields.is_empty() && !new.fields.is_empty();
            let mut count = 0;
            self.each_column(id, |c| {
                count += c.len();
                // SAFETY: `schema::migrate` moves or drops every field of
                // the old value with the old build's code (still mapped: its
                // keepalive is in `current`), and fills the new one from the
                // new build's default.
                unsafe {
                    if migrate {
                        c.migrate(ty, |old, to| {
                            schema::migrate(
                                (old, &current.fields, &current.field_drops),
                                (to, &new.fields, &new.field_drops, new.default),
                            )
                        })
                    } else {
                        let drop = current.ty;
                        c.migrate(ty, |old, to| {
                            drop_value(drop, old);
                            (new.default)(to);
                        })
                    }
                }
            });
            Some(if migrate {
                format!(
                    "migrated {count} value(s) of {} to {}'s layout: {}",
                    desc.name,
                    build.name,
                    schema::report(&current.fields, &new.fields)
                )
            } else {
                let why = if current.version != new.version { "its version changed" } else { "it has no schema" };
                format!("{} changed layout in {} and {why}; reset {count} value(s) to its default", desc.name, build.name)
            })
        };
        *installed = Some(new);
        Ok(report)
    }

    /// Runs `f` on every column holding component `c`: each page of each
    /// table with it, and its sparse set.
    fn each_column(&self, c: ComponentId, mut f: impl FnMut(&mut ErasedColumn)) {
        for t in self.tables() {
            if let Some(i) = t.column_index(c) {
                let mut pages = t.columns[i].take_write();
                for page in pages.iter_mut() {
                    f(page);
                }
            }
        }
        if let Some(set) = self.component(c).sparse.get() {
            f(&mut set.take_write().values);
        }
    }

    pub fn component(&self, id: ComponentId) -> &ComponentInfo {
        self.components.get(id.0 as usize)
    }

    pub fn components(&self) -> impl Iterator<Item = (ComponentId, &ComponentInfo)> {
        self.components.iter().enumerate().map(|(i, c)| (ComponentId(i as u32), c))
    }

    pub fn id(&self, name: &str) -> Option<ComponentId> {
        self.by_name.lock().unwrap().get(name).copied()
    }

    pub fn storage(&self, c: ComponentId) -> Storage {
        self.component(c).storage
    }

    /// Whether writing `c` can move rows of spatial tables: it's a spatial
    /// key, or a key's extent. A system that writes it gets an apply node,
    /// which re-sorts what it wrote.
    pub fn moves_rows(&self, c: ComponentId) -> bool {
        let info = self.component(c);
        info.spatial || info.ordered || self.extents.lock().unwrap().iter().any(|n| *n == info.name)
    }

    /// The glue of ordered key `c`, and when the build that installed it
    /// was loaded.
    fn order_desc(&self, c: ComponentId) -> Option<(OrderDesc, u64)> {
        self.component(c).installed.read().unwrap().as_ref().and_then(|i| Some((i.order?, i.loaded_at)))
    }

    /// The glue of spatial key `c`, from the build that installed it.
    fn spatial_desc(&self, c: ComponentId) -> Option<SpatialDesc> {
        self.component(c).installed.read().unwrap().as_ref().and_then(|i| i.spatial)
    }

    pub fn name(&self, c: ComponentId) -> &str {
        &self.component(c).name
    }

    /// The installed layout's type, for new columns.
    fn value_type(&self, c: ComponentId) -> ValueType {
        let installed = self.component(c).installed.read().unwrap();
        installed.as_ref().unwrap_or_else(|| panic!("{} used before it's installed", self.name(c))).ty
    }

    /// Whether `c` is installed with exactly this schema.
    pub fn installed_as(&self, c: ComponentId, fingerprint: u64) -> bool {
        let installed = self.component(c).installed.read().unwrap();
        installed.as_ref().is_some_and(|i| i.ty.fingerprint() == fingerprint)
    }

    pub fn sparse_set(&self, c: ComponentId) -> &RwLock<SparseSet> {
        self.component(c).sparse.get().unwrap_or_else(|| panic!("{} isn't an installed sparse component", self.name(c)))
    }

    // ---- Tables ----

    pub fn table(&self, id: TableId) -> &Table {
        self.tables.get(id.0 as usize)
    }

    /// Every table made so far. Tables only ever get added.
    pub fn tables(&self) -> impl Iterator<Item = &Table> {
        self.tables.iter()
    }

    /// The table for exactly these table components, made if it's new.
    pub fn table_for(&self, components: &[ComponentId]) -> TableId {
        let mut set: Vec<ComponentId> = components.to_vec();
        set.sort();
        set.dedup();
        let mut by_set = self.by_set.lock().unwrap();
        if let Some(&id) = by_set.get(&set) {
            return id;
        }
        let id = TableId(self.tables.count.load(Ordering::Acquire) as u32);
        let columns = set.iter().map(|&c| RwLock::new(vec![ErasedColumn::new(self.value_type(c))])).collect();
        // A table with a spatial key is kept in its order (the first key's,
        // should a table have two).
        let spatial = set.iter().copied().find(|&c| self.component(c).spatial).map(|key| SpatialTable {
            key,
            pages: RwLock::new(SpatialPages::default()),
        });
        let page_rows = if spatial.is_some() { SPATIAL_PAGE_ROWS } else { PAGE_ROWS };
        let ordered = spatial.is_none().then(|| set.iter().copied().find(|&c| self.component(c).ordered)).flatten().map(|key| {
            OrderedTable { key, order: RwLock::new(KeyOrder::default()) }
        });
        let table =
            Table { id, components: set.clone(), rows: RwLock::new(vec![Vec::new()]), columns, page_rows, spatial, ordered, arrived: AtomicU32::new(0), left: AtomicU32::new(0) };
        assert_eq!(self.tables.push(table), id.0 as usize);
        by_set.insert(set, id);
        id
    }

    // ---- Frames ----

    /// Opens a frame: from now until `end_frame`, the world is reached only
    /// through systems' parameters and apply nodes.
    pub fn begin_frame(&self) {
        self.frame.fetch_add(1, Ordering::AcqRel);
        self.frame_open.store(true, Ordering::Release);
    }

    /// Ends a frame: drops the events that became visible in the one before,
    /// which every reader has now had a full frame to see.
    pub fn end_frame(&self) {
        let frame = self.frame();
        for q in self.events.iter() {
            q.take_write().expire(frame);
        }
        self.frame_open.store(false, Ordering::Release);
    }

    pub fn frame(&self) -> u64 {
        self.frame.load(Ordering::Acquire)
    }

    /// A tick later than every value written so far, for stamping writes.
    pub fn next_tick(&self) -> u32 {
        self.change_tick.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// The latest tick handed out: every write so far is at or before it.
    pub fn current_tick(&self) -> u32 {
        self.change_tick.load(Ordering::Acquire)
    }

    /// Installs the threads to split work over, or none: between frames,
    /// by whoever owns them.
    pub fn set_executor(&self, executor: Option<Arc<dyn Executor>>) {
        assert!(!self.frame_open(), "the executor is changed between frames");
        *self.executor.write().unwrap_or_else(PoisonError::into_inner) = executor;
    }

    pub fn executor(&self) -> Option<Arc<dyn Executor>> {
        self.executor.read().unwrap_or_else(PoisonError::into_inner).clone()
    }

    pub fn frame_open(&self) -> bool {
        self.frame_open.load(Ordering::Acquire)
    }

    // ---- Inspection ----

    /// A clone of every value of `T`, if it's installed with `T`'s schema:
    /// for code in the loader's own process (tests, tooling).
    pub fn values<T: crate::Component + Clone>(&self) -> Option<Vec<(Entity, T)>> {
        let c = self.id(T::NAME)?;
        if !self.installed_as(c, T::FINGERPRINT) {
            return None;
        }
        let mut out = Vec::new();
        match self.storage(c) {
            Storage::Table => {
                for t in self.tables() {
                    let Some(i) = t.column_index(c) else { continue };
                    let rows = t.rows.read().unwrap_or_else(PoisonError::into_inner);
                    let pages = t.columns[i].read().unwrap_or_else(PoisonError::into_inner);
                    for (p, entities) in rows.iter().enumerate() {
                        out.extend(entities.iter().copied().zip(pages[p].as_slice::<T>().iter().cloned()));
                    }
                }
            }
            Storage::Sparse => {
                let set = self.sparse_set(c).read().unwrap_or_else(PoisonError::into_inner);
                for &e in set.entities() {
                    if self.entities.is_alive(e) {
                        out.push((e, set.get::<T>(e).expect("listed").clone()));
                    }
                }
            }
        }
        Some(out)
    }

    /// One line for `modctl list`: entity count, then each component's count.
    pub fn summary(&self) -> String {
        let counts: Vec<String> = self
            .components()
            .map(|(c, info)| {
                let n = match info.storage {
                    Storage::Table => self.tables().filter_map(|t| t.column_index(c).map(|_| t.len())).sum(),
                    Storage::Sparse => info.sparse.get().map_or(0, |s| {
                        let set = s.read().unwrap_or_else(PoisonError::into_inner);
                        set.entities().iter().filter(|e| self.entities.is_alive(**e)).count()
                    }),
                };
                format!("{} x{n}", info.name)
            })
            .collect();
        format!("world: {} entities; {}", self.entities.count(), counts.join(", "))
    }
}

fn contended<T>() -> T {
    panic!("a guard was taken: the scheduler started two overlapping nodes, or the world was used in a frame")
}

/// Takes a guard that nothing else may hold: one held means the scheduler
/// ran overlapping nodes, a bug to fail loudly on rather than wait out. A
/// poisoned lock is taken anyway: it means a system panicked holding it,
/// which fails that system's mod, and its writes stand like any system's.
pub(crate) trait TakeGuard<T> {
    fn take_read(&self) -> RwLockReadGuard<'_, T>;
    fn take_write(&self) -> RwLockWriteGuard<'_, T>;
}

impl<T> TakeGuard<T> for RwLock<T> {
    fn take_read(&self) -> RwLockReadGuard<'_, T> {
        match self.try_read() {
            Ok(guard) => guard,
            Err(TryLockError::Poisoned(poisoned)) => poisoned.into_inner(),
            Err(TryLockError::WouldBlock) => contended(),
        }
    }

    fn take_write(&self) -> RwLockWriteGuard<'_, T> {
        match self.try_write() {
            Ok(guard) => guard,
            Err(TryLockError::Poisoned(poisoned)) => poisoned.into_inner(),
            Err(TryLockError::WouldBlock) => contended(),
        }
    }
}

/// Write access to some tables and sparse sets, for applying structural
/// changes: an apply node takes one for exactly its footprint.
pub struct Structural<'w> {
    pub world: &'w World,
    /// By table id: a spawn looks its table up several times, and a hash
    /// of the id per lookup was a fifth of a spawn's cost.
    tables: Vec<Option<LockedTable<'w>>>,
    sparse: HashMap<ComponentId, RwLockWriteGuard<'w, SparseSet>>,
    events: HashMap<usize, RwLockWriteGuard<'w, EventQueue>>,
    /// The table the last shared component list spawned into: a system's
    /// spawns share one list, so a log of them resolves it once.
    pub(crate) spawned_into: Option<(Arc<[ComponentId]>, TableId)>,
    /// Entity slots freed by despawns, returned to the free list in one
    /// lock when this drops.
    freed: Vec<u32>,
    /// The tick this stamps what it spawns, inserts and takes out: one for
    /// all its changes, taken at the first, since no system runs while it's
    /// held, and a tick per row was an atomic add per spawn.
    tick: Option<u32>,
}

/// The new row a spawn or move is writing: each column's last page.
pub struct NewRow<'a, 'w> {
    columns: &'a mut [RwLockWriteGuard<'w, Vec<ErasedColumn>>],
}

impl NewRow<'_, '_> {
    pub fn column(&mut self, i: usize) -> &mut ErasedColumn {
        self.columns[i].last_mut().expect("a table has a page")
    }
}

struct LockedTable<'w> {
    table: &'w Table,
    rows: RwLockWriteGuard<'w, Vec<Vec<Entity>>>,
    columns: Vec<RwLockWriteGuard<'w, Vec<ErasedColumn>>>,
    spatial: Option<RwLockWriteGuard<'w, SpatialPages>>,
    ordered: Option<RwLockWriteGuard<'w, KeyOrder>>,
}

impl LockedTable<'_> {
    /// Marks a spatial or ordered table for re-sorting when the
    /// `Structural` drops.
    fn touch(&mut self) {
        if let Some(pages) = &mut self.spatial {
            pages.dirty = true;
        }
        if let Some(order) = &mut self.ordered {
            order.dirty = true;
        }
    }
}

impl<'w> Structural<'w> {
    pub fn new(world: &'w World) -> Structural<'w> {
        Structural { world, tables: Vec::new(), sparse: HashMap::new(), events: HashMap::new(), spawned_into: None, freed: Vec::new(), tick: None }
    }

    /// Every table and sparse set, for use between frames.
    pub fn everything(world: &'w World) -> Structural<'w> {
        let mut s = Structural::new(world);
        let tables: Vec<TableId> = world.tables().map(|t| t.id).collect();
        for t in tables {
            s.lock_table(t);
        }
        let sparse: Vec<ComponentId> = world
            .components()
            .filter(|(_, info)| info.storage == Storage::Sparse && info.sparse.get().is_some())
            .map(|(c, _)| c)
            .collect();
        for c in sparse {
            s.lock_sparse(c);
        }
        s
    }

    pub fn lock_table(&mut self, id: TableId) {
        if self.tables.get(id.0 as usize).is_some_and(Option::is_some) {
            return;
        }
        let table = self.world.table(id);
        let rows = table.rows.take_write();
        let columns = table.columns.iter().map(|c| c.take_write()).collect();
        let spatial = table.spatial.as_ref().map(|s| s.pages.take_write());
        let ordered = table.ordered.as_ref().map(|o| o.order.take_write());
        let i = id.0 as usize;
        if self.tables.len() <= i {
            self.tables.resize_with(i + 1, || None);
        }
        self.tables[i] = Some(LockedTable { table, rows, columns, spatial, ordered });
    }

    pub fn lock_sparse(&mut self, c: ComponentId) {
        self.lock_sparse_with(c, true);
    }

    /// `purge: false` leaves dead entities' entries in place, as they are
    /// between writes: for tests of the paths that meet them.
    pub fn lock_sparse_with(&mut self, c: ComponentId, purge: bool) {
        if self.sparse.contains_key(&c) {
            return;
        }
        let mut set = self.world.sparse_set(c).take_write();
        if purge {
            set.purge_dead(&self.world.entities);
        }
        self.sparse.insert(c, set);
    }

    /// Event queue `q`, for publishing; taken now if it isn't held yet.
    pub fn events(&mut self, q: usize) -> &mut EventQueue {
        let world = self.world;
        self.events.entry(q).or_insert_with(|| world.event_queue(q).take_write())
    }

    /// Table `id`, taken now if it isn't held yet: a sequential frame
    /// doesn't compute footprints up front. A parallel one does, and a table
    /// outside its footprint shows up here as a contended guard.
    fn locked(&mut self, id: TableId) -> &mut LockedTable<'w> {
        self.lock_table(id);
        self.tables[id.0 as usize].as_mut().expect("just locked")
    }

    /// A new page at the end of table `id`, if its last is full.
    fn make_room(&mut self, id: TableId) {
        let world = self.world;
        let t = self.locked(id);
        if t.rows.last().is_none_or(|p| p.len() == t.table.page_rows) {
            t.rows.push(Vec::new());
            for (i, c) in t.columns.iter_mut().enumerate() {
                c.push(ErasedColumn::new(world.value_type(t.table.components[i])));
            }
            if let Some(pages) = &mut t.spatial {
                pages.add_page();
            }
            if let Some(order) = &mut t.ordered {
                order.add_page();
            }
        }
    }

    /// The tick this `Structural`'s changes are stamped with: later than
    /// any `Query::now` taken before it.
    fn tick(&mut self) -> u32 {
        let world = self.world;
        *self.tick.get_or_insert_with(|| world.next_tick())
    }

    /// Places reserved entity `e` in table `id`, with `values` writing its
    /// components' values into the new row. A spawned value is a written
    /// one, so change detection sees what arrives, as it does a write.
    pub(crate) fn push_row(&mut self, id: TableId, e: Entity, values: impl FnOnce(NewRow<'_, 'w>)) {
        let world = self.world;
        let tick = self.tick();
        self.make_room(id);
        let t = self.locked(id);
        let page = t.rows.len() - 1;
        t.rows[page].push(e);
        let row = t.rows[page].len() - 1;
        values(NewRow { columns: &mut t.columns });
        assert!(t.columns.iter().all(|c| c[page].len() == row + 1), "every column gets a value");
        for c in t.columns.iter_mut() {
            c[page].set_tick(row, tick);
        }
        t.table.arrived.store(tick, Ordering::Relaxed);
        if let Some(pages) = &mut t.spatial {
            pages.push_row(page, e);
        }
        if let Some(order) = &mut t.ordered {
            order.push_row(page);
        }
        world.entities.place(e, Location { table: id, page: page as u32, row: row as u32 });
    }

    /// Takes a row out of its table: each value is moved to `into` if that
    /// table has its column, and dropped otherwise.
    fn take_row(&mut self, at: Location, into: Option<TableId>) {
        let world = self.world;
        let tick = self.tick();
        let (page, row) = (at.page as usize, at.row as usize);
        match into.filter(|&d| d != at.table) {
            Some(d) => {
                self.lock_table(at.table);
                self.lock_table(d);
                // Both tables' guards at once, from the one map.
                let [from, to] = self.tables.get_disjoint_mut([at.table.0 as usize, d.0 as usize]).expect("two different tables");
                let (from, to) = (from.as_mut().expect("source in the footprint"), to.as_mut().expect("destination in the footprint"));
                let (from_table, to_table) = (from.table, world.table(d));
                for (i, c) in from_table.components.iter().enumerate() {
                    match to_table.column_index(*c) {
                        Some(j) => from.columns[i][page].swap_remove_into(row, to.columns[j].last_mut().unwrap()),
                        None => from.columns[i][page].swap_remove_drop(row),
                    }
                }
            }
            None => {
                for column in self.locked(at.table).columns.iter_mut() {
                    column[page].swap_remove_drop(row);
                }
            }
        }
        let t = self.locked(at.table);
        t.table.left.store(tick, Ordering::Relaxed);
        t.rows[page].swap_remove(row);
        if let Some(pages) = &mut t.spatial {
            pages.swap_remove(page, row);
        }
        if let Some(order) = &mut t.ordered {
            order.swap_remove(page, row);
        }
        if let Some(&moved) = t.rows[page].get(row) {
            world.entities.place(moved, Location { table: at.table, page: at.page, row: at.row });
        }
    }

    /// Moves `e` to table `to`, keeping the values both tables have and
    /// dropping the rest; `extra` writes the rest.
    fn move_entity(&mut self, e: Entity, at: Location, to: TableId, extra: impl FnOnce(NewRow<'_, 'w>, &Table)) {
        let world = self.world;
        let table = world.table(to);
        let tick = self.tick();
        self.make_room(to);
        self.take_row(at, Some(to));
        let t = self.locked(to);
        t.table.arrived.store(tick, Ordering::Relaxed);
        let page = t.rows.len() - 1;
        t.rows[page].push(e);
        let row = t.rows[page].len() - 1;
        extra(NewRow { columns: &mut t.columns }, table);
        assert!(t.columns.iter().all(|c| c[page].len() == row + 1), "every column gets a value");
        if let Some(pages) = &mut t.spatial {
            pages.push_row(page, e);
        }
        if let Some(order) = &mut t.ordered {
            order.push_row(page);
        }
        world.entities.place(e, Location { table: to, page: page as u32, row: row as u32 });
    }

    pub fn spawn_empty(&mut self, e: Entity) {
        self.lock_table(TableId(0));
        self.push_row(TableId(0), e, |_| {});
    }

    /// Inserts `value` as component `c` on `e`, replacing any value it had. A
    /// dead entity's is dropped.
    pub fn insert_id<T: crate::Component>(&mut self, e: Entity, c: ComponentId, value: T) {
        let world = self.world;
        if world.storage(c) == Storage::Sparse {
            if world.entities.is_alive(e) {
                self.lock_sparse(c);
                self.sparse.get_mut(&c).expect("locked").insert(e, value);
            }
            return;
        }
        let Some(at) = world.entities.location(e) else { return };
        let from = world.table(at.table);
        if let Some(i) = from.column_index(c) {
            let moves = world.moves_rows(c);
            let tick = world.next_tick();
            let t = self.locked(at.table);
            let column = &mut t.columns[i][at.page as usize];
            column.replace(at.row as usize, value);
            column.set_tick(at.row as usize, tick);
            if moves {
                t.touch();
            }
            return;
        }
        let mut set = from.components.clone();
        set.push(c);
        let to = world.table_for(&set);
        self.lock_table(to);
        // Written as it's inserted, as a spawn's values are: the values
        // moved with it keep their ticks.
        let tick = self.tick();
        self.move_entity(e, at, to, |mut row, table| {
            let column = row.column(table.column_index(c).unwrap());
            column.push(value);
            column.set_tick(column.len() - 1, tick);
        });
    }

    /// Removes component `c`, whatever its type: moving and dropping values
    /// needs no type.
    pub fn remove_id(&mut self, e: Entity, c: ComponentId) {
        let world = self.world;
        if world.storage(c) == Storage::Sparse {
            self.lock_sparse(c);
            self.sparse.get_mut(&c).expect("locked").remove(e);
            return;
        }
        let Some(at) = world.entities.location(e) else { return };
        let from = world.table(at.table);
        if from.column_index(c).is_none() {
            return;
        }
        let set: Vec<ComponentId> = from.components.iter().copied().filter(|&x| x != c).collect();
        let to = world.table_for(&set);
        self.lock_table(to);
        self.move_entity(e, at, to, |_, _| {});
    }

    /// Drops `e`'s row. Its sparse components' entries become invisible at
    /// once and are purged when each set is next written.
    pub fn despawn(&mut self, e: Entity) {
        let Some(at) = self.world.entities.location(e) else { return };
        self.take_row(at, None);
        self.world.entities.kill(e);
        self.freed.push(e.index);
    }

    /// The value of `c` on `e`, through the guards this holds.
    pub fn get<T: crate::Component>(&mut self, e: Entity, c: ComponentId) -> Option<&mut T> {
        let world = self.world;
        if world.storage(c) == Storage::Sparse {
            self.lock_sparse(c);
            return self.sparse.get_mut(&c)?.get_mut::<T>(e);
        }
        let at = world.entities.location(e)?;
        let t = self.tables.get_mut(at.table.0 as usize)?.as_mut()?;
        let i = t.table.column_index(c)?;
        // Handed out mutably: counted as written, and if it's a key or an
        // extent, its row may move.
        if world.moves_rows(c) {
            t.touch();
        }
        let column = &mut t.columns[i][at.page as usize];
        column.set_tick(at.row as usize, world.next_tick());
        Some(&mut column.as_mut_slice::<T>()[at.row as usize])
    }

    /// Re-sorts spatial table `t` when this drops: after a system that wrote
    /// its keys or extents in place.
    pub fn resort(&mut self, t: TableId) {
        self.locked(t).touch();
    }
}

impl Drop for Structural<'_> {
    /// Re-sorts every spatial table this changed, so whoever takes its
    /// guards next finds it in order.
    fn drop(&mut self) {
        let world = self.world;
        world.entities.free(&mut self.freed);
        for t in self.tables.iter_mut().flatten() {
            if let Some(order) = t.ordered.as_mut().filter(|o| o.dirty) {
                let key = t.table.ordered.as_ref().expect("an ordered table").key;
                let (desc, desc_loaded_at) = world.order_desc(key).expect("an installed ordered key");
                ordered::Resort {
                    table: t.table.id,
                    rows: &mut t.rows,
                    columns: t.columns.iter_mut().map(|c| &mut **c).collect(),
                    order,
                    key: t.table.column_index(key).expect("the key's own table"),
                    desc,
                    desc_loaded_at,
                    page_rows: t.table.page_rows,
                    entities: &world.entities,
                    now: world.current_tick(),
                }
                .run();
            }
            let Some(pages) = t.spatial.as_mut().filter(|p| p.dirty) else { continue };
            let spatial = t.table.spatial.as_ref().expect("a spatial table");
            let desc = world.spatial_desc(spatial.key).expect("an installed spatial key");
            let key = t.table.column_index(spatial.key).expect("the key's own table");
            // The extent is read only as the layout the glue was built for.
            let extent = world
                .id(desc.extent)
                .filter(|&x| world.installed_as(x, desc.extent_fingerprint))
                .and_then(|x| t.table.column_index(x));
            Resort {
                table: t.table.id,
                rows: &mut t.rows,
                columns: t.columns.iter_mut().map(|c| &mut **c).collect(),
                pages,
                key,
                extent,
                desc,
                entities: &world.entities,
                now: world.current_tick(),
                workers: crate::par::Workers::new(world.executor()),
            }
            .run();
        }
    }
}

/// Read access to a table's rows, and its order if it's spatial, for a
/// query.
pub struct TableRead<'w> {
    pub table: &'w Table,
    pub rows: RwLockReadGuard<'w, Vec<Vec<Entity>>>,
    pub spatial: Option<RwLockReadGuard<'w, SpatialPages>>,
    pub ordered: Option<RwLockReadGuard<'w, KeyOrder>>,
}

impl<'w> TableRead<'w> {
    pub fn new(table: &'w Table) -> TableRead<'w> {
        let spatial = table.spatial.as_ref().map(|s| s.pages.take_read());
        let ordered = table.ordered.as_ref().map(|o| o.order.take_read());
        TableRead { table, rows: table.rows.take_read(), spatial, ordered }
    }
}

pub enum ColumnGuard<'w> {
    Read(RwLockReadGuard<'w, Vec<ErasedColumn>>),
    Write(RwLockWriteGuard<'w, Vec<ErasedColumn>>),
}

impl<'w> ColumnGuard<'w> {
    pub fn take(column: &'w RwLock<Vec<ErasedColumn>>, write: bool) -> ColumnGuard<'w> {
        if write {
            ColumnGuard::Write(column.take_write())
        } else {
            ColumnGuard::Read(column.take_read())
        }
    }

    pub fn pages(&self) -> &[ErasedColumn] {
        match self {
            ColumnGuard::Read(g) => g,
            ColumnGuard::Write(g) => g,
        }
    }

    pub fn pages_mut(&mut self) -> &mut [ErasedColumn] {
        match self {
            ColumnGuard::Read(_) => panic!("column was declared read-only"),
            ColumnGuard::Write(g) => g,
        }
    }
}

pub enum SparseGuard<'w> {
    Read(RwLockReadGuard<'w, SparseSet>),
    Write(RwLockWriteGuard<'w, SparseSet>),
}

impl<'w> SparseGuard<'w> {
    pub fn take(set: &'w RwLock<SparseSet>, write: bool) -> SparseGuard<'w> {
        if write {
            SparseGuard::Write(set.take_write())
        } else {
            SparseGuard::Read(set.take_read())
        }
    }

    pub fn set(&self) -> &SparseSet {
        match self {
            SparseGuard::Read(g) => g,
            SparseGuard::Write(g) => g,
        }
    }

    pub fn set_mut(&mut self) -> &mut SparseSet {
        match self {
            SparseGuard::Read(_) => panic!("sparse set was declared read-only"),
            SparseGuard::Write(g) => g,
        }
    }
}
