//! The world between frames: what hooks (`load`, `close`) and message
//! handlers use. Nothing else runs then, so each call takes the guards it
//! needs and gives them back, and changes land at once. In a frame the world
//! is reached only through systems' parameters, so `World::between_frames`
//! refuses there: that is what keeps a service called from a system out of
//! the world.

use crate::component::{Component, ComponentDesc, Entity};
use crate::events::Event;
use crate::query::{Bundle, Data, Declare, FilterDecl, Log, Query, QueryDecl};
use crate::world::{Build, ComponentId, Structural, World};

pub struct WorldMut<'w> {
    world: &'w World,
    /// The build whose code this is: components it names are installed with
    /// it on first use.
    build: Build,
    /// Where reports go (a migration's): the host's log, since this code runs
    /// inside a mod, and a mod printing through its own std leaks that std's
    /// stdout buffer when it unloads
    /// (docs/lore/a-mods-own-std-leaks-its-stdout-buffer-when-unloaded.md).
    /// None for the loader's and tests' own use, which drops them. `'static`
    /// so dropping a `WorldMut` doesn't keep the world borrowed (a closure
    /// bounded by `'w` would, by drop check).
    log: Option<Box<dyn Fn(&str)>>,
}

impl World {
    /// The world for `build`'s hook or message handler, if no frame is open.
    pub fn between_frames(&self, build: Build) -> Result<WorldMut<'_>, String> {
        if self.frame_open() {
            return Err("the world is reached through a system's parameters during a frame".into());
        }
        Ok(WorldMut { world: self, build, log: None })
    }
}

impl<'w> WorldMut<'w> {
    pub fn world(&self) -> &'w World {
        self.world
    }

    /// Sends this world's reports to `log`.
    pub fn with_log(mut self, log: impl Fn(&str) + 'static) -> WorldMut<'w> {
        self.log = Some(Box::new(log));
        self
    }

    /// `T`'s id, installing this build's layout of it if need be.
    fn install(&self, desc: &ComponentDesc) -> ComponentId {
        match self.world.install(desc, &self.build) {
            Ok(Some(report)) => {
                if let Some(log) = &self.log {
                    log(&report);
                }
            }
            Ok(None) => {}
            Err(e) => panic!("{e}"),
        }
        self.world.id(desc.name).expect("installed")
    }

    pub fn id<T: Component>(&self) -> ComponentId {
        self.install(&ComponentDesc::of::<T>())
    }

    /// A new entity with `bundle`'s components.
    pub fn spawn<B: Bundle>(&mut self, bundle: B) -> Entity {
        let mut d = Declare::new(self.world);
        let ids = B::components(&mut d);
        for desc in &d.components {
            self.install(desc);
        }
        let e = self.world.entities.reserve();
        let mut s = Structural::new(self.world);
        s.spawn(e, bundle, &ids);
        e
    }

    fn at_entity(&self, e: Entity) -> Structural<'w> {
        let mut s = Structural::new(self.world);
        if let Some(at) = self.world.entities.location(e) {
            s.lock_table(at.table);
        }
        s
    }

    /// Inserts `value` on `e`, replacing any it had. A dead entity's is dropped.
    pub fn insert<T: Component>(&mut self, e: Entity, value: T) {
        let c = self.id::<T>();
        self.at_entity(e).insert_id(e, c, value);
    }

    pub fn remove<T: Component>(&mut self, e: Entity) {
        let c = self.id::<T>();
        self.at_entity(e).remove_id(e, c);
    }

    pub fn despawn(&mut self, e: Entity) {
        self.at_entity(e).despawn(e);
    }

    pub fn is_alive(&self, e: Entity) -> bool {
        self.world.entities.is_alive(e)
    }

    /// `e`'s `T`, copied out.
    pub fn get<T: Component + Clone>(&self, e: Entity) -> Option<T> {
        let c = self.id::<T>();
        self.at_entity(e).get::<T>(e, c).cloned()
    }

    /// Calls `f` with `e`'s `T`, which it may change.
    pub fn with_mut<T: Component, R>(&mut self, e: Entity, f: impl FnOnce(&mut T) -> R) -> Option<R> {
        let c = self.id::<T>();
        self.at_entity(e).get::<T>(e, c).map(f)
    }

    /// Every entity with all of `D`'s components, with their values, which a
    /// `&mut` term may change.
    pub fn for_each<D: Data>(&mut self, f: impl FnMut(Entity, D::Items<'_>)) {
        self.for_each_filtered::<D>(FilterDecl::default(), f);
    }

    fn for_each_filtered<D: Data>(&mut self, filter: FilterDecl, mut f: impl FnMut(Entity, D::Items<'_>)) {
        let mut d = Declare::new(self.world);
        let terms = D::terms(&mut d);
        for desc in &d.components {
            self.install(desc);
        }
        let reorders = terms.iter().any(|&(c, write)| write && self.world.moves_rows(c));
        let decl = QueryDecl { terms, filter, changes: Default::default(), reorders };
        let log = Log::default();
        let mut q: Query<'_, D> = Query::take(self.world, &decl, &log);
        q.log_reorders();
        q.for_each(|row, items| f(row.entity(), items));
        drop(q);
        // Nothing else runs between frames: the writes re-sort at once.
        let mut s = Structural::new(self.world);
        for change in log.into_inner() {
            change.apply(&mut s);
        }
    }

    /// The first entity with all of `D`'s components, and its values copied
    /// out by `f`: for components there's one of (a clock, a level).
    pub fn single<D: Data, R>(&mut self, f: impl FnOnce(Entity, D::Items<'_>) -> R) -> Option<R> {
        let mut f = Some(f);
        let mut out = None;
        self.for_each::<D>(|e, items| {
            if let Some(f) = f.take() {
                out = Some(f(e, items));
            }
        });
        out
    }

    /// Sends an event, readable from the next frame.
    pub fn send_event<E: Event>(&mut self, event: E) {
        let desc = ComponentDesc::of::<E>();
        if let Err(e) = self.world.install_event(&desc, &self.build) {
            panic!("{e}");
        }
        let q = self.world.intern_event(&desc);
        let mut s = Structural::new(self.world);
        // Tagged for the next frame, so it lives through that frame's end.
        let frame = self.world.frame() + 1;
        s.events(q).push(event, frame);
    }
}

/// The components a declaration names, for installing them: what the loader
/// does when a load commits.
pub fn install_all(world: &World, components: &[ComponentDesc], events: &[ComponentDesc], build: &Build) -> Result<Vec<String>, String> {
    let mut reports = Vec::new();
    for desc in components {
        if let Some(report) = world.install(desc, build)? {
            reports.push(report);
        }
    }
    for desc in events {
        world.install_event(desc, build)?;
    }
    Ok(reports)
}
