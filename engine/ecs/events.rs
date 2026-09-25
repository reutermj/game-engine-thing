//! Events: values one mod sends and others read, without either depending
//! on the other's state. A system sends through an `EventWriter<E>`, its
//! sends going into its log like structural changes, published by its apply
//! node; a system later in the plan reading `E` waits for that, so it sees
//! them the same frame, and one earlier sees them the next. Outside a frame,
//! `WorldMut::send_event` publishes at once, for the next frame.
//!
//! Each reader sees each event once: the queue keeps a cursor per reading
//! system, by name, so it survives the system's reload. An event lives until
//! the end of the frame after the one it was published in. See
//! docs/architecture/storage.md.

use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::{Mutex, RwLock, RwLockReadGuard};

use crate::component::{Component, ComponentDesc};
use crate::erased::{ErasedColumn, ValueType};
use crate::query::{Change, Declare, FrameCx, Log, Param, ParamDecl};
use crate::world::{Build, Keepalive, TakeGuard, World};

/// An event type: declared with [`event!`](crate::event), under the same
/// rules as a component.
///
/// # Safety
/// As for [`Component`].
pub unsafe trait Event: Component {}

/// Declares an event type, with the syntax of [`component!`](crate::component).
#[macro_export]
macro_rules! event {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident : $id:literal $($rest:tt)*
    ) => {
        $crate::component! { $(#[$meta])* $vis struct $name : $id $($rest)* }
        unsafe impl $crate::Event for $name {}
    };
}

/// One event type's published events, oldest first.
pub struct EventQueue {
    pub name: String,
    ty: Option<ValueType>,
    loaded_at: u64,
    values: Option<ErasedColumn>,
    /// Parallel to `values`: each event's sequence number, and the frame it
    /// was published for.
    seqs: Vec<u64>,
    frames: Vec<u64>,
    next_seq: u64,
    /// The next sequence number each reader will read, by system name.
    cursors: Mutex<BTreeMap<String, u64>>,
    /// After `values`, so they drop while it still maps their code.
    _keepalive: Option<Keepalive>,
}

impl EventQueue {
    fn new(name: &str) -> EventQueue {
        EventQueue {
            name: name.into(),
            ty: None,
            loaded_at: 0,
            _keepalive: None,
            values: None,
            seqs: Vec::new(),
            frames: Vec::new(),
            next_seq: 0,
            cursors: Mutex::new(BTreeMap::new()),
        }
    }

    pub(crate) fn push<E: Event>(&mut self, event: E, frame: u64) {
        self.values.as_mut().expect("an installed event type").push(event);
        self.seqs.push(self.next_seq);
        self.frames.push(frame);
        self.next_seq += 1;
    }

    /// Drops the events published before `frame`.
    pub(crate) fn expire(&mut self, frame: u64) {
        let expired = self.frames.partition_point(|&f| f < frame);
        if expired == 0 {
            return;
        }
        self.values.as_mut().expect("events were published, so installed").drop_front(expired);
        self.seqs.drain(..expired);
        self.frames.drain(..expired);
    }

    /// The events `system` hasn't read, marking them read.
    fn unread<E: Event>(&self, system: &str) -> &[E] {
        let Some(values) = &self.values else { return &[] };
        let mut cursors = self.cursors.lock().unwrap();
        let cursor = cursors.entry(system.into()).or_insert(0);
        let start = self.seqs.partition_point(|&s| s < *cursor);
        if let Some(&last) = self.seqs.last() {
            *cursor = last + 1;
        }
        &values.as_slice::<E>()[start..]
    }
}

impl World {
    /// The queue for `desc`'s event type, made if it's new.
    pub fn intern_event(&self, desc: &ComponentDesc) -> usize {
        let mut by_name = self.events_by_name.lock().unwrap();
        if let Some(&q) = by_name.get(desc.name) {
            return q;
        }
        let q = self.events.push(RwLock::new(EventQueue::new(desc.name)));
        by_name.insert(desc.name.into(), q);
        q
    }

    /// Gives an event type `build`'s layout and code. A newer build with
    /// another layout drops the queued events, which only last a frame or
    /// two anyway. Between frames only.
    pub fn install_event(&self, desc: &ComponentDesc, build: &Build) -> Result<(), String> {
        let q = self.intern_event(desc);
        let mut queue = self.events.get(q).take_write();
        let ty = ValueType::from_desc(desc);
        match queue.ty {
            Some(current) if build.loaded_at <= queue.loaded_at => {
                if current.same_values(&ty) {
                    Ok(())
                } else {
                    Err(format!("{} was built with an older layout of the event {}", build.name, desc.name))
                }
            }
            Some(current) if current.same_values(&ty) => {
                if let Some(values) = &mut queue.values {
                    values.adopt(ty);
                }
                (queue.ty, queue.loaded_at, queue._keepalive) = (Some(ty), build.loaded_at, build.keepalive.clone());
                Ok(())
            }
            _ => {
                queue.values = Some(ErasedColumn::new(ty));
                queue.seqs.clear();
                queue.frames.clear();
                (queue.ty, queue.loaded_at, queue._keepalive) = (Some(ty), build.loaded_at, build.keepalive.clone());
                Ok(())
            }
        }
    }

    /// A clone of every queued event of type `E`, oldest first, as (sequence
    /// number, frame published for, event), and each reader's cursor by
    /// system name: for code in the loader's own process (tests,
    /// tooling), like [`World::values`]. `None` if `E` isn't installed with
    /// this layout.
    pub fn events_of<E: Event + Clone>(&self) -> Option<(Vec<(u64, u64, E)>, Vec<(String, u64)>)> {
        let q = *self.events_by_name.lock().unwrap().get(E::NAME)?;
        let queue = self.events.get(q).read().unwrap_or_else(std::sync::PoisonError::into_inner);
        if !queue.ty?.same_values(&ValueType::of::<E>()) {
            return None;
        }
        let values = queue.values.as_ref().map_or(&[][..], |v| v.as_slice::<E>());
        let events = queue.seqs.iter().zip(&queue.frames).zip(values).map(|((&s, &f), e)| (s, f, e.clone())).collect();
        let cursors = queue.cursors.lock().unwrap().iter().map(|(k, &v)| (k.clone(), v)).collect();
        Some((events, cursors))
    }

    pub(crate) fn event_queue(&self, q: usize) -> &RwLock<EventQueue> {
        self.events.get(q)
    }
}

/// Reads the events of type `E` this system hasn't seen.
pub struct EventReader<'w, E> {
    queue: RwLockReadGuard<'w, EventQueue>,
    system: &'w str,
    _marker: PhantomData<fn() -> E>,
}

impl<E: Event> EventReader<'_, E> {
    /// The events published since this system last read them, oldest first;
    /// each is returned once.
    pub fn read(&mut self) -> &[E] {
        self.queue.unread::<E>(self.system)
    }
}

/// Sends events of type `E`, published after this system returns.
pub struct EventWriter<'w, E> {
    queue: usize,
    log: &'w Log,
    _marker: PhantomData<fn() -> E>,
}

impl<E: Event> EventWriter<'_, E> {
    pub fn send(&self, event: E) {
        let publish = Box::new(move |q: &mut EventQueue, frame| q.push(event, frame));
        self.log.borrow_mut().push(Change::Event { queue: self.queue, publish });
    }
}

impl<E: Event> Param for EventReader<'static, E> {
    type Item<'w> = EventReader<'w, E>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Events { queue: d.event::<E>(), write: false }
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> EventReader<'w, E> {
        let ParamDecl::Events { queue, .. } = decl else { panic!("an event reader's declaration") };
        let queue = cx.world.event_queue(*queue).take_read();
        EventReader { queue, system: cx.system, _marker: PhantomData }
    }
}

impl<E: Event> Param for EventWriter<'static, E> {
    type Item<'w> = EventWriter<'w, E>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Events { queue: d.event::<E>(), write: true }
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> EventWriter<'w, E> {
        let ParamDecl::Events { queue, .. } = decl else { panic!("an event writer's declaration") };
        EventWriter { queue: *queue, log: cx.log, _marker: PhantomData }
    }
}
