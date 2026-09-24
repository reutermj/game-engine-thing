//! Option 4: contacts in one component owned by physics, rebuilt each step
//! (warm-started by pair), and a `Contacts<Data, Filter>` parameter over
//! it, as `Spatial` is over positions: hooks see each contact from the side
//! their query matches, with that side's components.

use engine_ecs::harness::Cx;
use engine_ecs::{Changes, Data, Declare, Entity, Filter, FrameCx, Param, ParamDecl, Query, Row, Without, component, field_struct};
use physics::Vec2;

use crate::common::{Body, Found, Pos, Shape, Solvable, Vel, detect, gravity, solve_step};

field_struct! {
    #[derive(Debug, Default, Copy, PartialEq)]
    pub struct ContactRow {
        pub a: Entity,
        pub b: Entity,
        pub nx: f32,
        pub ny: f32,
        pub depth: f32,
        pub jn: f32,
        pub jt: f32,
        pub restitution: f32,
        pub friction: f32,
        /// Not solved this step: a pre-solve hook said so.
        pub disabled: bool,
        /// Wasn't a contact last step.
        pub began: bool,
    }
}

component! {
    /// Every contact this step, sorted by pair: one entity has it.
    #[derive(Debug, Default, PartialEq)]
    pub struct ContactTable: "rel::ContactTable" { pub rows: Vec<ContactRow> }
}

pub fn setup(w: &engine_ecs::World) {
    w.between_frames(Default::default()).unwrap().spawn((ContactTable::default(),));
}

pub fn gravity_system(_: &mut Cx, mut bodies: Query<(&Body, &mut Vel)>) {
    gravity(&mut bodies);
}

/// Detection, then the table rebuilt from it, warm-started from last
/// step's rows by pair.
pub fn find(
    _: &mut Cx,
    mut bodies: Query<(&Pos, &Shape, &Vel, &Body)>,
    mut statics: Query<(&Pos, &Shape), Without<Body>>,
    table: Query<&mut ContactTable>,
) {
    merge(&detect(&mut bodies, &mut statics), table);
}

/// The storage's half of `find`: the table rebuilt, warm-started by pair.
pub fn merge(found: &[Found], mut table: Query<&mut ContactTable>) {
    table.single(|_, mut t| {
        let old = std::mem::take(&mut t.rows);
        let mut i = 0;
        t.rows = found
            .iter()
            .map(|f| {
                while i < old.len() && (old[i].a, old[i].b) < (f.a, f.b) {
                    i += 1;
                }
                let prev = old.get(i).filter(|o| (o.a, o.b) == (f.a, f.b));
                ContactRow {
                    a: f.a,
                    b: f.b,
                    nx: f.normal.x,
                    ny: f.normal.y,
                    depth: f.depth,
                    jn: prev.map_or(0.0, |p| p.jn),
                    jt: prev.map_or(0.0, |p| p.jt),
                    restitution: 0.1,
                    friction: 0.4,
                    disabled: false,
                    began: prev.is_none(),
                }
            })
            .collect();
    });
}

pub fn solve(_: &mut Cx, mut bodies: Query<(&Body, &mut Vel, &mut Pos)>, mut table: Query<&mut ContactTable>) {
    table.single(|_, mut t| {
        let live: Vec<usize> = (0..t.rows.len()).filter(|&i| !t.rows[i].disabled).collect();
        let solvable: Vec<Solvable> = live
            .iter()
            .map(|&i| {
                let r = &t.rows[i];
                Solvable { a: r.a, b: r.b, normal: Vec2::new(r.nx, r.ny), depth: r.depth, restitution: r.restitution, friction: r.friction, jn: r.jn, jt: r.jt }
            })
            .collect();
        let impulses = solve_step(&mut bodies, &solvable);
        for (&i, (jn, jt)) in live.iter().zip(impulses) {
            (t.rows[i].jn, t.rows[i].jt) = (jn, jt);
        }
    });
}

/// A contact seen from one side: the normal points from that side toward
/// `other`.
pub struct Seen<'a> {
    row: &'a mut ContactRow,
    flipped: bool,
}

impl Seen<'_> {
    pub fn other(&self) -> Entity {
        if self.flipped { self.row.a } else { self.row.b }
    }
    /// Toward the other side.
    pub fn normal(&self) -> Vec2 {
        let n = Vec2::new(self.row.nx, self.row.ny);
        if self.flipped { -n } else { n }
    }
    pub fn began(&self) -> bool {
        self.row.began
    }
    pub fn disable(&mut self) {
        self.row.disabled = true;
    }
    pub fn set_restitution(&mut self, r: f32) {
        self.row.restitution = r;
    }
}

/// Contacts involving entities `Data` and `Filter` match, each seen from
/// the matching side: what a pre-solve hook takes.
pub struct Contacts<'w, D: Data, F = (), C = ()> {
    table: Query<'w, &'static mut ContactTable>,
    query: Query<'w, D, F, C>,
}

impl<D: Data + 'static, F: Filter, C: Changes> Param for Contacts<'static, D, F, C> {
    type Item<'w> = Contacts<'w, D, F, C>;
    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        <(Query<'static, &'static mut ContactTable>, Query<'static, D, F, C>)>::declare(d)
    }
    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Contacts<'w, D, F, C> {
        let (table, query) = <(Query<'static, &'static mut ContactTable>, Query<'static, D, F, C>)>::fetch(cx, decl);
        Contacts { table, query }
    }
}

impl<D: Data, F, C> Contacts<'_, D, F, C> {
    /// Each contact whose side matches the query, with that side's row and
    /// items. A contact whose two sides both match is seen twice.
    pub fn for_each(&mut self, mut f: impl FnMut(&mut Seen<'_>, Row<'_>, D::Items<'_>)) {
        let Contacts { table, query } = self;
        table.single(|_, mut t| {
            for row in t.rows.iter_mut() {
                for flipped in [false, true] {
                    let me = if flipped { row.b } else { row.a };
                    query.with(me, |r, items| f(&mut Seen { row: &mut *row, flipped }, r, items));
                }
            }
        });
    }
}
