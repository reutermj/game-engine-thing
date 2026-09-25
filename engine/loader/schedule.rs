//! Turns the loaded builds' declarations into the order a frame runs their
//! systems in. Pure: the engine calls it before a load commits, to refuse one
//! that would make the order impossible, and again whenever the builds
//! change. See docs/architecture/scheduling.md, "Phases and order".

use std::collections::HashMap;

use engine_api::{PhaseDesc, SystemDesc, phase};

/// One mod's declarations, in load order.
pub struct ModDecls<'a> {
    pub name: &'a str,
    pub systems: &'a [SystemDesc],
    pub phases: &'a [PhaseDesc],
}

#[derive(Debug, PartialEq)]
pub struct Plan {
    pub phases: Vec<PlannedPhase>,
    /// By id: the phase and position of each system.
    index: Vec<(usize, usize)>,
}

#[derive(Debug, PartialEq)]
pub struct PlannedPhase {
    pub name: String,
    pub systems: Vec<Planned>,
    /// Steps per second, if fixed-rate: the frame runs it once per step.
    pub fixed_hz: Option<f32>,
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Planned {
    /// Its position in the frame, counting across phases: what a scheduler
    /// names it by.
    pub id: usize,
    pub module: String,
    /// Index into the mod's systems.
    pub index: usize,
    /// `mod::system`.
    pub name: String,
}

impl Plan {
    pub fn system(&self, id: usize) -> Option<&Planned> {
        let &(phase, at) = self.index.get(id)?;
        Some(&self.phases[phase].systems[at])
    }

    /// One line per phase with systems, for `modctl schedule`.
    pub fn describe(&self) -> String {
        let lines: Vec<String> = self
            .phases
            .iter()
            .filter(|p| !p.systems.is_empty())
            .map(|p| {
                let names: Vec<&str> = p.systems.iter().map(|s| s.name.as_str()).collect();
                let rate = p.fixed_hz.map_or(String::new(), |hz| format!(" ({hz} Hz)"));
                format!("{}{rate}: {}", p.name, names.join(", "))
            })
            .collect();
        if lines.is_empty() { "no systems".into() } else { lines.join("\n") }
    }
}

pub fn plan(mods: &[ModDecls]) -> Result<Plan, String> {
    let phases = order_phases(mods)?;
    let phase_rank: HashMap<&str, usize> = phases.iter().enumerate().map(|(i, p)| (p.as_str(), i)).collect();

    // Every system, in load order then declaration order: the tie-break.
    struct Node<'a> {
        planned: Planned,
        phase: usize,
        desc: &'a SystemDesc,
    }
    let mut nodes = Vec::new();
    for m in mods {
        for (index, desc) in m.systems.iter().enumerate() {
            let Some(&rank) = phase_rank.get(desc.phase.as_str()) else {
                return Err(format!("{}::{} runs in phase {}, which no loaded mod declares", m.name, desc.name, desc.phase));
            };
            let name = format!("{}::{}", m.name, desc.name);
            if nodes.iter().any(|n: &Node| n.planned.name == name) {
                return Err(format!("{} declares the system {name} twice", m.name));
            }
            nodes.push(Node { planned: Planned { id: 0, module: m.name.into(), index, name }, phase: rank, desc });
        }
    }
    let by_name: HashMap<&str, usize> = nodes.iter().enumerate().map(|(i, n)| (n.planned.name.as_str(), i)).collect();

    // Edges within a phase; constraints across phases must agree with the
    // phase order, and ones naming systems that aren't loaded are dropped.
    let mut edges: Vec<(usize, usize)> = Vec::new();
    for (i, n) in nodes.iter().enumerate() {
        let constraints = n.desc.after.iter().map(|a| (a, true)).chain(n.desc.before.iter().map(|b| (b, false)));
        for (other, after) in constraints {
            let Some(&j) = by_name.get(other.as_str()) else { continue };
            let (first, then) = if after { (j, i) } else { (i, j) };
            match nodes[first].phase.cmp(&nodes[then].phase) {
                std::cmp::Ordering::Equal => edges.push((first, then)),
                std::cmp::Ordering::Less => {}
                std::cmp::Ordering::Greater => {
                    return Err(format!(
                        "{} must run {} {other}, but its phase {} comes {} {}",
                        n.planned.name,
                        if after { "after" } else { "before" },
                        phases[n.phase],
                        if after { "before" } else { "after" },
                        phases[nodes[j].phase],
                    ));
                }
            }
        }
    }
    let order = topological(nodes.len(), &edges, |i| (nodes[i].phase, i)).map_err(|stuck| {
        let names: Vec<&str> = stuck.iter().map(|&i| nodes[i].planned.name.as_str()).collect();
        format!("the systems {} are ordered in a cycle", names.join(", "))
    })?;

    // `simulate` is fixed-rate, and any phase a mod declares so (the first
    // declaration with a rate wins).
    let rate = |name: &str| {
        mods.iter()
            .flat_map(|m| m.phases)
            .filter(|p| p.name == name)
            .find_map(|p| p.fixed_hz)
            .or_else(|| (name == phase::SIMULATE).then_some(phase::SIMULATE_HZ))
    };
    let mut planned: Vec<PlannedPhase> =
        phases.iter().map(|name| PlannedPhase { name: name.clone(), systems: Vec::new(), fixed_hz: rate(name) }).collect();
    for i in order {
        planned[nodes[i].phase].systems.push(nodes[i].planned.clone());
    }
    let mut index = Vec::new();
    for (p, phase) in planned.iter_mut().enumerate() {
        for (at, system) in phase.systems.iter_mut().enumerate() {
            system.id = index.len();
            index.push((p, at));
        }
    }
    Ok(Plan { phases: planned, index })
}

/// The engine's phases and every declared one, in order.
fn order_phases(mods: &[ModDecls]) -> Result<Vec<String>, String> {
    let mut names: Vec<String> = phase::BUILTIN.iter().map(|p| p.to_string()).collect();
    for p in mods.iter().flat_map(|m| m.phases) {
        if !names.contains(&p.name) {
            names.push(p.name.clone());
        }
    }
    let index = |name: &str| names.iter().position(|n| n == name);
    let mut edges: Vec<(usize, usize)> = (1..phase::BUILTIN.len()).map(|i| (i - 1, i)).collect();
    for p in mods.iter().flat_map(|m| m.phases) {
        let this = index(&p.name).unwrap();
        // A phase placed relative to one that isn't declared is placed by
        // its other constraints.
        edges.extend(p.after.iter().filter_map(|a| Some((index(a)?, this))));
        edges.extend(p.before.iter().filter_map(|b| Some((this, index(b)?))));
    }
    let order = topological(names.len(), &edges, |i| i).map_err(|stuck| {
        let stuck: Vec<&str> = stuck.iter().map(|&i| names[i].as_str()).collect();
        format!("the phases {} are ordered in a cycle", stuck.join(", "))
    })?;
    Ok(order.into_iter().map(|i| names[i].clone()).collect())
}

/// Kahn's algorithm, taking the ready node with the lowest `rank` first so
/// the order is deterministic. `Err` holds the nodes left in a cycle.
fn topological<K: Ord>(n: usize, edges: &[(usize, usize)], rank: impl Fn(usize) -> K) -> Result<Vec<usize>, Vec<usize>> {
    let mut incoming = vec![0usize; n];
    for &(_, to) in edges {
        incoming[to] += 1;
    }
    let mut done = vec![false; n];
    let mut order = Vec::with_capacity(n);
    while order.len() < n {
        let Some(next) = (0..n).filter(|&i| !done[i] && incoming[i] == 0).min_by_key(|&i| rank(i)) else {
            return Err((0..n).filter(|&i| !done[i]).collect());
        };
        done[next] = true;
        order.push(next);
        for &(from, to) in edges {
            if from == next {
                incoming[to] -= 1;
            }
        }
    }
    Ok(order)
}

#[cfg(test)]
mod tests {
    use engine_api::engine_ecs::{FrameCx, ParamDecl};
    use engine_api::{ModContext, Status};

    use super::*;

    unsafe fn nothing(_: *mut ModContext, _: &FrameCx<'_>, _: &[ParamDecl]) -> Status {
        Status::OK
    }

    fn system(name: &str, phase: &str, after: &[&str], before: &[&str]) -> SystemDesc {
        SystemDesc {
            name: name.into(),
            phase: phase.into(),
            after: after.iter().map(|s| s.to_string()).collect(),
            before: before.iter().map(|s| s.to_string()).collect(),
            params: Vec::new(),
            run: nothing,
        }
    }

    fn phase(name: &str, after: &[&str], before: &[&str]) -> PhaseDesc {
        PhaseDesc {
            name: name.into(),
            after: after.iter().map(|s| s.to_string()).collect(),
            before: before.iter().map(|s| s.to_string()).collect(),
            fixed_hz: None,
        }
    }

    fn order(mods: &[(&str, Vec<SystemDesc>, Vec<PhaseDesc>)]) -> Result<Vec<String>, String> {
        let decls: Vec<ModDecls> = mods.iter().map(|(name, systems, phases)| ModDecls { name, systems, phases }).collect();
        Ok(plan(&decls)?.phases.into_iter().flat_map(|p| p.systems).map(|s| s.name).collect())
    }

    #[test]
    fn phases_run_in_order_whatever_the_load_order() {
        let got = order(&[
            ("render", vec![system("draw", phase::RENDER, &[], &[])], vec![]),
            ("physics", vec![system("integrate", phase::SIMULATE, &[], &[])], vec![]),
            ("input", vec![system("read", phase::INPUT, &[], &[])], vec![]),
        ]);
        assert_eq!(got.unwrap(), ["input::read", "physics::integrate", "render::draw"]);
    }

    #[test]
    fn a_phase_ties_on_load_then_declaration_order() {
        let got = order(&[
            ("b", vec![system("two", phase::UPDATE, &[], &[]), system("one", phase::UPDATE, &[], &[])], vec![]),
            ("a", vec![system("x", phase::UPDATE, &[], &[])], vec![]),
        ]);
        assert_eq!(got.unwrap(), ["b::two", "b::one", "a::x"]);
    }

    #[test]
    fn constraints_order_systems_within_a_phase() {
        let got = order(&[
            ("pong", vec![system("move", phase::UPDATE, &["ai::think"], &[])], vec![]),
            ("ai", vec![system("think", phase::UPDATE, &[], &[])], vec![]),
            ("text", vec![system("draw", phase::UPDATE, &[], &["pong::move"])], vec![]),
        ]);
        assert_eq!(got.unwrap(), ["ai::think", "text::draw", "pong::move"]);
    }

    #[test]
    fn a_constraint_on_a_system_that_isnt_loaded_is_ignored() {
        let got = order(&[("a", vec![system("x", phase::UPDATE, &["gone::y"], &["gone::z"])], vec![])]);
        assert_eq!(got.unwrap(), ["a::x"]);
    }

    #[test]
    fn a_constraint_across_phases_must_agree_with_them() {
        let agrees = order(&[
            ("a", vec![system("x", phase::SIMULATE, &["b::y"], &[])], vec![]),
            ("b", vec![system("y", phase::INPUT, &[], &[])], vec![]),
        ]);
        assert_eq!(agrees.unwrap(), ["b::y", "a::x"]);
        let err = order(&[
            ("a", vec![system("x", phase::INPUT, &["b::y"], &[])], vec![]),
            ("b", vec![system("y", phase::SIMULATE, &[], &[])], vec![]),
        ])
        .unwrap_err();
        assert_eq!(err, "a::x must run after b::y, but its phase input comes before simulate");
    }

    #[test]
    fn a_cycle_is_refused_naming_its_systems() {
        let err = order(&[
            ("a", vec![system("x", phase::UPDATE, &["b::y"], &[])], vec![]),
            ("b", vec![system("y", phase::UPDATE, &["a::x"], &[])], vec![]),
            ("c", vec![system("z", phase::UPDATE, &[], &[])], vec![]),
        ])
        .unwrap_err();
        assert_eq!(err, "the systems a::x, b::y are ordered in a cycle");
    }

    #[test]
    fn a_mod_can_add_a_phase_between_the_engines() {
        let got = order(&[(
            "pong",
            vec![system("late", phase::LATE, &[], &[]), system("serve", "pong::serve", &[], &[])],
            vec![phase("pong::serve", &[phase::SIMULATE], &[phase::LATE])],
        )]);
        assert_eq!(got.unwrap(), ["pong::serve", "pong::late"]);
    }

    #[test]
    fn a_system_in_an_undeclared_phase_is_refused() {
        let err = order(&[("a", vec![system("x", "a::nowhere", &[], &[])], vec![])]).unwrap_err();
        assert_eq!(err, "a::x runs in phase a::nowhere, which no loaded mod declares");
    }

    #[test]
    fn phases_in_a_cycle_are_refused() {
        let err = order(&[("a", vec![], vec![phase("a::p", &[phase::RENDER], &[phase::INPUT])])]).unwrap_err();
        assert!(err.starts_with("the phases "), "{err}");
    }

    #[test]
    fn systems_are_numbered_in_frame_order() {
        let systems = [system("late", phase::LATE, &[], &[]), system("early", phase::INPUT, &[], &[])];
        let plan = plan(&[ModDecls { name: "m", systems: &systems, phases: &[] }]).unwrap();
        assert_eq!(plan.system(0).map(|s| s.name.as_str()), Some("m::early"));
        assert_eq!(plan.system(1).map(|s| s.name.as_str()), Some("m::late"));
        assert_eq!(plan.system(2), None);
    }

    #[test]
    fn the_plan_describes_itself_by_phase() {
        let systems = [system("read", phase::INPUT, &[], &[]), system("step", phase::UPDATE, &[], &[])];
        let plan = plan(&[ModDecls { name: "m", systems: &systems, phases: &[] }]).unwrap();
        assert_eq!(plan.describe(), "input: m::read\nupdate: m::step");
    }
}
