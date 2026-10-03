//! Contacts grouped by level of the sweep in pair order (physics.md, "The
//! solver's speed"): what lets a lanes kernel solve several contacts at
//! once and still be the solve one contact at a time in pair order, bit
//! for bit. Both mods' lanes take it: 2D's as a variant (`Wide::Levels`),
//! 3D's as its default order.

use engine_ecs::shape::{Coloring, UNSOLVED};

/// Each of `n` contacts' level, `ends(i)` its two bodies' indices into
/// `moves`: one past the latest level of the contacts before it that
/// share a body it moves, which, in pair order, is the level that body's
/// last contact left, so one pass finds them all. A body that doesn't
/// move (a static, the one standing for every static) is in any number of
/// a level's contacts, since none of them changes it; a contact neither
/// end of which moves is `UNSOLVED`.
///
/// Solving the levels in turn, a contact sees each body it moves exactly
/// as the sweep would have left it, and no two contacts of a level share
/// one, so a level's contacts can go in lanes in any order: level
/// scheduling, as sparse triangular solves run in parallel (Anderson and
/// Saad, 1989). Unlike `Coloring::greedy` it never overflows: a pile has
/// hundreds of levels where it has six or seven colors, which is why
/// threads share colors (get-emj.90) and the order-keeping lanes take
/// levels. Written into a `Coloring` so `Coloring::pack` lays out both.
pub fn levels(n: usize, ends: impl Fn(usize) -> (u32, u32), moves: &[bool]) -> Coloring {
    let mut coloring = Coloring::default();
    // By body, in pair order so far: the next level.
    let mut next = vec![0u32; moves.len()];
    for i in 0..n {
        let (a, b) = ends(i);
        let (a, b) = (a as usize, b as usize);
        if !moves[a] && !moves[b] {
            coloring.of.push(UNSOLVED);
            continue;
        }
        let at = |e: usize| if moves[e] { next[e] } else { 0 };
        let level = at(a).max(at(b));
        for e in [a, b] {
            if moves[e] {
                next[e] = level + 1;
            }
        }
        if coloring.count.len() <= level as usize {
            coloring.count.resize(level as usize + 1, 0);
        }
        coloring.count[level as usize] += 1;
        coloring.of.push(level);
    }
    coloring
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain 0-1-2 and a contact apart, with body 3 static: each contact
    /// one past its moving bodies' last, the static never holding one
    /// back, and a contact between statics unsolved.
    #[test]
    fn a_contact_is_one_past_the_last_at_a_body_it_moves() {
        let pairs = [(0, 1), (1, 2), (0, 3), (2, 3), (4, 5), (3, 3), (1, 3)];
        let moves = [true, true, true, false, true, true];
        let c = levels(pairs.len(), |i| pairs[i], &moves);
        assert_eq!(c.of, [0, 1, 1, 2, 0, UNSOLVED, 2]);
        assert_eq!((c.count, c.overflow), (vec![2, 2, 2], 0));
    }
}
