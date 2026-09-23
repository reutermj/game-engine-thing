# Retrospective: a platformer, the second game (2026-09-23)

A small platformer played by an agent (Claude) through a text interface, on
the same lockstep bootstrap as pong. Chosen to exercise what pong didn't:
level data, one-shot input, mods acting on each other's entities, and
entities removed mid-system.

What was built: `platformer` (the player and the rules: running, jumping,
tile collision, coins, spikes, the goal), `walkers` (enemies that patrol,
hurt on touch and die when stomped), `level` (the map, `map.txt`, compiled
into the mod) and `platformer_text` (commands and drawing, over messages).

## The game as played

The agent read `state`, worked out each jump from the physics constants, and
won in three turns (255 frames, no deaths): every landing came out where it
was predicted. It then went back for the second coin and stomped the walker
by planning the jump from the walker's position and the two-frame window a
stomp allows, again from `state` alone. The same routes are now
`//platformer:platformer_test`.

Then, live:

- **A map edit** (a wider pit, two coins, a second walker) reloaded only
  `level`, which rebuilt the level and moved the player to the new start.
  The player's coins and deaths carried over.
- **A code-only change to `level`** reloaded it without rebuilding anything:
  the map was the same, so the played level, collected coins included, stayed.
- **Tuning `JUMP_SPEED`** was refused as a per-mod reload, and the game reload
  reloaded all four platformer mods to change one number (below).

## What held up

- **A second game needed nothing new from the engine.** It reused the
  lockstep bootstrap, messages and the clock unchanged. Pong found the
  bootstrap's limits; this game didn't hit them again, because it had the
  same needs.
- **A level is a mod.** Making the level a mod that builds entities from a
  map, and rebuilds only when the map changes, gave live level editing with
  nothing level-specific in the engine.
- **Generational entities earned their keep.** The level remembers every
  entity it spawned and despawns them all on a rebuild, including coins
  already collected and walkers already stomped. Those stale handles are
  no-ops instead of hitting whatever reused the slot.
- **Mods acting on each other's entities needed no API.** `walkers` hurts the
  player by setting `Player::hurt` and bounces it by setting `vy`; the rules
  respawn a hurt player. Plain components were enough.
- **Played routes make good tests.** The scripts the agent typed are the
  test cases, and test builds of `level` (another map, or the same map with
  different code) cover the live-edit paths.
- **The tests found a rule bug:** `engine_mod` passed `testonly` to one of
  the targets it generates and not the others. Fixed.

## Friction

- **Tuning lives in the interface.** `platformer`'s interface holds what
  dependents need (the player's size, the tile kinds) and also the tuning only
  the rules use (`JUMP_SPEED`, `GRAVITY`, `RUN_SPEED`), because they're all
  constants in one file. Changing a tuning value is an interface change, and
  reloads every dependent. Either the convention should be "tuning stays in
  the implementation", or tuning should be data (a component) that changes
  with no rebuild at all. The digest can't tell which constants a dependent
  actually used.
- **Level data doesn't fit components.** With no arrays and no singletons,
  the level is one entity per non-empty cell, and both `platformer` and
  `walkers` rebuild a map of tiles from them every frame. Fine for 140 tiles;
  it's work done twice per frame, growing with the level. A resource that
  can hold a grid would remove both.
- **Deferred despawns, three times.** Coins, stomped walkers and the level
  rebuild all collect entities during a query and despawn them after. The
  pattern is easy but always the same: a command buffer would absorb it.
  *(Addressed 2026-09-23 for the systems: coins and stomped walkers are
  despawned through `cx.commands()`. The level's rebuild runs in `load`,
  where the world can change directly.)*
- **Order matters, and is implicit.** `walkers` runs after the rules, so a
  hurt player respawns a frame later, and the stomp bounce lands after the
  rules' physics has already moved the player. It works because of load
  order, which pong also ran into. *(Addressed 2026-09-23: the order is
  declared, `walkers::walk` after `platformer::play` in `simulate`, with
  input applied in `input` from `Run`/`Jump` events. The one-frame lag
  itself is unchanged, and the replayed routes still pass.)*
- **Where does progress live?** `won` is on the player, so after a map edit
  the new level shows "YOU WIN". Whether progress belongs to the player, the
  level or something else is a game decision, but the engine gives no place
  for "state of this level" other than a component someone owns.
- **The agent played from the source.** Planning jumps needed `JUMP_SPEED`,
  `GRAVITY` and the stomp rule, which the agent read from the code. An agent
  without the source would need the interface to report the rules (a `rules`
  command), or would have to learn them by experiment.
