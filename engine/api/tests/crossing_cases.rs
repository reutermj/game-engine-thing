//! Service signatures `service!` must refuse, as doctests that must fail to
//! compile (`//engine/api:crossing_test`): each a way the other side of a
//! call could keep a reference into a build that a reload unmaps
//! (get-y5t.9, `Crossing`). Every refused case has a twin that compiles,
//! the same code with the reference made owned or left to the call, so a
//! refusal is the signature's and not a mistake in the snippet: rustdoc's
//! `compile_fail` passes on any error.
//!
//! A caller can't keep a returned `&'static`:
//!
//! ```compile_fail
//! # use engine_api::Cx;
//! engine_api::service! {
//!     pub trait Names {
//!         fn name() -> &'static str;
//!     }
//! }
//! #[derive(Default)]
//! struct Kept(Option<&'static str>);
//! fn keep(cx: &mut Cx, kept: &mut Kept) {
//!     kept.0 = name(cx).ok();
//! }
//! ```
//!
//! nor one behind an alias:
//!
//! ```compile_fail
//! # use engine_api::Cx;
//! type Name = &'static str;
//! engine_api::service! {
//!     pub trait Names {
//!         fn name() -> Name;
//!     }
//! }
//! #[derive(Default)]
//! struct Kept(Option<Name>);
//! fn keep(cx: &mut Cx, kept: &mut Kept) {
//!     kept.0 = name(cx).ok();
//! }
//! ```
//!
//! What it keeps is owned:
//!
//! ```
//! # use engine_api::Cx;
//! engine_api::service! {
//!     pub trait Names {
//!         fn name() -> String;
//!     }
//! }
//! #[derive(Default)]
//! struct Kept(Option<String>);
//! fn keep(cx: &mut Cx, kept: &mut Kept) {
//!     kept.0 = name(cx).ok();
//! }
//! ```
//!
//! A provider can't keep an argument declared `&'static`:
//!
//! ```compile_fail
//! # use engine_api::{Cx, Mod};
//! engine_api::service! {
//!     pub trait Store {
//!         fn keep(name: &'static str);
//!     }
//! }
//! engine_api::mod_state! {
//!     #[derive(Default)]
//!     struct Keeper {}
//! }
//! #[derive(Default)]
//! struct Kept(Option<&'static str>);
//! impl Mod for Keeper {
//!     type Transient = Kept;
//! }
//! impl Store for Keeper {
//!     fn keep(&mut self, kept: &mut Kept, _: &mut Cx, name: &'static str) {
//!         kept.0 = Some(name);
//!     }
//! }
//! ```
//!
//! nor one borrowed for the call:
//!
//! ```compile_fail
//! # use engine_api::{Cx, Mod};
//! engine_api::service! {
//!     pub trait Store {
//!         fn keep(name: &str);
//!     }
//! }
//! engine_api::mod_state! {
//!     #[derive(Default)]
//!     struct Keeper {}
//! }
//! #[derive(Default)]
//! struct Kept(Option<&'static str>);
//! impl Mod for Keeper {
//!     type Transient = Kept;
//! }
//! impl Store for Keeper {
//!     fn keep(&mut self, kept: &mut Kept, _: &mut Cx, name: &str) {
//!         kept.0 = Some(name);
//!     }
//! }
//! ```
//!
//! nor a `&'static` inside what an argument borrows:
//!
//! ```compile_fail
//! # use engine_api::{Cx, Mod};
//! pub struct Named {
//!     pub name: &'static str,
//! }
//! engine_api::service! {
//!     pub trait Store {
//!         fn keep(named: &Named);
//!     }
//! }
//! engine_api::mod_state! {
//!     #[derive(Default)]
//!     struct Keeper {}
//! }
//! #[derive(Default)]
//! struct Kept(Option<&'static str>);
//! impl Mod for Keeper {
//!     type Transient = Kept;
//! }
//! impl Store for Keeper {
//!     fn keep(&mut self, kept: &mut Kept, _: &mut Cx, named: &Named) {
//!         kept.0 = Some(named.name);
//!     }
//! }
//! ```
//!
//! and it can't hand the caller one through a `&mut` either, by writing a
//! reference to its own image into the caller's slot:
//!
//! ```compile_fail
//! # use engine_api::{Cx, Mod};
//! engine_api::service! {
//!     pub trait Fill {
//!         fn fill(slot: &mut &str);
//!     }
//! }
//! engine_api::mod_state! {
//!     #[derive(Default)]
//!     struct Filler {}
//! }
//! impl Mod for Filler {
//!     type Transient = ();
//! }
//! impl Fill for Filler {
//!     fn fill(&mut self, _: &mut (), _: &mut Cx, slot: &mut &str) {
//!         *slot = "in the provider's image";
//!     }
//! }
//! ```
//!
//! What a provider keeps of an argument, it copies, and what it hands back
//! through a `&mut` is owned:
//!
//! ```
//! # use engine_api::{Cx, Mod};
//! engine_api::service! {
//!     pub trait Store {
//!         fn keep(name: &str);
//!         fn fill(slot: &mut String);
//!     }
//! }
//! engine_api::mod_state! {
//!     #[derive(Default)]
//!     struct Keeper {}
//! }
//! #[derive(Default)]
//! struct Kept(Option<String>);
//! impl Mod for Keeper {
//!     type Transient = Kept;
//! }
//! impl Store for Keeper {
//!     fn keep(&mut self, kept: &mut Kept, _: &mut Cx, name: &str) {
//!         kept.0 = Some(name.to_string());
//!     }
//!     fn fill(&mut self, _: &mut Kept, _: &mut Cx, slot: &mut String) {
//!         *slot = "owned".to_string();
//!     }
//! }
//! ```
