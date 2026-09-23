//! `vault`'s interface: a resident mod's service.

engine_api::service! {
    pub trait Vault {
        /// Which build of vault is running.
        fn build() -> String;
        /// How many transient parts this build has made: one for the session.
        fn transients_made() -> u32;
        /// Ticks counted by the thread vault's transient part owns.
        fn ticks() -> u64;
    }
}
