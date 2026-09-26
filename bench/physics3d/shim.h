// The C ABI the Rust backends call into, shared by the Jolt and Box3D shims
// so both backends marshal bodies the same way. Mirrored by hand in
// ffi.rs: change both together.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One body to add. shape 0 is a sphere of radius dims[0], shape 1 an
// axis-aligned box of half-extents dims. A fixed body is static and ignores vel.
typedef struct P3Spec {
	int32_t shape;
	float dims[3];
	float pos[3];
	float vel[3];
	int32_t fixed;
} P3Spec;

// velocity_iters and position_iters of 0 mean the engine's defaults. Box3D
// reads velocity_iters as its substep count and ignores position_iters.
typedef struct P3Config {
	uint32_t max_bodies;
	int32_t velocity_iters;
	int32_t position_iters;
	int32_t allow_sleep;
} P3Config;

typedef struct JoltWorld JoltWorld;

JoltWorld* p3_jolt_create(const P3Config* config);
void p3_jolt_destroy(JoltWorld* world);
// Adds n bodies; out_handles[i] is spec i's handle, dense in add order from 0.
void p3_jolt_add(JoltWorld* world, uint32_t n, const P3Spec* specs, uint32_t* out_handles);
void p3_jolt_step(JoltWorld* world, float dt);
// Writes position then velocity (6 floats) for each of the n handles.
void p3_jolt_read(const JoltWorld* world, uint32_t n, const uint32_t* handles, float* out);
// Body pairs with a contact manifold in the last step (touching or speculative).
uint32_t p3_jolt_touching(const JoltWorld* world);
// The iteration counts the world actually runs with.
void p3_jolt_iterations(const JoltWorld* world, int32_t* velocity, int32_t* position);

typedef struct Box3dWorld Box3dWorld;

Box3dWorld* p3_box3d_create(const P3Config* config);
void p3_box3d_destroy(Box3dWorld* world);
void p3_box3d_add(Box3dWorld* world, uint32_t n, const P3Spec* specs, uint32_t* out_handles);
void p3_box3d_step(Box3dWorld* world, float dt);
void p3_box3d_read(const Box3dWorld* world, uint32_t n, const uint32_t* handles, float* out);
uint32_t p3_box3d_touching(const Box3dWorld* world);
int32_t p3_box3d_substeps(const Box3dWorld* world);
// The last step's b3Profile, in milliseconds: step, pairs, collide, solve.
void p3_box3d_profile(const Box3dWorld* world, float* out4);

#ifdef __cplusplus
}
#endif
