// Box3D behind the C ABI in shim.h. workerCount 1 with no task callbacks is
// Box3D's single-threaded mode: it creates no threads (types.h, b3WorldDef).

#include "bench/physics3d/shim.h"

#include "box3d/box3d.h"

#include <stdlib.h>

// Box3D's own samples and benchmark step with 4 substeps; b3World_Step has no
// default of its own.
#define DEFAULT_SUBSTEPS 4

struct Box3dWorld {
	b3WorldId world;
	int substeps;
	bool allow_sleep;
	b3BodyId* bodies;
	bool* fixed;
	uint32_t count;
	uint32_t capacity;
};

Box3dWorld* p3_box3d_create(const P3Config* config) {
	Box3dWorld* w = calloc(1, sizeof(Box3dWorld));
	b3WorldDef def = b3DefaultWorldDef();
	def.gravity = (b3Vec3){0.0f, -9.81f, 0.0f};
	def.workerCount = 1;
	def.enableSleep = config->allow_sleep != 0;
	def.capacity.dynamicBodyCount = (int)config->max_bodies;
	def.capacity.dynamicShapeCount = (int)config->max_bodies;
	def.capacity.contactCount = (int)config->max_bodies * 8;
	w->world = b3CreateWorld(&def);
	w->substeps = config->velocity_iters > 0 ? config->velocity_iters : DEFAULT_SUBSTEPS;
	w->allow_sleep = config->allow_sleep != 0;
	return w;
}

void p3_box3d_destroy(Box3dWorld* w) {
	b3DestroyWorld(w->world);
	free(w->bodies);
	free(w->fixed);
	free(w);
}

void p3_box3d_add(Box3dWorld* w, uint32_t n, const P3Spec* specs, uint32_t* out_handles) {
	if (w->count + n > w->capacity) {
		uint32_t cap = w->capacity ? w->capacity : 64;
		while (cap < w->count + n) cap *= 2;
		w->bodies = realloc(w->bodies, cap * sizeof(b3BodyId));
		w->fixed = realloc(w->fixed, cap * sizeof(bool));
		w->capacity = cap;
	}
	for (uint32_t i = 0; i < n; ++i) {
		const P3Spec* s = &specs[i];
		const bool fixed = s->fixed != 0;
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = fixed ? b3_staticBody : b3_dynamicBody;
		bd.position = (b3Pos){s->pos[0], s->pos[1], s->pos[2]};
		bd.enableSleep = w->allow_sleep;
		if (!fixed) {
			bd.linearVelocity = (b3Vec3){s->vel[0], s->vel[1], s->vel[2]};
			bd.motionLocks.angularX = true;
			bd.motionLocks.angularY = true;
			bd.motionLocks.angularZ = true;
		}
		b3BodyId body = b3CreateBody(w->world, &bd);

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.baseMaterial.friction = 0.5f;
		sd.baseMaterial.restitution = 0.0f;
		if (s->shape == 0) {
			const float r = s->dims[0];
			// Density for a mass of 1; b3Body_SetMassData afterwards would do too,
			// but costs a second mass update per body.
			sd.density = 1.0f / (4.0f / 3.0f * 3.14159265f * r * r * r);
			b3Sphere sphere = {{0.0f, 0.0f, 0.0f}, r};
			b3CreateSphereShape(body, &sd, &sphere);
		} else {
			sd.density = 1.0f / (8.0f * s->dims[0] * s->dims[1] * s->dims[2]);
			b3BoxHull box = b3MakeBoxHull(s->dims[0], s->dims[1], s->dims[2]);
			b3CreateHullShape(body, &sd, &box.base);
		}
		out_handles[i] = w->count;
		w->bodies[w->count] = body;
		w->fixed[w->count] = fixed;
		w->count += 1;
	}
}

void p3_box3d_step(Box3dWorld* w, float dt) { b3World_Step(w->world, dt, w->substeps); }

void p3_box3d_read(const Box3dWorld* w, uint32_t n, const uint32_t* handles, float* out) {
	for (uint32_t i = 0; i < n; ++i) {
		b3BodyId id = w->bodies[handles[i]];
		b3Pos p = b3Body_GetPosition(id);
		b3Vec3 v = b3Body_GetLinearVelocity(id);
		float* o = out + 6 * i;
		o[0] = (float)p.x;
		o[1] = (float)p.y;
		o[2] = (float)p.z;
		o[3] = v.x;
		o[4] = v.y;
		o[5] = v.z;
	}
}

// Walks every dynamic body's touching contacts (Box3D's "touching" includes
// speculative points, as Rapier's and Jolt's counts do). A pair of dynamics is
// seen from both sides and a pair with a static from one, hence the weights.
uint32_t p3_box3d_touching(const Box3dWorld* w) {
	uint64_t twice = 0;
	b3ContactData* data = NULL;
	int data_cap = 0;
	for (uint32_t i = 0; i < w->count; ++i) {
		if (w->fixed[i]) continue;
		b3BodyId body = w->bodies[i];
		int cap = b3Body_GetContactCapacity(body);
		if (cap > data_cap) {
			data_cap = cap;
			data = realloc(data, (size_t)cap * sizeof(b3ContactData));
		}
		int k = b3Body_GetContactData(body, data, data_cap);
		for (int j = 0; j < k; ++j) {
			b3BodyId a = b3Shape_GetBody(data[j].shapeIdA);
			b3BodyId b = b3Shape_GetBody(data[j].shapeIdB);
			b3BodyId other = B3_ID_EQUALS(a, body) ? b : a;
			twice += b3Body_GetType(other) == b3_staticBody ? 2 : 1;
		}
	}
	free(data);
	return (uint32_t)(twice / 2);
}

int32_t p3_box3d_substeps(const Box3dWorld* w) { return w->substeps; }

void p3_box3d_profile(const Box3dWorld* w, float* out4) {
	b3Profile p = b3World_GetProfile(w->world);
	out4[0] = p.step;
	out4[1] = p.pairs;
	out4[2] = p.collide;
	out4[3] = p.solve;
}
