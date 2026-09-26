// Jolt behind the C ABI in shim.h: one PhysicsSystem per world, run on
// JobSystemSingleThreaded so every Jolt number is one core's.

#include "bench/physics3d/shim.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <atomic>
#include <mutex>
#include <vector>

using namespace JPH;

namespace {

// The two-layer setup from Jolt's HelloWorld: statics never test each other.
constexpr ObjectLayer kStatic = 0;
constexpr ObjectLayer kMoving = 1;

class ObjectPairFilter final : public ObjectLayerPairFilter {
public:
	bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override { return a == kMoving || b == kMoving; }
};

class BroadPhaseLayers final : public BroadPhaseLayerInterface {
public:
	uint GetNumBroadPhaseLayers() const override { return 2; }
	BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override { return BroadPhaseLayer(BroadPhaseLayer::Type(layer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override { return "layer"; }
#endif
};

class ObjectVsBroadPhase final : public ObjectVsBroadPhaseLayerFilter {
public:
	bool ShouldCollide(ObjectLayer a, BroadPhaseLayer b) const override {
		return a == kMoving || b == BroadPhaseLayer(BroadPhaseLayer::Type(kMoving));
	}
};

// Counts manifolds reported in a step. Jolt reports one per sub-shape pair,
// and every shape here is a single convex, so that is one per body pair: the
// same thing Rapier's has_any_active_contact counts (both include speculative
// contacts within their speculative distance). Sleeping pairs get no
// callback, so with sleep on this undercounts a settled pile.
class Counter final : public ContactListener {
public:
	std::atomic<uint32_t> count {0};
	void OnContactAdded(const Body&, const Body&, const ContactManifold&, ContactSettings&) override { count.fetch_add(1, std::memory_order_relaxed); }
	void OnContactPersisted(const Body&, const Body&, const ContactManifold&, ContactSettings&) override {
		count.fetch_add(1, std::memory_order_relaxed);
	}
};

// Jolt's type registry and allocator hooks are process globals.
void init_jolt_once() {
	static std::once_flag once;
	std::call_once(once, [] {
		RegisterDefaultAllocator();
		Factory::sInstance = new Factory();
		RegisterTypes();
	});
}

}  // namespace

struct JoltWorld {
	BroadPhaseLayers bp_layers;
	ObjectVsBroadPhase object_vs_bp;
	ObjectPairFilter pair_filter;
	Counter counter;
	// Sized for the 10k piles with room to spare: Jolt's step aborts (and
	// asserts in debug) when the temp allocator runs dry.
	TempAllocatorImpl temp {256u * 1024u * 1024u};
	JobSystemSingleThreaded jobs {cMaxPhysicsJobs};
	PhysicsSystem physics;
	std::vector<BodyID> bodies;
	uint32_t touching = 0;
	bool allow_sleep = true;
};

extern "C" JoltWorld* p3_jolt_create(const P3Config* config) {
	init_jolt_once();
	auto* w = new JoltWorld();
	const uint max_bodies = config->max_bodies;
	// Pairs and contact constraints: a packed pile has ~6 neighbours a body.
	w->physics.Init(max_bodies, 0, max_bodies * 16, max_bodies * 16, w->bp_layers, w->object_vs_bp, w->pair_filter);
	PhysicsSettings settings = w->physics.GetPhysicsSettings();
	if (config->velocity_iters > 0) settings.mNumVelocitySteps = uint(config->velocity_iters);
	if (config->position_iters > 0) settings.mNumPositionSteps = uint(config->position_iters);
	settings.mAllowSleeping = config->allow_sleep != 0;
	w->physics.SetPhysicsSettings(settings);
	w->physics.SetGravity(Vec3(0.0f, -9.81f, 0.0f));
	w->physics.SetContactListener(&w->counter);
	w->allow_sleep = config->allow_sleep != 0;
	return w;
}

extern "C" void p3_jolt_destroy(JoltWorld* w) {
	BodyInterface& bi = w->physics.GetBodyInterfaceNoLock();
	if (!w->bodies.empty()) {
		bi.RemoveBodies(w->bodies.data(), int(w->bodies.size()));
		bi.DestroyBodies(w->bodies.data(), int(w->bodies.size()));
	}
	delete w;
}

extern "C" void p3_jolt_add(JoltWorld* w, uint32_t n, const P3Spec* specs, uint32_t* out_handles) {
	BodyInterface& bi = w->physics.GetBodyInterfaceNoLock();
	// One array per motion type, since AddBodiesFinalize activates all or none.
	std::vector<BodyID> fixed, moving;
	for (uint32_t i = 0; i < n; ++i) {
		const P3Spec& s = specs[i];
		Ref<Shape> shape;
		if (s.shape == 0)
			shape = new SphereShape(s.dims[0]);
		else
			// Jolt's default convex radius (0.05) rounds the box's edges and
			// corners. Faces, where these boxes mostly rest, are exact, but the
			// harness measures sharp boxes, so a contact at a Jolt corner can read
			// up to 0.05 * (sqrt(3) - 1) = 0.037 deeper than Jolt thinks it is.
			shape = new BoxShape(Vec3(s.dims[0], s.dims[1], s.dims[2]));
		const bool is_fixed = s.fixed != 0;
		BodyCreationSettings b(shape, RVec3(s.pos[0], s.pos[1], s.pos[2]), Quat::sIdentity(),
							   is_fixed ? EMotionType::Static : EMotionType::Dynamic, is_fixed ? kStatic : kMoving);
		b.mFriction = 0.5f;
		b.mRestitution = 0.0f;
		b.mAllowSleeping = w->allow_sleep;
		if (!is_fixed) {
			b.mAllowedDOFs = EAllowedDOFs::TranslationX | EAllowedDOFs::TranslationY | EAllowedDOFs::TranslationZ;
			b.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
			b.mMassPropertiesOverride.mMass = 1.0f;
			b.mLinearVelocity = Vec3(s.vel[0], s.vel[1], s.vel[2]);
		}
		Body* body = bi.CreateBody(b);
		out_handles[i] = uint32_t(w->bodies.size());
		w->bodies.push_back(body->GetID());
		(is_fixed ? fixed : moving).push_back(body->GetID());
	}
	// AddBodiesPrepare may reorder the arrays it is given, which is why they
	// are copies and not w->bodies.
	if (!fixed.empty()) {
		auto state = bi.AddBodiesPrepare(fixed.data(), int(fixed.size()));
		bi.AddBodiesFinalize(fixed.data(), int(fixed.size()), state, EActivation::DontActivate);
	}
	if (!moving.empty()) {
		auto state = bi.AddBodiesPrepare(moving.data(), int(moving.size()));
		bi.AddBodiesFinalize(moving.data(), int(moving.size()), state, EActivation::Activate);
	}
	// Jolt asks for this after adding many bodies at once; the rain's small
	// batches go without, as a game adding a few bodies a frame would.
	if (n >= 256) w->physics.OptimizeBroadPhase();
}

extern "C" void p3_jolt_step(JoltWorld* w, float dt) {
	w->counter.count.store(0, std::memory_order_relaxed);
	w->physics.Update(dt, 1, &w->temp, &w->jobs);
	w->touching = w->counter.count.load(std::memory_order_relaxed);
}

extern "C" void p3_jolt_read(const JoltWorld* w, uint32_t n, const uint32_t* handles, float* out) {
	const BodyInterface& bi = w->physics.GetBodyInterfaceNoLock();
	for (uint32_t i = 0; i < n; ++i) {
		const BodyID id = w->bodies[handles[i]];
		const RVec3 p = bi.GetPosition(id);
		const Vec3 v = bi.GetLinearVelocity(id);
		float* o = out + 6 * i;
		o[0] = float(p.GetX());
		o[1] = float(p.GetY());
		o[2] = float(p.GetZ());
		o[3] = v.GetX();
		o[4] = v.GetY();
		o[5] = v.GetZ();
	}
}

extern "C" uint32_t p3_jolt_touching(const JoltWorld* w) { return w->touching; }

extern "C" void p3_jolt_iterations(const JoltWorld* w, int32_t* velocity, int32_t* position) {
	const PhysicsSettings& s = w->physics.GetPhysicsSettings();
	*velocity = int32_t(s.mNumVelocitySteps);
	*position = int32_t(s.mNumPositionSteps);
}
