// A handful of plain C calls over Box2D v3, so the Rust side's FFI is a
// few scalars and pointers to floats instead of Box2D's definition structs,
// whose layout (unions, callbacks, padding) would have to be mirrored in
// Rust and kept in step with the pinned version by hand.
//
// Every body is set up the way the comparison needs (see compare.rs):
// mass 1 whatever its shape, rotation locked, friction mixed by the least
// and restitution by the greatest of the two, as //engine/std/physics does.

#include <stdlib.h>
#include <string.h>

#include "box2d/box2d.h"

typedef struct bx_world
{
	b2WorldId id;
	b2BodyId* bodies;
	int count;
	int capacity;
} bx_world;

static float least( float a, int ma, float b, int mb )
{
	(void)ma;
	(void)mb;
	return a < b ? a : b;
}

static float greatest( float a, int ma, float b, int mb )
{
	(void)ma;
	(void)mb;
	return a > b ? a : b;
}

bx_world* bx_new( float gx, float gy, int sleep, int continuous );
void bx_free( bx_world* w );
int bx_add( bx_world* w, int dynamic, int circle, float x, float y, float hx, float hy, float friction, float restitution );
void bx_set_velocity( bx_world* w, int handle, float vx, float vy );
void bx_remove( bx_world* w, int handle );
void bx_step( bx_world* w, float dt, int substeps );
void bx_state( const bx_world* w, int handle, float* out );
int bx_profile( const bx_world* w, float* out, int len );
int bx_counters( const bx_world* w, int* out, int len );

bx_world* bx_new( float gx, float gy, int sleep, int continuous )
{
	b2WorldDef def = b2DefaultWorldDef();
	def.gravity = (b2Vec2){ gx, gy };
	def.enableSleep = sleep != 0;
	def.enableContinuous = continuous != 0;
	def.frictionCallback = least;
	def.restitutionCallback = greatest;
	// workerCount stays 1 and there is no task system: one thread.
	bx_world* w = calloc( 1, sizeof( bx_world ) );
	w->id = b2CreateWorld( &def );
	return w;
}

void bx_free( bx_world* w )
{
	b2DestroyWorld( w->id );
	free( w->bodies );
	free( w );
}

int bx_add( bx_world* w, int dynamic, int circle, float x, float y, float hx, float hy, float friction, float restitution )
{
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = dynamic ? b2_dynamicBody : b2_staticBody;
	bd.position = (b2Vec2){ x, y };
	bd.fixedRotation = true;
	b2BodyId body = b2CreateBody( w->id, &bd );

	b2ShapeDef sd = b2DefaultShapeDef();
	sd.material.friction = friction;
	sd.material.restitution = restitution;
	if ( circle )
	{
		b2Circle c = { { 0.0f, 0.0f }, hx };
		b2CreateCircleShape( body, &sd, &c );
	}
	else
	{
		b2Polygon p = b2MakeBox( hx, hy );
		b2CreatePolygonShape( body, &sd, &p );
	}
	if ( dynamic )
	{
		// Mass 1 for every body, as the engine's Body::default() has it,
		// rather than by density and area. Inertia 0, not any value: this
		// call sets the inverse inertia from it without looking at
		// fixedRotation, so a body given 1 rotates after all (see
		// docs/lore/box2d-set-mass-data-unlocks-a-fixed-rotation.md).
		b2MassData m = { 1.0f, { 0.0f, 0.0f }, 0.0f };
		b2Body_SetMassData( body, m );
	}

	if ( w->count == w->capacity )
	{
		w->capacity = w->capacity ? 2 * w->capacity : 1024;
		w->bodies = realloc( w->bodies, (size_t)w->capacity * sizeof( b2BodyId ) );
	}
	w->bodies[w->count] = body;
	return w->count++;
}

void bx_set_velocity( bx_world* w, int handle, float vx, float vy )
{
	b2Body_SetLinearVelocity( w->bodies[handle], (b2Vec2){ vx, vy } );
}

void bx_remove( bx_world* w, int handle )
{
	b2DestroyBody( w->bodies[handle] );
	w->bodies[handle] = b2_nullBodyId;
}

void bx_step( bx_world* w, float dt, int substeps )
{
	b2World_Step( w->id, dt, substeps );
}

// Position, velocity and angle: x, y, vx, vy, radians.
void bx_state( const bx_world* w, int handle, float* out )
{
	b2Vec2 p = b2Body_GetPosition( w->bodies[handle] );
	b2Vec2 v = b2Body_GetLinearVelocity( w->bodies[handle] );
	out[0] = p.x;
	out[1] = p.y;
	out[2] = v.x;
	out[3] = v.y;
	out[4] = b2Rot_GetAngle( b2Body_GetRotation( w->bodies[handle] ) );
}

// The last step's b2Profile, all floats in milliseconds, in declaration
// order. Returns how many it has, so the caller can check it agrees.
int bx_profile( const bx_world* w, float* out, int len )
{
	b2Profile p = b2World_GetProfile( w->id );
	int n = (int)( sizeof( p ) / sizeof( float ) );
	memcpy( out, &p, (size_t)( n < len ? n : len ) * sizeof( float ) );
	return n;
}

// b2Counters, all ints, in declaration order.
int bx_counters( const bx_world* w, int* out, int len )
{
	b2Counters c = b2World_GetCounters( w->id );
	int n = (int)( sizeof( c ) / sizeof( int ) );
	memcpy( out, &c, (size_t)( n < len ? n : len ) * sizeof( int ) );
	return n;
}
