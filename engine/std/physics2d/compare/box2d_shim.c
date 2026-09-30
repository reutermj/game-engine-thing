// A handful of plain C calls over Box2D v3, so the Rust side's FFI is a
// few scalars and pointers to floats instead of Box2D's definition structs,
// whose layout (unions, callbacks, padding) would have to be mirrored in
// Rust and kept in step with the pinned version by hand.
//
// Every body is set up the way the comparison needs (see compare.rs):
// mass 1 whatever its shape, rotation locked or its inertia that of its
// shape at mass 1, friction mixed by the least and restitution by the
// greatest of the two, as //engine/std/physics2d does.

#include <stdlib.h>
#include <string.h>

#include "box2d/box2d.h"

// A task system for Box2D's multithreaded step (`bx_new_threads`): `workers
// - 1` threads kept for the world's life, the caller the last, as the physics
// mod's threads are the host's (tests/pool.rs does the same for ours). A
// worker spins for work, for 50 µs as tests/pool.rs does, then sleeps until
// a task is enqueued. Box2D asks for tasks only from the thread that steps
// (`b2World_Step`); its solver's tasks wait on each other, so every task
// enqueued is open to every thread at once, and the stepping thread helps
// while it waits in `bx_finish`. Which cores the threads run on is the
// process's (`taskset`), as for ours.
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>

#define BX_SLOTS 256
#define BX_WORKERS 64

typedef struct bx_task
{
	b2TaskCallback* fn;
	void* context;
	int items;
	int chunks;
	atomic_int next;
	atomic_int done;
} bx_task;

typedef struct bx_pool bx_pool;

typedef struct bx_worker
{
	bx_pool* pool;
	int index;
} bx_worker;

struct bx_pool
{
	int workers;
	pthread_t threads[BX_WORKERS];
	bx_worker me[BX_WORKERS];
	// A ring of tasks: Box2D finishes every task a step enqueues before the
	// step returns, a few dozen, so a slot is never reused while open.
	bx_task tasks[BX_SLOTS];
	atomic_int posted;
	atomic_int stop;
	atomic_int sleepers;
	pthread_mutex_t lock;
	pthread_cond_t wake;
};

// Runs one chunk of any open task as worker `index`: 1 if there was one.
static int bx_help( bx_pool* p, int index )
{
	int posted = atomic_load( &p->posted );
	int from = posted > 64 ? posted - 64 : 0;
	for ( int i = from; i < posted; ++i )
	{
		bx_task* t = p->tasks + ( i % BX_SLOTS );
		if ( atomic_load( &t->next ) >= t->chunks )
		{
			continue;
		}
		int k = atomic_fetch_add( &t->next, 1 );
		if ( k >= t->chunks )
		{
			continue;
		}
		int start = (int)( (long)t->items * k / t->chunks );
		int end = (int)( (long)t->items * ( k + 1 ) / t->chunks );
		t->fn( start, end, (uint32_t)index, t->context );
		atomic_fetch_add( &t->done, 1 );
		return 1;
	}
	return 0;
}

static double bx_seconds( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void* bx_work( void* arg )
{
	bx_worker* me = arg;
	bx_pool* p = me->pool;
	double idle = bx_seconds();
	while ( atomic_load( &p->stop ) == 0 )
	{
		if ( bx_help( p, me->index ) )
		{
			idle = bx_seconds();
			continue;
		}
		if ( bx_seconds() - idle < 50e-6 )
		{
			continue;
		}
		int seen = atomic_load( &p->posted );
		pthread_mutex_lock( &p->lock );
		atomic_fetch_add( &p->sleepers, 1 );
		while ( atomic_load( &p->posted ) == seen && atomic_load( &p->stop ) == 0 )
		{
			pthread_cond_wait( &p->wake, &p->lock );
		}
		atomic_fetch_sub( &p->sleepers, 1 );
		pthread_mutex_unlock( &p->lock );
		idle = bx_seconds();
	}
	return NULL;
}

static void* bx_enqueue( b2TaskCallback* fn, int items, int min_range, void* context, void* user )
{
	bx_pool* p = user;
	int chunks = min_range > 0 ? items / min_range : items;
	chunks = chunks < 1 ? 1 : ( chunks > 4 * p->workers ? 4 * p->workers : chunks );
	int n = atomic_load( &p->posted );
	bx_task* t = p->tasks + ( n % BX_SLOTS );
	t->fn = fn;
	t->context = context;
	t->items = items;
	t->chunks = chunks;
	atomic_store( &t->done, 0 );
	atomic_store( &t->next, 0 );
	atomic_store( &p->posted, n + 1 );
	if ( atomic_load( &p->sleepers ) > 0 )
	{
		pthread_mutex_lock( &p->lock );
		pthread_cond_broadcast( &p->wake );
		pthread_mutex_unlock( &p->lock );
	}
	return t;
}

static void bx_finish( void* task, void* user )
{
	bx_task* t = task;
	while ( atomic_load( &t->done ) < t->chunks )
	{
		bx_help( user, 0 );
	}
}

static bx_pool* bx_pool_new( int workers )
{
	bx_pool* p = calloc( 1, sizeof( bx_pool ) );
	p->workers = workers;
	pthread_mutex_init( &p->lock, NULL );
	pthread_cond_init( &p->wake, NULL );
	for ( int i = 1; i < workers; ++i )
	{
		p->me[i] = ( bx_worker ){ p, i };
		pthread_create( p->threads + i, NULL, bx_work, p->me + i );
	}
	return p;
}

static void bx_pool_free( bx_pool* p )
{
	pthread_mutex_lock( &p->lock );
	atomic_store( &p->stop, 1 );
	pthread_cond_broadcast( &p->wake );
	pthread_mutex_unlock( &p->lock );
	for ( int i = 1; i < p->workers; ++i )
	{
		pthread_join( p->threads[i], NULL );
	}
	free( p );
}

typedef struct bx_world
{
	b2WorldId id;
	b2BodyId* bodies;
	int count;
	int capacity;
	// Its threads, if it has more than one.
	bx_pool* pool;
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
bx_world* bx_new_threads( float gx, float gy, int sleep, int continuous, int workers );
void bx_free( bx_world* w );
int bx_add( bx_world* w, int dynamic, int circle, float x, float y, float hx, float hy, float friction, float restitution, int turning,
			float angle, float mass, float gravity_scale );
void bx_set_spin( bx_world* w, int handle, float spin );
int bx_awake( const bx_world* w, int handle );
int bx_marks( const bx_world* w, int handle, float* out, int capacity );
void bx_set_velocity( bx_world* w, int handle, float vx, float vy );
void bx_remove( bx_world* w, int handle );
void bx_step( bx_world* w, float dt, int substeps );
void bx_state( const bx_world* w, int handle, float* out );
int bx_profile( const bx_world* w, float* out, int len );
int bx_counters( const bx_world* w, int* out, int len );

bx_world* bx_new( float gx, float gy, int sleep, int continuous )
{
	return bx_new_threads( gx, gy, sleep, continuous, 1 );
}

// On `workers` threads, the caller's included: one is Box2D's default, with
// no task system.
bx_world* bx_new_threads( float gx, float gy, int sleep, int continuous, int workers )
{
	b2WorldDef def = b2DefaultWorldDef();
	def.gravity = (b2Vec2){ gx, gy };
	def.enableSleep = sleep != 0;
	def.enableContinuous = continuous != 0;
	def.frictionCallback = least;
	def.restitutionCallback = greatest;
	bx_world* w = calloc( 1, sizeof( bx_world ) );
	if ( workers > 1 )
	{
		w->pool = bx_pool_new( workers < BX_WORKERS ? workers : BX_WORKERS );
		def.workerCount = w->pool->workers;
		def.enqueueTask = bx_enqueue;
		def.finishTask = bx_finish;
		def.userTaskContext = w->pool;
	}
	w->id = b2CreateWorld( &def );
	return w;
}

void bx_free( bx_world* w )
{
	b2DestroyWorld( w->id );
	if ( w->pool != NULL )
	{
		bx_pool_free( w->pool );
	}
	free( w->bodies );
	free( w );
}

int bx_add( bx_world* w, int dynamic, int circle, float x, float y, float hx, float hy, float friction, float restitution, int turning,
			float angle, float mass, float gravity_scale )
{
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = dynamic ? b2_dynamicBody : b2_staticBody;
	bd.position = (b2Vec2){ x, y };
	bd.rotation = b2MakeRot( angle );
	bd.gravityScale = gravity_scale;
	bd.fixedRotation = turning == 0;
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
		// The scene's mass (1 but where a scene sets one), rather than by
		// density and area. Locked, inertia 0, not any value: this call sets
		// the inverse inertia from it without looking at fixedRotation, so a
		// body given 1 rotates after all (see
		// docs/lore/box2d-set-mass-data-unlocks-a-fixed-rotation.md).
		// Turning, the inertia of the shape at that mass about its center,
		// as the engine's `Collider::inertia_per_mass` has it.
		float inertia = mass * ( circle ? 0.5f * hx * hx : ( hx * hx + hy * hy ) / 3.0f );
		b2MassData m = { mass, { 0.0f, 0.0f }, turning ? inertia : 0.0f };
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

void bx_set_spin( bx_world* w, int handle, float spin )
{
	b2Body_SetAngularVelocity( w->bodies[handle], spin );
}

int bx_awake( const bx_world* w, int handle )
{
	return b2Body_IsAwake( w->bodies[handle] ) ? 1 : 0;
}

// The body's contacts that have points, for the debug view: per point x,
// y, the normal (from shape A to B) and its separation, 5 floats, at most
// `capacity` points. Returns how many it wrote.
int bx_marks( const bx_world* w, int handle, float* out, int capacity )
{
	b2BodyId body = w->bodies[handle];
	int n = b2Body_GetContactCapacity( body );
	if ( n == 0 )
	{
		return 0;
	}
	b2ContactData* data = malloc( (size_t)n * sizeof( b2ContactData ) );
	n = b2Body_GetContactData( body, data, n );
	int written = 0;
	for ( int i = 0; i < n; ++i )
	{
		const b2Manifold* m = &data[i].manifold;
		for ( int k = 0; k < m->pointCount && written < capacity; ++k )
		{
			float* o = out + 5 * written;
			o[0] = m->points[k].point.x;
			o[1] = m->points[k].point.y;
			o[2] = m->normal.x;
			o[3] = m->normal.y;
			o[4] = m->points[k].separation;
			written += 1;
		}
	}
	free( data );
	return written;
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

// Position, velocity, angle and angular velocity: x, y, vx, vy, radians,
// radians a second.
void bx_state( const bx_world* w, int handle, float* out )
{
	b2Vec2 p = b2Body_GetPosition( w->bodies[handle] );
	b2Vec2 v = b2Body_GetLinearVelocity( w->bodies[handle] );
	out[0] = p.x;
	out[1] = p.y;
	out[2] = v.x;
	out[3] = v.y;
	out[4] = b2Rot_GetAngle( b2Body_GetRotation( w->bodies[handle] ) );
	out[5] = b2Body_GetAngularVelocity( w->bodies[handle] );
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
