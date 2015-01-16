/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Contact.b2ContactEdge;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Structures.b2DistanceData;
	import Box2D.Collision.Structures.b2DistanceProxy;
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Collision.Structures.b2SeparationFunction;
	import Box2D.Collision.Structures.b2SimplexCache;
	import Box2D.Collision.Structures.b2TOIData;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2BroadPhase;
	import Box2D.Collision.b2Collision;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Callbacks.b2ContactListener;
	import Box2D.Dynamics.Callbacks.b2DestructionListener;
	import Box2D.Dynamics.Callbacks.b2QueryCallback;
	import Box2D.Dynamics.Callbacks.b2RayCastCallback;
	import Box2D.Dynamics.Def.b2BodyDef;
	import Box2D.Dynamics.Def.b2JointDef;
	import Box2D.Dynamics.Filters.b2Filter;
	import Box2D.Dynamics.Joints.b2Joint;
	import Box2D.Dynamics.Joints.b2JointEdge;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 * The world class manages all physics entities, dynamic simulation,
	 * and asynchronous queries. The world also contains efficient memory
	 * management facilities.
	 */
	public class b2World
	{
		//
		static b2internal var e_newFixture:uint = 0x0001;
		static b2internal var e_locked:uint = 0x0002;
		static b2internal var e_clearForces:uint = 0x0004;

		static b2internal var worldRef:b2World;

		b2internal var m_contactManager:b2ContactManager;
		b2internal var m_flags:int;

		b2internal var m_bodyList:b2Body;
		b2internal var m_jointList:b2Joint;

		b2internal var m_bodyCount:int;
		b2internal var m_jointCount:int;

		b2internal var m_gravity:b2Vec2;
		b2internal var m_allowSleep:Boolean;

		// This is used to compute the time step ratio to
		// support a variable time step.
		b2internal var m_inv_dt0:Number;

		// These are for debugging the solver.
		b2internal var m_warmStarting:Boolean;
		b2internal var m_continuousPhysics:Boolean;
		b2internal var m_subStepping:Boolean;
		b2internal var m_stepComplete:Boolean;

		b2internal var m_destructionListener:b2DestructionListener;  // TODO: Think about remove that functionality

		//
		private var _worldQueryWrapper:b2WorldQueryWrapper;
		private var _worldRayCastWrapper:b2WorldRayCastWrapper;
		private var _rayCastHelper:b2RayCastData;
		private var _step:b2TimeStep;
		private var _subStep:b2TimeStep;
		private var _island:b2Island;
		private var _stack:Vector.<b2Body>;
		private var _toiData:b2TOIData;
		private var _backupSweep1:b2Sweep;
		private var _backupSweep2:b2Sweep;
		private var _bodiesHelper:Vector.<b2Body>;
		private var _cacheSimplex:b2SimplexCache;
		private var _distanceData:b2DistanceData;
		private var _xfA:b2Mat22;
		private var _xfB:b2Mat22;
		private var _fcn:b2SeparationFunction;
		private var _pointHelper:b2SPoint;

		// TODO: add some debugging props from original

		/**
		 */
		public function b2World(p_gravityX:Number = 0.0, p_gravityY:Number = 10.0)
		{
			m_gravity = b2Vec2.Get(p_gravityX, p_gravityY);

			m_bodyCount = 0;
			m_jointCount = 0;

			m_warmStarting = true;
			m_continuousPhysics = true;
			m_subStepping = false;
			m_stepComplete = true;
			m_allowSleep = true;
			m_flags = e_clearForces;

			m_inv_dt0 = 0.0;

			m_contactManager = new b2ContactManager();

			//
			_worldQueryWrapper = new b2WorldQueryWrapper();
			_worldRayCastWrapper = new b2WorldRayCastWrapper();
			_rayCastHelper = new b2RayCastData();
			_step = new b2TimeStep();
			_subStep = new b2TimeStep();
			_island = new b2Island();
			_stack = new <b2Body>[];
			_toiData = new b2TOIData();
			_backupSweep1 = b2Sweep.Get();
			_backupSweep2 = b2Sweep.Get();
			_bodiesHelper = new Vector.<b2Body>(2);
			_cacheSimplex = new b2SimplexCache();
			_distanceData = new b2DistanceData();
			_xfA = b2Mat22.Get();
			_xfB = b2Mat22.Get();
			_fcn = new b2SeparationFunction();
			_pointHelper = b2SPoint.Get();

			worldRef = this;
		}

		/**
		 * Register a destruction listener. The listener is owned by you and must remain in scope.
		 */
		final public function SetDestructionListener(p_listener:b2DestructionListener):void
		{
			m_destructionListener = p_listener;
		}

		/**
		 * Register a contact filter to provide specific control over collision.
		 * Otherwise the default filter is used (b2_defaultFilter). The listener is
		 * owned by you and must remain in scope.
		 */
		[Inline]
		final public function SetContactFilter(p_filter:b2Filter):void
		{
			m_contactManager.m_contactFilter = p_filter;
		}

		/**
		 * Register a contact event listener. The listener is owned by you and must remain in scope.
		 */
		[Inline]
		final public function SetContactListener(p_listener:b2ContactListener):void
		{
			m_contactManager.m_contactListener = p_listener;
		}

		/**
		 * Create a rigid body given a definition. No reference to the definition is retained.
		 * @warning This function is locked during callbacks.
		 */
		final public function CreateBody(p_def:b2BodyDef):b2Body
		{
			CONFIG::debug
			{
				b2Assert(IsLocked == false, "the world is locked")
			}

			if (IsLocked)
			{
				return null;
			}

			var b:b2Body = b2Body.Get(p_def, this);

			// Add to world doubly linked list.
			b.m_prev = null;
			b.m_next = m_bodyList;

			if (m_bodyList)
			{
				m_bodyList.m_prev = b;
			}

			m_bodyList = b;
			++m_bodyCount;

			return b;
		}

		/**
		 * Destroy a rigid body given a definition. No reference to the definition
		 * is retained. This function is locked during callbacks.
		 * @warning This automatically deletes all associated shapes and joints.
		 * @warning This function is locked during callbacks.
		 */
		final public function DestroyBody(b:b2Body):void
		{
			CONFIG::debug
			{
				b2Assert(m_bodyCount > 0, "count of bodies is 0");
				b2Assert(IsLocked == false, "world is locked");
			}

			if (IsLocked)
			{
				return;
			}

			// Delete the attached joints.
			var je:b2JointEdge = b.m_jointList;
			var je0:b2JointEdge;
			var isDestructionListener:Boolean = m_destructionListener != null;

			while (je)
			{
				je0 = je;
				je = je.next;

				if (isDestructionListener)
				{
					m_destructionListener.SayGoodbyeJoint(je0.joint);
				}

				DestroyJoint(je0.joint);

				b.m_jointList = je;
			}

			b.m_jointList = null;

			// Delete the attached contacts.
			var ce:b2ContactEdge = b.m_contactList;
			var ce0:b2ContactEdge;
			var contactManager:b2ContactManager = m_contactManager;

			while (ce)
			{
				ce0 = ce;
				ce = ce.next;
				contactManager.Destroy(ce0.contact);
			}

			b.m_contactList = null;

			// Delete the attached fixtures. This destroys broad-phase proxies.
			var f:b2Fixture = b.m_fixtureList;
			var f0:b2Fixture;
			var broadPhase:b2BroadPhase = contactManager.m_broadPhase;

			while (f)
			{
				f0 = f;
				f = f.m_next;

				if (isDestructionListener)
				{
					m_destructionListener.SayGoodbyeFixture(f0);
				}

				f0.DestroyProxies(broadPhase);
				f0.Dispose();

				b.m_fixtureList = f;
				b.m_fixtureCount -= 1;
			}

			b.m_fixtureList = null;
			b.m_fixtureCount = 0;

			// Remove world body list.
			if (b.m_prev)
			{
				b.m_prev.m_next = b.m_next;
			}

			if (b.m_next)
			{
				b.m_next.m_prev = b.m_prev;
			}

			if (b == m_bodyList)
			{
				m_bodyList = b.m_next;
			}

			--m_bodyCount;

			b.Dispose();
		}

		/**
		 * Create a joint to constrain bodies together. No reference to the definition
		 * is retained. This may cause the connected bodies to cease colliding.
		 * @warning This function is locked during callbacks.
		 */
		final public function CreateJoint(p_def:b2JointDef):b2Joint
		{
			CONFIG::debug
			{
				b2Assert(IsLocked == false, "world is locked");
			}

			if (IsLocked)
			{
				return null;
			}

			var joint:b2Joint = b2Joint.Create(p_def);

			// Connect to the world list.
			joint.m_prev = null;
			joint.m_next = m_jointList;

			if (m_jointList)
			{
				m_jointList.m_prev = joint;
			}

			m_jointList = joint;
			++m_jointCount;

			// Connect to the bodies' doubly linked lists.
			var edgeA:b2JointEdge = joint.m_edgeA;
			var edgeB:b2JointEdge = joint.m_edgeB;

			var bodyA:b2Body = joint.m_bodyA;
			var bodyB:b2Body = joint.m_bodyB;

			var jointListA:b2JointEdge = bodyA.m_jointList;
			var jointListB:b2JointEdge = bodyB.m_jointList;

			edgeA.joint = joint;
			edgeA.other = bodyB;
			edgeA.prev = null;
			edgeA.next = jointListA;
			if (jointListA) jointListA.prev = edgeA;
			bodyA.m_jointList = edgeA;

			edgeB.joint = joint;
			edgeB.other = bodyA;
			edgeB.prev = null;
			edgeB.next = jointListB;
			if (jointListB) jointListB.prev = edgeB;
			bodyB.m_jointList = edgeB;

			bodyA = p_def.bodyA;
			bodyB = p_def.bodyB;

			// If the joint prevents collisions, then flag any contacts for filtering.
			if (p_def.collideConnected == false)
			{
				var edge:b2ContactEdge = bodyB.GetContactList();
				while (edge)
				{
					if (edge.other == bodyA)
					{
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
						edge.contact.FlagForFiltering();
					}

					edge = edge.next;
				}
			}

			// Note: creating a joint doesn't wake the bodies.

			return joint;
		}

		/**
		 * Destroy a joint. This may cause the connected bodies to begin colliding.
		 * @warning This function is locked during callbacks.
		 */
		final public function DestroyJoint(p_joint:b2Joint):void
		{
			CONFIG::debug
			{
				b2Assert(IsLocked == false, "world is locked");
			}

			if (IsLocked)
			{
				return;
			}

			var collideConnected:Boolean = p_joint.m_collideConnected;

			// Remove from the doubly linked list.
			if (p_joint.m_prev)
			{
				p_joint.m_prev.m_next = p_joint.m_next;
			}

			if (p_joint.m_next)
			{
				p_joint.m_next.m_prev = p_joint.m_prev;
			}

			if (p_joint == m_jointList)
			{
				m_jointList = p_joint.m_next;
			}

			// Disconnect from island graph.
			var bodyA:b2Body = p_joint.m_bodyA;
			var bodyB:b2Body = p_joint.m_bodyB;

			// Wake up connected bodies.
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);

			// Remove from body 1.
			var edgeA:b2JointEdge = p_joint.m_edgeA;
			var prevA:b2JointEdge = edgeA.prev;
			var nextA:b2JointEdge = edgeA.next;

			if (prevA)
			{
				prevA.next = nextA;
			}

			if (nextA)
			{
				nextA.prev = prevA;
			}

			if (edgeA == bodyA.m_jointList)
			{
				bodyA.m_jointList = nextA;
			}

			edgeA.prev = null;
			edgeA.next = null;

			// Remove from body 2
			var edgeB:b2JointEdge = p_joint.m_edgeB;
			var prevB:b2JointEdge = edgeB.prev;
			var nextB:b2JointEdge = edgeB.next;

			if (prevB)
			{
				prevB.next = nextB;
			}

			if (nextB)
			{
				nextB.prev = prevB;
			}

			if (edgeB == bodyB.m_jointList)
			{
				bodyB.m_jointList = nextB;
			}

			edgeB.prev = null;
			edgeB.next = null;

			b2Joint.Destroy(p_joint);

			CONFIG::debug
			{
				b2Assert(m_jointCount > 0, "jointCount equal 0");
			}

			--m_jointCount;

			// If the joint prevents collisions, then flag any contacts for filtering.
			if (collideConnected == false)
			{
				var edge:b2ContactEdge = bodyB.GetContactList();
				while (edge)
				{
					if (edge.other == bodyA)
					{
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
						edge.contact.FlagForFiltering();
					}

					edge = edge.next;
				}
			}
		}

		/**
		 * Take a time step. This performs collision detection, integration,
		 * and constraint solution.
		 * @param p_dt the amount of time to simulate, this should not vary.
		 * @param p_velocityIterations for the velocity constraint solver.
		 * @param p_positionIterations for the position constraint solver.
		 */
		final public function Step(p_dt:Number = 0.0333, p_velocityIterations:int = 10, p_positionIterations:int = 10):void
		{
			// If new fixtures were added, we need to find the new contacts.
			if ((m_flags & e_newFixture) != 0)
			{
				m_contactManager.FindNewContacts();
				m_flags &= ~e_newFixture;
			}

			m_flags |= e_locked;

			_step.dt = p_dt;
			_step.velocityIterations = p_velocityIterations;
			_step.positionIterations = p_positionIterations;

			if (p_dt > 0.0)
			{
				_step.inv_dt = 1.0 / p_dt;
			}
			else
			{
				_step.inv_dt = 0.0;
			}

			_step.dtRatio = m_inv_dt0 * p_dt;

			_step.warmStarting = m_warmStarting;

			// Update contacts. This is where some contacts are destroyed.
			m_contactManager.Collide();

			// Integrate velocities, solve velocity constraints, and integrate positions.
			if (m_stepComplete && _step.dt > 0.0)
			{
				Solve(_step);
			}

			// Handle TOI events.
			if (m_continuousPhysics && _step.dt > 0.0)
			{
				SolveTOI(_step);
			}

			if (_step.dt > 0.0)
			{
				m_inv_dt0 = _step.inv_dt;
			}

			if ((m_flags & e_clearForces) != 0)
			{
				ClearForces();
			}

			m_flags &= ~e_locked;
		}

		/**
		 * Manually clear the force buffer on all bodies. By default, forces are cleared automatically
		 * after each call to Step. The default behavior is modified by calling SetAutoClearForces.
		 * The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
		 * a fixed sized time step under a variable frame-rate.
		 * When you perform sub-stepping you will disable auto clearing of forces and instead call
		 * ClearForces after all sub-steps are complete in one pass of your game loop.
		 * @see SetAutoClearForces
		 */
		[Inline]
		final public function ClearForces():void
		{
			for (var body:b2Body = m_bodyList; body; body = body.GetNext())
			{
				body.m_forceX = 0.0;
				body.m_forceY = 0.0;
				body.m_torque = 0.0;
			}
		}

		/**
		 * Query the world for all fixtures that potentially overlap the
		 * provided AABB.
		 * @param p_callback a user implemented callback class.
		 * @param p_aabb the query box.
		 */
		[Inline]
		final public function QueryAABB(p_callback:b2QueryCallback, p_aabb:b2AABB):void
		{
			_worldQueryWrapper.broadPhase = m_contactManager.m_broadPhase;
			_worldQueryWrapper.callback = p_callback;
			m_contactManager.m_broadPhase.Query(_worldQueryWrapper, p_aabb);
		}

		/**
		 * Ray-cast the world for all fixtures in the path of the ray. Your callback
		 * controls whether you get the closest point, any point, or n-points.
		 * The ray-cast ignores shapes that contain the starting point.
		 * @param p_callback a user implemented callback class.
		 * @param p_point1X the ray starting point
		 * @param p_point1Y the ray starting point
		 * @param p_point2X the ray ending point
		 * @param p_point2Y the ray ending point
		 */
		final public function RayCast(p_callback:b2RayCastCallback, p_point1X:Number, p_point1Y:Number, p_point2X:Number, p_point2Y:Number):void
		{
			_worldRayCastWrapper.broadPhase = m_contactManager.m_broadPhase;
			_worldRayCastWrapper.callback = p_callback;

			_rayCastHelper.p1X = p_point1X;
			_rayCastHelper.p1Y = p_point1Y;
			_rayCastHelper.p2X = p_point2X;
			_rayCastHelper.p2Y = p_point2Y;
			_rayCastHelper.maxFraction = 1.0;

			m_contactManager.m_broadPhase.RayCast(_worldRayCastWrapper, _rayCastHelper);
		}

		/**
		 */
		final private function Solve(p_step:b2TimeStep):void
		{
			var b:b2Body;
			var other:b2Body;

			// Size the island for the worst case.
			_island.Initializer(m_bodyCount, m_contactManager.m_contactCount, m_jointCount, m_contactManager.m_contactListener);

			// Clear all the island flags.
			for (b = m_bodyList; b; b = b.m_next)
			{
				b.m_flags &= ~b2Body.e_islandFlag;
			}
			for (var c:b2Contact = m_contactManager.m_contactList; c; c = c.m_next)
			{
				c.m_flags &= ~b2Contact.e_islandFlag;
			}
			for (var j:b2Joint = m_jointList; j; j = j.m_next)
			{
				j.m_islandFlag = false;
			}

			// Build and simulate all awake islands.
			var stackSize:int = m_bodyCount;

			for (var seed:b2Body = m_bodyList; seed; seed = seed.m_next)
			{
				if ((seed.m_flags & b2Body.e_islandFlag) != 0)
				{
					continue;
				}

				if (seed.IsAwake() == false || seed.IsActive() == false)
				{
					continue;
				}

				// The seed can be dynamic or kinematic.
				if (seed.GetType() == b2Body.STATIC)
				{
					continue;
				}

				// Reset island and stack.
				_island.Clear();
				var stackCount:int = 0;
				_stack[stackCount++] = seed;
				seed.m_flags |= b2Body.e_islandFlag;

				// Perform a depth first search (DFS) on the constraint graph.

				while (stackCount > 0)
				{
					// Grab the next body off the stack and add it to the island.
					b = _stack[--stackCount];

					CONFIG::debug
					{
						b2Assert(b.IsActive(), "body isn't active");
					}

					_island.AddBody(b);

					// Make sure the body is awake.
					b.SetAwake(true);

					// To keep islands as small as possible, we don't
					// propagate islands across static bodies.
					if (b.GetType() == b2Body.STATIC)
					{
						continue;
					}

					// Search all contacts connected to this body.
					var contact:b2Contact;

					for (var ce:b2ContactEdge = b.m_contactList; ce; ce = ce.next)
					{
						contact = ce.contact;

						// Has this contact already been added to an island?
						if ((contact.m_flags & b2Contact.e_islandFlag) != 0)
						{
							continue;
						}

						// Is this contact solid and touching?
						if (contact.IsEnabled() == false || contact.IsTouching() == false)
						{
							continue;
						}

						// Skip sensors.
						var sensorA:Boolean = contact.m_fixtureA.m_isSensor;
						var sensorB:Boolean = contact.m_fixtureB.m_isSensor;

						if (sensorA || sensorB)
						{
							continue;
						}

						_island.AddContact(contact);
						contact.m_flags |= b2Contact.e_islandFlag;

						other = ce.other;

						// Was the other body already added to this island?
						if ((other.m_flags & b2Body.e_islandFlag) != 0)
						{
							continue;
						}

						CONFIG::debug
						{
							b2Assert(stackCount < stackSize, "!(stackCount < stackSize)");
						}

						_stack[stackCount++] = other;
						other.m_flags |= b2Body.e_islandFlag;
					}

					// Search all joints connect to this body.
					for (var je:b2JointEdge = b.m_jointList; je; je = je.next)
					{
						if (je.joint.m_islandFlag == true)
						{
							continue;
						}

						other = je.other;

						// Don't simulate joints connected to inactive bodies.
						if (other.IsActive() == false)
						{
							continue;
						}

						_island.AddJoint(je.joint);
						je.joint.m_islandFlag = true;

						if ((other.m_flags & b2Body.e_islandFlag) != 0)
						{
							continue;
						}

						CONFIG::debug
						{
							b2Assert(stackCount < stackSize, "!(stackCount < stackSize)");
						}

						_stack[stackCount++] = other;
						other.m_flags |= b2Body.e_islandFlag;
					}
				}

				_island.Solve(p_step, m_gravity.x, m_gravity.y, m_allowSleep);

				// Post solve cleanup.
				var bodyCount:int = _island.m_bodyCount;
				var bodyList:Vector.<b2Body> = _island.m_bodies;

				for (var i:int = 0; i < bodyCount; ++i)
				{
					// Allow static bodies to participate in other islands.
					b = bodyList[i];

					if (b.GetType() == b2Body.STATIC)
					{
						b.m_flags &= ~b2Body.e_islandFlag;
					}
				}
			}

			_stack.length = 0;

			// Synchronize fixtures, check for out of range bodies.
			for (b = m_bodyList; b; b = b.GetNext())
			{
				// If a body was not in an island then it did not move.
				if ((b.m_flags & b2Body.e_islandFlag) == 0)
				{
					continue;
				}

				if (b.GetType() == b2Body.STATIC)
				{
					continue;
				}

				// Update fixtures (for broad-phase).
				b.SynchronizeFixtures();
			}

			// Look for new contacts.
			m_contactManager.FindNewContacts();
		}

		/**
		 * Find TOI contacts and solve them.
		 */
		final private function SolveTOI(p_step:b2TimeStep):void
		{
			var b:b2Body;
			var c:b2Contact;

			//
			_island.Initializer(2 * b2Settings.maxTOIContacts, b2Settings.maxTOIContacts, 0, m_contactManager.m_contactListener);

			if (m_stepComplete)
			{
				for (b = m_bodyList; b; b = b.m_next)
				{
					b.m_flags &= ~b2Body.e_islandFlag;
					b.m_sweep.t0 = 0.0;
				}

				for (c = m_contactManager.m_contactList; c; c = c.m_next)
				{
					// Invalidate TOI
					c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
					c.m_toiCount = 0;
					c.m_toi = 1.0;
				}
			}

			// Find TOI events and solve them.
			var minContact:b2Contact;
			var minAlpha:Number = 1.0;
			var alpha:Number;
			var fA:b2Fixture;
			var fB:b2Fixture;
			var bA:b2Body;
			var bB:b2Body;
			var typeA:uint;
			var typeB:uint;
			var activeA:Boolean;
			var activeB:Boolean;
			var collideA:Boolean;
			var collideB:Boolean;
			var indexA:int;
			var indexB:int;

			while (true)
			{
				// Find the first TOI.
				for (c = m_contactManager.m_contactList; c; c = c.m_next)
				{
					// Is this contact disabled?
					if (c.IsEnabled() == false)
					{
						continue;
					}

					// Prevent excessive sub-stepping.
					if (c.m_toiCount > b2Settings.maxSubSteps)
					{
						continue;
					}

					alpha = 1.0;

					if ((c.m_flags & b2Contact.e_toiFlag) != 0)
					{
						// This contact has a valid cached TOI.
						alpha = c.m_toi;
					}
					else
					{
						fA = c.GetFixtureA();
						fB = c.GetFixtureB();

						// Is there a sensor?
						if (fA.IsSensor() || fB.IsSensor())
						{
							continue;
						}

						bA = fA.GetBody();
						bB = fB.GetBody();

						typeA = bA.m_type;
						typeB = bB.m_type;

						CONFIG::debug
						{
							b2Assert(typeA == b2Body.DYNAMIC || typeB == b2Body.DYNAMIC, "one of the two bodies isn't dynamic");
						}

						activeA = bA.IsAwake() && typeA != b2Body.STATIC;
						activeB = bB.IsAwake() && typeB != b2Body.STATIC;

						// Is at least one body active (awake and dynamic or kinematic)?
						if (activeA == false && activeB == false)
						{
							continue;
						}

						collideA = bA.IsBullet() || typeA != b2Body.DYNAMIC;
						collideB = bB.IsBullet() || typeB != b2Body.DYNAMIC;

						// Are these two non-bullet dynamic bodies?
						if (collideA == false && collideB == false)
						{
							continue;
						}

						// Compute the TOI for this contact.
						// Put the sweeps onto the same time interval.
						var alpha0:Number = bA.m_sweep.t0;
						var alphaBb0:Number = bB.m_sweep.t0;

						if (alpha0 < alphaBb0)
						{
							alpha0 = alphaBb0;
							bA.m_sweep.Advance(alpha0);
						}
						else if (alphaBb0 < alpha0)
						{
							bB.m_sweep.Advance(alpha0);
						}

						CONFIG::debug
						{
							b2Assert(alpha0 < 1.0, "!(alpha0 < 1.0)");
						}

						indexA = c.GetChildIndexA();
						indexB = c.GetChildIndexB();

						// Compute the time of impact in interval [0, minTOI]
						_toiData.proxyA.Set(fA.GetShape(), indexA);
						_toiData.proxyB.Set(fB.GetShape(), indexB);
						_toiData.sweepA.Set(bA.m_sweep);
						_toiData.sweepB.Set(bB.m_sweep);
						_toiData.tMax = 1.0;

						TimeOfImpact(_toiData);

						// Beta is the fraction of the remaining portion of the .
						var beta:Number = _toiData.t;
						if (_toiData.state == b2TOIData.e_touching)
						{
							alpha = b2Math.Min(alpha0 + (1.0 - alpha0) * beta, 1.0);
						}
						else
						{
							alpha = 1.0;
						}

						c.m_toi = alpha;
						c.m_flags |= b2Contact.e_toiFlag;
					}

					if (alpha < minAlpha)
					{
						// This is the minimum TOI found so far.
						minContact = c;
						minAlpha = alpha;
					}
				}

				if (minContact == null || 1.0 - 10.0 * b2Math.EPSILON < minAlpha)
				{
					// No more TOI events. Done!
					m_stepComplete = true;
					break;
				}

				// Advance the bodies to the TOI.
				fA = minContact.GetFixtureA();
				fB = minContact.GetFixtureB();
				bA = fA.GetBody();
				bB = fB.GetBody();

				_backupSweep1.Set(bA.m_sweep);
				_backupSweep2.Set(bB.m_sweep);

				bA.Advance(minAlpha);
				bB.Advance(minAlpha);

				// The TOI contact likely has some new contact points.
				minContact.Update(m_contactManager.m_contactListener);
				minContact.m_flags &= ~b2Contact.e_toiFlag;
				++minContact.m_toiCount;

				// Is the contact solid?
				if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
				{
					// Restore the sweeps.
					minContact.SetEnabled(false);
					bA.m_sweep.Set(_backupSweep1);
					bB.m_sweep.Set(_backupSweep2);
					bA.SynchronizeTransform();
					bB.SynchronizeTransform();

					continue;
				}

				bA.SetAwake(true);
				bB.SetAwake(true);

				// Build the island
				_island.Clear();
				_island.AddBody(bA);
				_island.AddBody(bB);
				_island.AddContact(minContact);

				bA.m_flags |= b2Body.e_islandFlag;
				bB.m_flags |= b2Body.e_islandFlag;
				minContact.m_flags |= b2Contact.e_islandFlag;

				// Get contacts on bodyA and bodyB
				_bodiesHelper[0] = bA;
				_bodiesHelper[1] = bB;

				var body:b2Body;

				for (var i:int = 0; i < 2; i++)
				{
					body = _bodiesHelper[i];

					if (body.m_type == b2Body.DYNAMIC)
					{
						for (var ce:b2ContactEdge = body.m_contactList; ce; ce = ce.next)
						{
							if (_island.m_bodyCount == _island.m_bodyCapacity)
							{
								break;
							}

							if (_island.m_contactCount == _island.m_contactCapacity)
							{
								break;
							}

							var contact:b2Contact = ce.contact;

							// Has this contact already been added to the island?
							if ((contact.m_flags & b2Contact.e_islandFlag) != 0)
							{
								continue;
							}

							// Only add static, kinematic, or bullet bodies.
							var other:b2Body = ce.other;
							if (other.m_type == b2Body.DYNAMIC &&
									body.IsBullet() == false && other.IsBullet() == false)
							{
								continue;
							}

							// Skip sensors.
							var sensorA:Boolean = contact.m_fixtureA.m_isSensor;
							var sensorB:Boolean = contact.m_fixtureB.m_isSensor;

							if (sensorA || sensorB)
							{
								continue;
							}

							// Tentatively advance the body to the TOI.
							_backupSweep1.Set(other.m_sweep);

							if ((other.m_flags & b2Body.e_islandFlag) == 0)
							{
								other.Advance(minAlpha);
							}

							// Update the contact points
							contact.Update(m_contactManager.m_contactListener);

							// Was the contact disabled by the user?
							// Are there contact points?
							if (contact.IsEnabled() == false || contact.IsTouching() == false)
							{
								other.m_sweep.Set(_backupSweep1);
								other.SynchronizeTransform();
								continue;
							}

							// Add the contact to the island
							contact.m_flags |= b2Contact.e_islandFlag;
							_island.AddContact(contact);

							// Has the other body already been added to the island?
							if ((other.m_flags & b2Body.e_islandFlag) != 0)
							{
								continue;
							}

							// Add the other body to the island.
							other.m_flags |= b2Body.e_islandFlag;

							if (other.m_type != b2Body.STATIC)
							{
								other.SetAwake(true);
							}

							_island.AddBody(other);
						}
					}
				}

				_subStep.dt = (1.0 - minAlpha) * p_step.dt;
				_subStep.inv_dt = 1.0 / _subStep.dt;
				_subStep.dtRatio = 1.0;
				_subStep.positionIterations = 20;
				_subStep.velocityIterations = p_step.velocityIterations;
				_subStep.warmStarting = false;

				_island.SolveTOI(_subStep, bA.m_islandIndex, bB.m_islandIndex);

				// Reset island flags and synchronize broad-phase proxies.
				var bodyCount:int = _island.m_bodyCount;
				var bodies:Vector.<b2Body> = _island.m_bodies;

				for (i = 0; i < bodyCount; ++i)
				{
					body = bodies[i];
					body.m_flags &= ~b2Body.e_islandFlag;

					if (body.m_type != b2Body.DYNAMIC)
					{
						continue;
					}

					body.SynchronizeFixtures();

					// Invalidate all contact TOIs on this displaced body.
					for (ce = body.m_contactList; ce; ce = ce.next)
					{
						ce.contact.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
					}
				}

				// Commit fixture proxy movements to the broad-phase so that new contacts are created.
				// Also, some contacts can be destroyed.
				m_contactManager.FindNewContacts();

				if (m_subStepping)
				{
					m_stepComplete = false;
					break;
				}
			}
		}

		/**
		 * Compute the upper bound on time before two shapes penetrate. Time is represented as
		 * a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
		 * non-tunneling collision. If you change the time interval, you should call this function
		 * again.
		 * Note: use b2Distance to compute the contact point and normal at the time of impact.
		 * CCD via the local separating axis method. This seeks progression
		 * by computing the largest time at which separation is maintained.
		 */
		public function TimeOfImpact(p_toiData:b2TOIData):void
		{
			p_toiData.state = b2TOIData.e_unknown;
			p_toiData.t = p_toiData.tMax;

			var proxyA:b2DistanceProxy = p_toiData.proxyA;
			var proxyB:b2DistanceProxy = p_toiData.proxyB;
			var sweepA:b2Sweep = _backupSweep1;
			sweepA.Set(p_toiData.sweepA);

			var sweepB:b2Sweep = _backupSweep2;
			sweepB.Set(p_toiData.sweepB);

			// Large rotations can make the root finder fail, so we normalize the
			// sweep angles.
			sweepA.Normalize();
			sweepB.Normalize();

			var tMax:Number = p_toiData.tMax;

			var totalRadius:Number = proxyA.m_radius + proxyB.m_radius;
			var target:Number = b2Math.Max(b2Settings.linearSlop, totalRadius - 3.0 * b2Settings.linearSlop);
			var tolerance:Number = 0.25 * b2Settings.linearSlop;

			CONFIG::debug
			{
				b2Assert(target > tolerance, "!(target > tolerance)");
			}

			var t1:Number = 0.0;
			var iter:int = 0;

			// Prepare input for distance query.
			_cacheSimplex.count = 0;

			_distanceData.proxyA = p_toiData.proxyA;
			_distanceData.proxyB = p_toiData.proxyB;
			_distanceData.useRadii = false;

			// The outer loop progressively attempts to compute new separating axes.
			// This loop terminates when an axis is repeated (no progress is made).

			while (true)
			{
				sweepA.GetTransform(_xfA, t1);
				sweepB.GetTransform(_xfB, t1);

				// Get the distance between shapes. We can also use the results
				// to get a separating axis.
				_distanceData.transformA = _xfA;
				_distanceData.transformB = _xfB;

				b2Collision.b2Distance(_distanceData, _cacheSimplex);

				// If the shapes are overlapped, we give up on continuous collision.
				if (_distanceData.distance <= 0.0)
				{
					// Failure!
					p_toiData.state = b2TOIData.e_overlapped;
					p_toiData.t = 0.0;
					break;
				}

				if (_distanceData.distance < target + tolerance)
				{
					// Victory!
					p_toiData.state = b2TOIData.e_touching;
					p_toiData.t = t1;
					break;
				}

				// Initialize the separating axis.
				_fcn.Initialize(_cacheSimplex, proxyA, sweepA, proxyB, sweepB, t1);

				// Compute the TOI on the separating axis. We do this by successively
				// resolving the deepest point. This loop is bounded by the number of vertices.
				var done:Boolean = false;
				var t2:Number = tMax;
				var pushBackIter:int = 0;
				var indexA:int;
				var indexB:int;
				var s2:Number;

				while (true)
				{
					// Find the deepest point at t2. Store the witness point indices.
					s2 = _fcn.FindMinSeparation(t2, _pointHelper);
					indexA = _pointHelper.x;
					indexB = _pointHelper.y;

					// Is the final configuration separated?
					if (s2 > target + tolerance)
					{
						// Victory!
						p_toiData.state = b2TOIData.e_separated;
						p_toiData.t = tMax;
						done = true;
						break;
					}

					// Has the separation reached tolerance?
					if (s2 > target - tolerance)
					{
						// Advance the sweeps
						t1 = t2;
						break;
					}

					// Compute the initial separation of the witness points.
					var s1:Number = _fcn.Evaluate(indexA, indexB, t1);

					// Check for initial overlap. This might happen if the root finder
					// runs out of iterations.
					if (s1 < target - tolerance)
					{
						p_toiData.state = b2TOIData.e_failed;
						p_toiData.t = t1;
						done = true;
						break;
					}

					// Check for touching
					if (s1 <= target + tolerance)
					{
						// Victory! t1 should hold the TOI (could be 0.0).
						p_toiData.state = b2TOIData.e_touching;
						p_toiData.t = t1;
						done = true;
						break;
					}

					// Compute 1D root of: f(x) - target = 0
					var rootIterCount:int = 0;
					var a1:Number = t1;
					var a2:Number = t2;

					while (true)
					{
						// Use a mix of the secant rule and bisection.
						var t:Number;
						if ((rootIterCount & 1) != 0)
						{
							// Secant rule to improve convergence.
							t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
						}
						else
						{
							// Bisection to guarantee progress.
							t = 0.5 * (a1 + a2);
						}

						++rootIterCount;

						var s:Number = _fcn.Evaluate(indexA, indexB, t);

						if (b2Math.Abs(s - target) < tolerance)
						{
							// t2 holds a tentative value for t1
							t2 = t;
							break;
						}

						// Ensure we continue to bracket the root.
						if (s > target)
						{
							a1 = t;
							s1 = s;
						}
						else
						{
							a2 = t;
							s2 = s;
						}

						if (rootIterCount == 50)
						{
							break;
						}
					}

					++pushBackIter;

					if (pushBackIter == b2Settings.maxPolygonVertices)
					{
						break;
					}
				}

				++iter;

				if (done)
				{
					break;
				}

				if (iter == b2Settings.maxIterations)
				{
					// Root finder got stuck. Semi-victory.
					p_toiData.state = b2TOIData.e_failed;
					p_toiData.t = t1;
					break;
				}
			}
		}

		/**
		 * Get the world body list. With the returned body, use b2Body::GetNext to get
		 * the next body in the world list. A NULL body indicates the end of the list.
		 * @return the head of the world body list.
		 */
		[Inline]
		final public function GetBodyList():b2Body
		{
			return m_bodyList;
		}

		/**
		 * Get the world joint list. With the returned joint, use b2Joint::GetNext to get
		 * the next joint in the world list. A NULL joint indicates the end of the list.
		 * @return the head of the world joint list.
		 */
		[Inline]
		final public function GetJointList():b2Joint
		{
			return m_jointList;
		}

		/**
		 * Get the world contact list. With the returned contact, use b2Contact::GetNext to get
		 * the next contact in the world list. A NULL contact indicates the end of the list.
		 * @return the head of the world contact list.
		 * @warning contacts are created and destroyed in the middle of a time step.
		 * Use b2ContactListener to avoid missing contacts.
		 */
		[Inline]
		final public function GetContactList():b2Contact
		{
			return m_contactManager.m_contactList
		}

		/**
		 * Enable/disable sleep.
		 */
		[Inline]
		final public function SetAllowSleeping(p_flag:Boolean):void
		{
			if (p_flag != m_allowSleep)
			{
				m_allowSleep = p_flag;

				if (!p_flag)
				{
					for (var b:b2Body = m_bodyList; b; b = b.m_next)
					{
						b.SetAwake(true);
					}
				}
			}
		}

		/**
		 */
		[Inline]
		final public function GetAllowSleeping():Boolean
		{
			return m_allowSleep;
		}

		/**
		 * Enable/disable warm starting. For testing.
		 */
		[Inline]
		final public function SetWarmStarting(p_flag:Boolean):void
		{
			m_warmStarting = p_flag;
		}

		/**
		 */
		[Inline]
		final public function GetWarmStarting():Boolean
		{
			return m_warmStarting;
		}

		/**
		 * Enable/disable continuous physics. For testing.
		 */
		[Inline]
		final public function SetContinuousPhysics(p_flag:Boolean):void
		{
			m_continuousPhysics = p_flag;
		}

		/**
		 */
		[Inline]
		final public function GetContinuousPhysics():Boolean
		{
			return m_continuousPhysics;
		}

		/**
		 * Enable/disable single stepped continuous physics. For testing.
		 */
		[Inline]
		final public function SetSubStepping(p_flag:Boolean):void
		{
			m_subStepping = p_flag;
		}

		/**
		 */
		[Inline]
		final public function GetSubStepping():Boolean
		{
			return m_subStepping;
		}

		/**
		 * Get the number of bodies.
		 */
		[Inline]
		final public function GetBodyCount():int
		{
			return m_bodyCount;
		}

		/**
		 */
		[Inline]
		final public function GetJointCount():int
		{
			return m_jointCount;
		}

		/**
		 */
		[Inline]
		final public function GetContactCount():int
		{
			return m_contactManager.m_contactCount;
		}

		/**
		 */
		[Inline]
		final public function SetGravity(p_gravityX:Number, p_gravityY:Number):void
		{
			m_gravity.x = p_gravityX;
			m_gravity.y = p_gravityY;
		}

		/**
		 */
		[Inline]
		final public function GetGravity():b2Vec2
		{
			return m_gravity;
		}

		/**
		 * Is the world locked (in the middle of a time step).
		 * @return
		 */
		[Inline]
		final public function get IsLocked():Boolean
		{
			return (m_flags & e_locked) == e_locked;
		}

		/**
		 */
		[Inline]
		final public function SetAutoClearForces(p_flag:Boolean):void
		{
			if (p_flag)
			{
				m_flags |= e_clearForces;
			}
			else
			{
				m_flags &= ~e_clearForces;
			}
		}

		/**
		 * Get the flag that controls automatic clearing of forces after each time step.
		 */
		[Inline]
		final public function GetAutoClearForces():Boolean
		{
			return (m_flags & e_clearForces) == e_clearForces;
		}

		/**
		 */
		[Inline]
		final public function GetContactManager():b2ContactManager
		{
			return m_contactManager;
		}

		/**
		 * Dispose the world.
		 */
		[Inline]
		final public function Dispose():void
		{
			// Some shapes allocate using b2Alloc.
			var b:b2Body = m_bodyList;
			var bNext:b2Body;
			while (b)
			{
				bNext = b.m_next;

				var f:b2Fixture = b.m_fixtureList;
				var fNext:b2Fixture;

				while (f)
				{
					fNext = f.m_next;
					f.m_proxyCount = 0;
					f.Dispose();
					f = fNext;
				}

				b = bNext;
			}
		}

		//
		static b2internal var bodyDefDefault:b2BodyDef = new b2BodyDef();

		/**
		 * Return body with shape represents box.
		 * Fof prototyping.
		 * @return b2Body
		 */
		static public function GetBox(p_width:Number = 100, p_height:Number = 100, p_type:Number = b2Body.STATIC):b2Body
		{
			bodyDefDefault.type = p_type;
			var body:b2Body = worldRef.CreateBody(bodyDefDefault);
			body.CreateFixture2(b2PolygonShape.GetAsBox(p_width, p_height), (p_type == b2Body.STATIC) ? 0.0 : 1.0);

			return body;
		}

		/**
		 * Return body with shape represents circle.
		 * Fof prototyping.
		 * @return b2Body
		 */
		static public function GetCircle(p_radius:Number = 50, p_type:Number = b2Body.STATIC):b2Body
		{
			bodyDefDefault.type = p_type;
			var body:b2Body = worldRef.CreateBody(bodyDefDefault);
			body.CreateFixture2(b2CircleShape.Get(p_radius), (p_type == b2Body.STATIC) ? 0.0 : 1.0);

			return body;
		}
	}
}

import Box2D.Collision.Structures.b2RayCastData;
import Box2D.Collision.b2BroadPhase;
import Box2D.Dynamics.Callbacks.b2QueryCallback;
import Box2D.Dynamics.Callbacks.b2RayCastCallback;
import Box2D.Dynamics.b2Fixture;
import Box2D.Dynamics.b2FixtureProxy;

// internal classes

internal class b2WorldQueryWrapper
{
	public function QueryCallback(proxyId:int):Boolean
	{
		var proxy:b2FixtureProxy = broadPhase.GetUserData(proxyId);
		return callback.ReportFixture(proxy.fixture);
	}

	public var broadPhase:b2BroadPhase;
	public var callback:b2QueryCallback;
}

/**
 */
internal class b2WorldRayCastWrapper
{
	/**
	 */
	public function RayCastCallback(p_rayCastData:b2RayCastData, proxyId:int):Number
	{
		var proxy:b2FixtureProxy = broadPhase.GetUserData(proxyId);
		var fixture:b2Fixture = proxy.fixture;
		var index:int = proxy.childIndex;
		var hit:Boolean = fixture.RayCast(p_rayCastData, index);

		if (hit)
		{
			var fraction:Number = p_rayCastData.fraction;
			var pointX:Number = (1.0 - fraction) * p_rayCastData.p1X + fraction * p_rayCastData.p2X;
			var pointY:Number = (1.0 - fraction) * p_rayCastData.p1Y + fraction * p_rayCastData.p2Y;

			return callback.ReportFixture(fixture, pointX, pointY, p_rayCastData.normalX, p_rayCastData.normalY, fraction);
		}

		return p_rayCastData.maxFraction;
	}

	public var broadPhase:b2BroadPhase;
	public var callback:b2RayCastCallback;
}