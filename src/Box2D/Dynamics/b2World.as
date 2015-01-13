/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Contact.b2ContactEdge;
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2BroadPhase;
	import Box2D.Common.Math.b2Vec2;
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
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Body;
	import Box2D.b2Assert;

	import flash.profiler.profile;

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

		private var _worldQueryWrapper:b2WorldQueryWrapper;
		private var _worldRayCastWrapper:b2WorldRayCastWrapper;
		private var _rayCastHelper:b2RayCastData;
		private var _step:b2TimeStep;
		private var _island:b2Island;
		private var _stack:Vector.<b2Body>;

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

			_worldQueryWrapper = new b2WorldQueryWrapper();
			_worldRayCastWrapper = new b2WorldRayCastWrapper();
			_rayCastHelper = new b2RayCastData();
			_step = new b2TimeStep();
			_island = new b2Island();
			_stack = new <b2Body>[];
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

			while(je)
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

			while(ce)
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

			while(f)
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
		final public function Step(p_dt:Number, p_velocityIterations:int, p_positionIterations:int):void
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
		 * TODO:
		 */
		final private function SolveTOI(p_step:b2TimeStep):void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
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
	}
}

import Box2D.Collision.Structures.b2RayCastData;
import Box2D.Collision.b2BroadPhase;
import Box2D.Dynamics.Callbacks.b2QueryCallback;
import Box2D.Dynamics.Callbacks.b2RayCastCallback;
import Box2D.Dynamics.b2Body;
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

/**
 *
 */
internal class BodyStack
{
	private var _stack:Vector.<b2Body>;
	private var _count:int = 0;

	/**
	 */
	public function BodyStack()
	{
		_stack = new <b2Body>[];
	}

	/**
	 */
	[Inline]
	final public function Push(p_element:b2Body):void
	{
		_stack[_count++] = p_element;
	}

	/**
	 */
	[Inline]
	final public function Pop():b2Body
	{
		return _stack[--_count];
	}

	/**
	 */
	[Inline]
	final public function GetCount():int
	{
		return _count;
	}

	/**
	 * Clear stack.
	 */
	public function Free():void
	{
		var count:int = _stack.length;
		for (var i:int = 0; i < count; i++)
		{
			_stack[i] = null;
		}

		_stack.length = 0;
		_count = 0;
	}
}
