/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Contact.b2ContactEdge;
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
		 * TODO
		*/
		final public function CreateJoint(p_def:b2JointDef):b2Joint
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
			return null;
		}

		/**
		 * Destroy a joint. This may cause the connected bodies to begin colliding.
		 * @warning This function is locked during callbacks.
		 * TODO
		 */
		final public function DestroyJoint(p_joint:b2Joint):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		* Take a time step. This performs collision detection, integration,
		* and constraint solution.
		* @param p_timeStep the amount of time to simulate, this should not vary.
		* @param p_velocityIterations for the velocity constraint solver.
		* @param p_positionIterations for the position constraint solver.
		 * TODO
		*/
		final public function Step(	p_timeStep:Number, p_velocityIterations:int, p_positionIterations:int):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		* Manually clear the force buffer on all bodies. By default, forces are cleared automatically
		* after each call to Step. The default behavior is modified by calling SetAutoClearForces.
		* The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
		* a fixed sized time step under a variable frame-rate.
		* When you perform sub-stepping you will disable auto clearing of forces and instead call
		* ClearForces after all sub-steps are complete in one pass of your game loop.
		* @see SetAutoClearForces
		 * TODO
		*/
		final public function ClearForces():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		* Query the world for all fixtures that potentially overlap the
		* provided AABB.
		* @param p_callback a user implemented callback class.
		* @param p_aabb the query box.
		 * TODO
		*/
		final public function QueryAABB(p_callback:b2QueryCallback, p_aabb:b2AABB):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
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
		 * TODO
		*/
		final public function RayCast(p_callback:b2RayCastCallback, p_point1X:Number, p_point1Y:Number, p_point2X:Number, p_point2Y:Number):void
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
