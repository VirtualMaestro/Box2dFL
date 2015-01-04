/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Manifold.b2WorldManifold;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.b2Collision;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Callbacks.b2ContactListener;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 * The class manages contact between two shapes. A contact exists for each overlapping
	 * AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	 * that has no contact points.
	 */
	public class b2Contact
	{
		/**
		 * Used when crawling contact graph when forming islands.
		 */
		static b2internal var e_islandFlag:uint = 0x0001;

		/**
		 * Set when the shapes are touching.
		 */
		static b2internal var e_touchingFlag:uint = 0x0002;

		/**
		 * This contact can be disabled (by user).
		 */
		static b2internal var e_enabledFlag:uint = 0x0004;

		/**
		 * This contact needs filtering because a fixture filter was changed.
		 */
		static b2internal var e_filterFlag:uint = 0x0008;

		/**
		 * This bullet contact had a TOI event.
		 */
		static b2internal var e_bulletHitFlag:uint = 0x0010;

		/**
		 * This contact has a valid TOI in m_toi
		 */
		static b2internal var e_toiFlag:uint = 0x0020;

		/**
		 */
		static b2internal var s_initialized:Boolean = false;

		/**
		 */
		static b2internal var s_registers:Vector.<Vector.<b2ContactRegister>>;


		/**
		 *
		 * @param p_createFcn
		 * @param p_destroyFcn
		 * @param p_typeA
		 * @param p_typeB
		 */
		static b2internal function AddType(p_createFcn:Function, p_destroyFcn:Function, p_typeA:int, p_typeB:int):void
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_typeA && p_typeA < b2Shape.TYPE_COUNT, "type A out of range");
				b2Assert(0 <= p_typeB && p_typeB < b2Shape.TYPE_COUNT, "type B out of range");
			}

			s_registers[p_typeA][p_typeB].createFcn = p_createFcn;
			s_registers[p_typeA][p_typeB].destroyFcn = p_destroyFcn;
			s_registers[p_typeA][p_typeB].primary = true;

			if (p_typeA != p_typeB)
			{
				s_registers[p_typeB][p_typeA].createFcn = p_createFcn;
				s_registers[p_typeB][p_typeA].destroyFcn = p_destroyFcn;
				s_registers[p_typeB][p_typeA].primary = false;
			}
		}

		/**
		 */
		static b2internal function InitializeRegisters():void
		{
			s_registers = new <Vector.<b2ContactRegister>>[];
			var ar:Vector.<b2ContactRegister>;

			for (var i:int = 0; i < b2Shape.TYPE_COUNT; i++)
			{
				ar = new <b2ContactRegister>[];
				s_registers[i] = ar;

				for (var j:int = 0; j < b2Shape.TYPE_COUNT; j++)
				{
					ar[j] = new b2ContactRegister();
				}
			}

			//
			AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.CIRCLE, b2Shape.CIRCLE);
			AddType(b2PolygonAndCircleContact.Create, b2PolygonAndCircleContact.Destroy, b2Shape.POLYGON, b2Shape.CIRCLE);
			AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.POLYGON, b2Shape.POLYGON);
			AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.EDGE, b2Shape.CIRCLE);
			AddType(b2EdgeAndPolygonContact.Create, b2EdgeAndPolygonContact.Destroy, b2Shape.EDGE, b2Shape.POLYGON);
			AddType(b2ChainAndCircleContact.Create, b2ChainAndCircleContact.Destroy, b2Shape.CHAIN, b2Shape.CIRCLE);
			AddType(b2ChainAndPolygonContact.Create, b2ChainAndPolygonContact.Destroy, b2Shape.CHAIN, b2Shape.POLYGON);
		}

		/**
		 *
		 * @param p_fixtureA
		 * @param p_indexA
		 * @param p_fixtureB
		 * @param p_indexB
		 * @return
		 */
		static b2internal function Create(p_fixtureA:b2Fixture, p_indexA:int, p_fixtureB:b2Fixture, p_indexB:int):b2Contact
		{
			if (s_initialized == false)
			{
				InitializeRegisters();
				s_initialized = true;
			}

			var typeA:int = p_fixtureA.GetType();
			var typeB:int = p_fixtureB.GetType();

			CONFIG::debug
			{
				b2Assert(0 <= typeA && typeA < b2Shape.TYPE_COUNT, "type A out of range");
				b2Assert(0 <= typeB && typeB < b2Shape.TYPE_COUNT, "type B out of range");
			}

			var createFcn:Function = s_registers[typeA][typeB].createFcn;

			if (createFcn)
			{
				if (s_registers[typeA][typeB].primary)
				{
					return createFcn(p_fixtureA, p_indexA, p_fixtureB, p_indexB);
				}
				else
				{
					return createFcn(p_fixtureB, p_indexB, p_fixtureA, p_indexA);
				}
			}

			return null;
		}

		/**
		 * @param p_contact
		 */
		static b2internal function Destroy(p_contact:b2Contact):void
		{
			CONFIG::debug
			{
				b2Assert(s_initialized == true, "s_initialized is false");
			}

			var fixtureA:b2Fixture = p_contact.m_fixtureA;
			var fixtureB:b2Fixture = p_contact.m_fixtureB;

			if (p_contact.m_manifold.pointCount > 0 && !fixtureA.IsSensor() && !fixtureB.IsSensor())
			{
				fixtureA.GetBody().SetAwake(true);
				fixtureB.GetBody().SetAwake(true);
			}

			var typeA:int = fixtureA.GetType();
			var typeB:int = fixtureB.GetType();

			CONFIG::debug
			{
				b2Assert(0 <= typeA && typeB < b2Shape.TYPE_COUNT, "wrong type value");
				b2Assert(0 <= typeA && typeB < b2Shape.TYPE_COUNT, "wrong type value");
			}

			var destroyFn:Function = s_registers[typeA][typeB].destroyFcn;
			destroyFn(p_contact);
		}

		/**
		 * Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
		 * For example, anything slides on ice.
 		 * @return
		 */
		[Inline]
		static public function b2MixFriction(p_friction1:Number, p_friction2:Number):Number
		{
			return Math.sqrt(p_friction1 * p_friction2);
		}

		/**
		 * Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
		 * For example, a superball bounces on anything.
 		 */
		[Inline]
		static public function b2MixRestitution(p_restitution1:Number, p_restitution2:Number):Number
		{
			return p_restitution1 > p_restitution2 ? p_restitution1 : p_restitution2;
		}


		////

		protected var m_flags:uint;

		// World pool and list pointers.
		protected var m_prev:b2Contact;
		protected var m_next:b2Contact;

		// Nodes for connecting bodies.
		protected var m_nodeA:b2ContactEdge;
		protected var m_nodeB:b2ContactEdge;

		protected var m_fixtureA:b2Fixture;
		protected var m_fixtureB:b2Fixture;

		protected var m_indexA:int;
		protected var m_indexB:int;

		protected var m_manifold:b2Manifold;
		protected var m_oldManifold:b2Manifold;

		protected var m_toiCount:int;
		protected var m_toi:Number;

		protected var m_friction:Number;
		protected var m_restitution:Number;

		protected var m_tangentSpeed:Number;

		/**
		 *
		 * @param p_fixtureA
		 * @param p_indexA
		 * @param p_fixtureB
		 * @param p_indexB
		 */
		public function b2Contact(p_fixtureA:b2Fixture, p_indexA:int, p_fixtureB:b2Fixture, p_indexB:int)
		{
			m_flags = e_enabledFlag;
			m_fixtureA = p_fixtureA;
			m_fixtureB = p_fixtureB;
			m_indexA = p_indexA;
			m_indexB = p_indexB;
			m_manifold = new b2Manifold();
			m_oldManifold = new b2Manifold();
			m_nodeA = new b2ContactEdge();
			m_nodeB = new b2ContactEdge();
			m_toiCount = 0;
			m_friction = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
			m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
			m_tangentSpeed = 0;
		}

		// public //

		/**
		 * Get the contact manifold. Do not modify the manifold unless you understand the internals of Box2D.
		 */
		public function GetManifold():b2Manifold
		{
			return m_manifold;
		}

		/**
		 * Get the world manifold.
		 */
		public function GetWorldManifold(p_worldManifold:b2WorldManifold):void
		{
			var bodyA:b2Body = m_fixtureA.GetBody();
			var bodyB:b2Body = m_fixtureB.GetBody();
			var shapeA:b2Shape = m_fixtureA.GetShape();
			var shapeB:b2Shape = m_fixtureB.GetShape();

			p_worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);

		}

		/**
		 * Is this contact touching?
		 */
		[Inline]
		final public function IsTouching():Boolean
		{
			return (m_flags & e_touchingFlag) == e_touchingFlag;
		}

		/**
		 * Enable/disable this contact.
		 * This can be used inside the pre-solve contact listener.
		 * The contact is only disabled for the current time step (or sub-step in continuous collisions).
		 */
		public function SetEnabled(p_flag:Boolean):void
		{
			if (p_flag)
			{
				m_flags = m_flags | e_enabledFlag;
			}
			else
			{
				m_flags = m_flags & ~e_enabledFlag;
			}
		}

		/**
		 * Has this contact been disabled?
		 * @return
		 */
		[Inline]
		final public function IsEnabled():Boolean
		{
			return (m_flags & e_enabledFlag) == e_enabledFlag;
		}

		/**
		 * Get the next contact in the world's contact list.
		 */
		public function GetNext():b2Contact
		{
			return m_next;
		}

		/**
		 * Get the child primitive index for fixture A.
		 * @return
		 */
		public function GetChildIndexA():int
		{
			return m_indexA;
		}

		/**
		 * Get the child primitive index for fixture B.
		 */
		public function GetChildIndexB():int
		{
			return m_indexB;
		}

		/**
		 * Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
		 * This value persists until set or reset.
		 */
		public function SetFriction(p_friction:Number):void
		{
			m_friction = p_friction;
		}

		/**
		 * Get the friction.
		 */
		public function GetFriction():Number
		{
			return m_friction;
		}

		/**
		 * Reset the friction mixture to the default value.
		 */
		public function ResetFriction():void
		{
			m_friction = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		}

		/**
		 * Override the default restitution mixture.
		 * You can call this in b2ContactListener::PreSolve.
		 * The value persists until you set or reset.
		 * @param p_restitution
		 */
		public function SetRestitution(p_restitution:Number):void
		{
			m_restitution = p_restitution;
		}

		/**
		 * Get the restitution.
		 */
		public function GetRestitution():Number
		{
			return m_restitution;
		}

		/**
		 * Reset the restitution to the default value.
		 */
		public function ResetRestitution():void
		{
			m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
		}

		/**
		 * Set the desired tangent speed for a conveyor belt behavior. In meters per second.
		 */
		public function SetTangentSpeed(p_speed:Number):void
		{
			m_tangentSpeed = p_speed;
		}

		/**
		 * Get the desired tangent speed. In meters per second.
		 */
		public function GetTangentSpeed():Number
		{
			return m_tangentSpeed;
		}

		/**
		 * Evaluate this contact with your own manifold and transforms.
		 * @param p_manifold
		 * @param p_xfA
		 * @param p_xfB
		 */
		public function Evaluate(p_manifold:b2Manifold, p_xfA:b2Mat22, p_xfB:b2Mat22):void
		{
			// abstract method has to be
		}

		/**
		 * Get fixture A in this contact.
		 * @return
		 */
		[Inline]
		final public function GetFixtureA():b2Fixture
		{
			return m_fixtureA;
		}

		/**
		 * Get fixture B in this contact.
		 * @return
		 */
		[Inline]
		final public function GetFixtureB():b2Fixture
		{
			return m_fixtureB;
		}

		/**
		 * Flag this contact for filtering. Filtering will occur the next time step.
		 */
		public function FlagForFiltering():void
		{
			m_flags |= e_filterFlag;
		}

		/**
		* Update the contact manifold and touching status.
		* Note: do not assume the fixture AABBs are overlapping or are valid.
		*/
		protected function Update(p_listener:b2ContactListener):void
		{
			// Swaps places between old and current manifold state
			var tMp:b2Manifold = m_oldManifold;
			m_oldManifold = m_manifold;
			m_manifold = tMp;

			// Re-enable this contact.
			m_flags = m_flags | e_enabledFlag;

			var touching:Boolean = false;
			var wasTouching:Boolean = (m_flags & e_touchingFlag) == e_touchingFlag;

			var sensorA:Boolean = m_fixtureA.IsSensor();
			var sensorB:Boolean = m_fixtureB.IsSensor();
			var sensor:Boolean = sensorA || sensorB;

			var bodyA:b2Body = m_fixtureA.GetBody();
			var bodyB:b2Body = m_fixtureB.GetBody();

			var xfA:b2Mat22 = bodyA.GetTransform();
			var xfB:b2Mat22 = bodyB.GetTransform();

			// Is this contact a sensor?
			if (sensor)
			{
				var shapeA:b2Shape = m_fixtureA.GetShape();
				var shapeB:b2Shape = m_fixtureB.GetShape();

				touching = b2Collision.b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

				// Sensors don't generate manifolds.
				m_manifold.pointCount = 0;
			}
			else
			{
				Evaluate(m_manifold, xfA, xfB);

				touching = m_manifold.pointCount > 0;

				var pointCount:int = m_manifold.pointCount;
				var mp2:Vector.<b2ManifoldPoint> = m_manifold.points;
				var mp1:Vector.<b2ManifoldPoint> = m_oldManifold.points;
				var id2:b2ContactID;
				var manifoldPoint:b2ManifoldPoint;
				var oldManifoldPoint:b2ManifoldPoint;
				var oldCount:int = m_oldManifold.pointCount;

				// Match old contact ids to new contact ids and copy the
				// stored impulses to warm start the solver.
				for (var i:int = 0; i < pointCount; i++)
				{
					manifoldPoint = mp2[i];
					manifoldPoint.normalImpulse = 0.0;
					manifoldPoint.tangentImpulse = 0.0;
					id2 = manifoldPoint.id;

					for (var j:int = 0; j < oldCount; j++)
					{
						oldManifoldPoint = mp1[j];

						if (oldManifoldPoint.id.key == id2.key)
						{
							manifoldPoint.normalImpulse = oldManifoldPoint.normalImpulse;
							manifoldPoint.tangentImpulse = oldManifoldPoint.tangentImpulse;
							break;
						}
					}
				}

				if (touching != wasTouching)
				{
					bodyA.SetAwake(true);
					bodyB.SetAwake(true);
				}
			}

			//
			if (touching)
			{
				m_flags = m_flags | e_touchingFlag;
			}
			else
			{
				m_flags = m_flags & ~e_touchingFlag;
			}

			//
			if (p_listener)
			{
				if (touching)
				{
					if (!wasTouching)
					{
						p_listener.BeginContact(this);
					}

					if (!sensor)
					{
						p_listener.PreSolve(this, m_oldManifold);
					}
				}
				else if (wasTouching)
				{
					p_listener.EndContact(this);
				}
			}
		}
	}
}
