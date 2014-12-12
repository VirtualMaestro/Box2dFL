/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2WorldManifold;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;

	use namespace b2internal;

	/**
	 * The class manages contact between two shapes. A contact exists for each overlapping
	 * AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	 * that has no contact points.
	 *
	 * TODO:
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
		static b2internal var s_initialized:Boolean;

		/**
		 *
		 * @param p_createFcn
		 * @param p_destroyFcn
		 * @param p_typeA
		 * @param p_typeB
		 */
		static public function AddType(p_createFcn/*:b2ContactCreateFcn*/, p_destroyFcn/*:b2ContactDestroyFcn*/, p_typeA:int, p_typeB:int):void
		{
			// TODO:
		}

		static public function InitializeRegisters():void
		{
			// TODO:
		}

		static public function Create(p_fixtureA:b2Fixture, p_indexA:int, p_fixtureB:b2Fixture, p_indexB:int):b2Contact
		{
			// TODO:
		}

		static public function Destroy(p_contact:b2Contact, p_typeA:int, p_typeB:int):void
		{
			// TODO:
		}

		static public function Destroy2(p_contact:b2Contact):void
		{
			// TODO:
		}

		// TODO:
//		static public var s_registers[b2Shape.TYPE_COUNT][b2Shape.TYPE_COUNT] /*b2ContactRegister */;

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

		protected var m_toiCount:int;
		protected var m_toi:Number;

		protected var m_friction:Number;
		protected var m_restitution:Number;

		protected var m_tangentSpeed:Number;


		public function b2Contact()
		{
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
		 * TODO:
		 */
		protected function Update(p_listener/*:b2ContactListener*/):void
		{

		}
	}
}
