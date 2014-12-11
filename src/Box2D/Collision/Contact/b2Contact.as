/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Common.b2internal;
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

		static b2internal var s_initialized:Boolean;

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

		/**
		 *
		 * @return
		 */
		[Inline]
		final public function GetFixtureA():b2Fixture
		{
			return m_fixtureA;
		}

		/**
		 *
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
	}
}
