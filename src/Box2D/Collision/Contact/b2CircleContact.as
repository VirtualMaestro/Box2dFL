/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 9:28
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.b2CollideCircle;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	/**
	 *
	 */
	public class b2CircleContact extends b2Contact
	{
		/**
		 * @param p_fixtureA
		 * @param p_fixtureB
		 * @return
		 */
		static public function Create(p_fixtureA:b2Fixture, p_fixtureB:b2Fixture):b2CircleContact
		{
			return new b2CircleContact(p_fixtureA, p_fixtureB);
		}

		/**
		 * @param p_contact
		 */
		static public function Destroy(p_contact:b2Contact)
		{
			// TODO: Maybe need invocation of Dispose method of p_contact. In that case need to extends b2Contact from b2Disposable
		}

		/**
		 * @param p_fixtureA
		 * @param p_fixtureB
		 */
		public function b2CircleContact(p_fixtureA:b2Fixture, p_fixtureB:b2Fixture)
		{
			super(p_fixtureA, b2Shape.CIRCLE, p_fixtureB, b2Shape.CIRCLE);

			CONFIG::debug
			{
				b2Assert(m_fixtureA.GetType() == b2Shape.CIRCLE, "discrepancy fixture type. Has to be CIRCLE");
				b2Assert(m_fixtureB.GetType() == b2Shape.CIRCLE, "discrepancy fixture type. Has to be CIRCLE");
			}
		}

		/**
		 *
		 * @param p_manifold
		 * @param p_xfA
		 * @param p_xfB
		 */
		override public function Evaluate(p_manifold:b2Manifold, p_xfA:b2Mat22, p_xfB:b2Mat22):void
		{
			b2CollideCircle.b2CollideCircles(p_manifold, m_fixtureA.GetShape() as b2CircleShape, p_xfA, m_fixtureB.GetShape() as b2CircleShape, p_xfB);
		}
	}
}
