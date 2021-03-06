/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 9:28
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Shapes.b2ChainShape;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.b2Collision;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	/**
	 *
	 */
	public class b2ChainAndCircleContact extends b2Contact
	{
		/**
		 * @param p_fixtureA
		 * @param p_indexA
		 * @param p_fixtureB
		 * @return
		 * @param p_indexB
		 */
		static public function Create(p_fixtureA:b2Fixture, p_indexA:int, p_fixtureB:b2Fixture, p_indexB:int):b2ChainAndCircleContact
		{
			return new b2ChainAndCircleContact(p_fixtureA, p_indexA, p_fixtureB, p_indexB);  // TODO: Think about pool
		}

		/**
		 * @param p_contact
		 */
		static public function Destroy(p_contact:b2Contact):void
		{
			// TODO: Maybe need invocation of Dispose method of p_contact. In that case need to extends b2Contact from b2Disposable
		}

		/**
		 * @param p_fixtureA
		 * @param p_indexA
		 * @param p_fixtureB
		 * @param p_indexB
		 */
		public function b2ChainAndCircleContact(p_fixtureA:b2Fixture, p_indexA:int, p_fixtureB:b2Fixture, p_indexB:int)
		{
			super(p_fixtureA, p_indexA, p_fixtureB, p_indexB);

			CONFIG::debug
			{
				b2Assert(m_fixtureA.GetType() == b2Shape.CHAIN, "discrepancy fixtureA type. Has to be CHAIN");
				b2Assert(m_fixtureB.GetType() == b2Shape.CIRCLE, "discrepancy fixtureB type. Has to be CIRCLE");
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
			var chain:b2ChainShape = m_fixtureA.GetShape() as b2ChainShape;
			var edge:b2EdgeShape = new b2EdgeShape();
			chain.GetChildEdge(edge, m_indexA);

			b2Collision.b2CollideEdgeAndCircle(p_manifold, edge, p_xfA, m_fixtureB.GetShape() as b2CircleShape, p_xfB)
		}
	}
}
