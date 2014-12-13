/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:32
 */
package Box2D.Collision
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Common.Math.b2Mat22;

	/**
	 * TODO:
	 */
	public class b2CollideEdge
	{
		public function b2CollideEdge()
		{
		}

		/**
		 * Compute contact points for edge versus circle.
		 * This accounts for edge connectivity.
		 * TODO:
		 */
		static public function b2CollideEdgeAndCircle(p_manifold:b2Manifold,
														p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
														p_circleB:b2CircleShape, p_xfB:b2Mat22):void
		{

		}

		/**
		 *
		 * @param p_manifold
		 * @param p_edgeA
		 * @param p_xfA
		 * @param p_polygonB
		 * @param p_xfB
		 */
		static public function b2CollideEdgeAndPolygon(p_manifold:b2Manifold,
														 p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
														 p_polygonB:b2PolygonShape, p_xfB:b2Mat22):void
		{
			var collider:b2EPCollider = new b2EPCollider();
			collider.Collide(p_manifold, p_edgeA, p_xfA, p_polygonB, p_xfB);
		}

	}
}
