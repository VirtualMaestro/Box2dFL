/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 10:23
 */
package Box2D.Collision
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * TODO:
	 */
	public class b2CollideCircle
	{
		/**
		 *
		 * @param p_manifold
		 * @param p_circleA
		 * @param p_xfA
		 * @param p_circleB
		 * @param p_xfB
		 */
		static public function b2CollideCircles(p_manifold:b2Manifold,
		                                        p_circleA:b2CircleShape, p_xfA:b2Mat22,
	  	                                        p_circleB:b2CircleShape, p_xfB:b2Mat22):void
		{
			p_manifold.pointCount = 0;

			var cos:Number = p_xfA.c11;
			var sin:Number = p_xfA.c12;

			var capX:Number = p_circleA.m_pX;
			var capY:Number = p_circleA.m_pY;
			var pAX:Number = (cos * capX - sin * capY) + p_xfA.tx;
			var pAY:Number = (sin * capX + cos * capY) + p_xfA.ty;

			cos = p_xfB.c11;
			sin = p_xfB.c12;

			var cbpX:Number = p_circleB.m_pX;
			var cbpY:Number = p_circleB.m_pY;
			var pBX:Number = (cos * cbpX - sin * cbpY) + p_xfB.tx;
			var pBY:Number = (sin * cbpX + cos * cbpY) + p_xfB.ty;

			var dX:Number = pBX - pAX;
			var dY:Number = pBY - pAY;

			var distSqr:Number = dX * dX + dY * dY;
			var radius:Number = p_circleA.m_radius + p_circleB.m_radius;

			if (distSqr > radius*radius)
			{
				return;
			}

			p_manifold.type = b2Manifold.CIRCLES;
			p_manifold.localPointX = p_circleA.m_pX;
			p_manifold.localPointY = p_circleA.m_pY;
			p_manifold.localNormalX = 0;
			p_manifold.localNormalY = 0;
			p_manifold.pointCount = 1;

			var mPoint:b2ManifoldPoint = p_manifold.points[0];
			mPoint.localPointX = p_circleB.m_pX;
			mPoint.localPointY = p_circleB.m_pY;
			mPoint.id.key = 0;
		}
	}
}
