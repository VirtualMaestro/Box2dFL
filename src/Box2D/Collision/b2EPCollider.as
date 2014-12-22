/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:47
 */
package Box2D.Collision
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Structures.b2EPAxis;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;

	/**
	 * This class collides and edge and a polygon, taking into account edge adjacency.
	 * TODO:
	 */
	public class b2EPCollider
	{
		public var m_polygonB:b2TempPolygon;

		public var m_xf:b2Mat22;

		public var m_centroidBX:Number;
		public var m_centroidBY:Number;

		public var m_v0X:Number;
		public var m_v0Y:Number;
		public var m_v1X:Number;
		public var m_v1Y:Number;
		public var m_v2X:Number;
		public var m_v2Y:Number;
		public var m_v3X:Number;
		public var m_v3Y:Number;

		public var m_normalX:Number;
		public var m_normalY:Number;
		public var m_normal0X:Number;
		public var m_normal0Y:Number;
		public var m_normal1X:Number;
		public var m_normal1Y:Number;
		public var m_normal2X:Number;
		public var m_normal2Y:Number;

//		VertexType m_type1, m_type2;

		public var m_lowerLimitX:Number;
		public var m_lowerLimitY:Number;
		public var m_upperLimitX:Number;
		public var m_upperLimitY:Number;

		public var m_radius:Number;
		public var m_front:Boolean;

		/**
		 */
		public function b2EPCollider()
		{
			m_polygonB = new b2TempPolygon();
			m_xf = b2Mat22.Get();
		}

		/**
		* Algorithm:
		* 1. Classify v1 and v2
		* 2. Classify polygon centroid as front or back
		* 3. Flip normal if necessary
		* 4. Initialize normal range to [-pi, pi] about face normal
		* 5. Adjust normal range according to adjacent edges
		* 6. Visit each separating axes, only accept axes within the range
		* 7. Return if _any_ axis indicates separation
		* 8. Clip
		*
		*
		 *
		 * @param p_manifold
		 * @param p_edgeA
		 * @param p_xfA
		 * @param p_polygonB
		 * @param p_xfB
		 *
		 * TODO:
		 */
		public function Collide(p_manifold:b2Manifold, p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
					            p_polygonB:b2PolygonShape, p_xfB:b2Mat22):void
		{

		}

		/**
		 * @return Box2D.Collision.Structures.b2EPAxis
		 */
		public function ComputeEdgeSeparation():b2EPAxis
		{
			var axis:b2EPAxis = new b2EPAxis();
			axis.type = b2EPAxis.e_edgeA;
			axis.index = m_front ? 0 : 1;
			axis.separation = Number.MAX_VALUE;

			var vertices:Vector.<Number>;
			var vX:Number;
			var vY:Number;
			var s:Number;
			var count:int = m_polygonB.count;
			for (var i:int = 0; i < count; i++)
			{
				vertices = m_polygonB.vertices;
				vX = b2Math.getX(vertices, i) - m_v1X;
				vY = b2Math.getY(vertices, i) - m_v1Y;

				s = m_normalX * vX + m_normalY * vY;

				if (s < axis.separation)
				{
					axis.separation = s;
				}
			}

			return axis;
		}

		/**
		 * TODO:
		 * @return
		 */
		public function ComputePolygonSeparation():b2EPAxis
		{
			return null;
		}
	}
}
