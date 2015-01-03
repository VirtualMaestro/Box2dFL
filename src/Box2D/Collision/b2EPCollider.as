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
	import Box2D.Common.b2Settings;
	import Box2D.b2Assert;

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
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * @return Box2D.Collision.Structures.b2EPAxis
		 */
		public function ComputeEdgeSeparation():b2EPAxis
		{
			var axis:b2EPAxis = new b2EPAxis();
			axis.type = b2EPAxis.e_edgeA;
			axis.index = m_front ? 0 : 1;

			var bestSeparation:Number = Number.MAX_VALUE;
			var vertices:Vector.<Number> = m_polygonB.vertices;
			var v1X:Number = m_v1X;
			var v1Y:Number = m_v1Y;
			var normalX:Number = m_normalX;
			var normalY:Number = m_normalY;
			var vX:Number;
			var vY:Number;
			var s:Number;
			var count:int = m_polygonB.count;

			for (var i:int = 0; i < count; i++)
			{
				vX = b2Math.getX(vertices, i) - v1X;
				vY = b2Math.getY(vertices, i) - v1Y;

				s = normalX * vX + normalY * vY;

				if (s < bestSeparation)
				{
					bestSeparation = s;
				}
			}

			axis.separation = bestSeparation;

			return axis;
		}

		/**
		 * TODO: Optimize creation b2EPAxis
		 */
		public function ComputePolygonSeparation():b2EPAxis
		{   var axis:b2EPAxis = new b2EPAxis();
			axis.type = b2EPAxis.e_unknown;
			axis.index = -1;
			axis.separation = -Number.MAX_VALUE;

			var perpX:Number = -m_normalY;
			var perpY:Number = -m_normalX;

			var count:int = m_polygonB.count;
			var nX:Number;
			var nY:Number;
			var vX:Number;
			var vY:Number;
			var normals:Vector.<Number>;
			var vertices:Vector.<Number>;

			for (var i:int = 0; i < count; i++)
			{
				normals = m_polygonB.normals;
				nX = -b2Math.getX(normals, i);
				nY = -b2Math.getY(normals, i);

				vertices = m_polygonB.vertices;
				vX = b2Math.getX(vertices, i);
				vY = b2Math.getY(vertices, i);

				var s1:Number = nX * (vX-m_v1X) + nY * (vY-m_v1Y);
				var s2:Number = nX * (vX-m_v2X) + nY * (vY-m_v2Y);
				var s:Number = b2Math.Min(s1, s2);

				if (s > m_radius)
				{
					// No collision
					axis.type = b2EPAxis.e_edgeB;
					axis.index = i;
					axis.separation = s;
					return axis;
				}

				// Adjacency
				if ((nX * perpX + nY * perpY) >= 0.0)
				{
					vX = nX - m_upperLimitX;
					vY = nY - m_upperLimitY;
				}
				else
				{
					vX = nX - m_lowerLimitX;
					vY = nY - m_lowerLimitY;
				}

				if ((vX * m_normalX + vY * m_normalY) < -b2Settings.angularSlop)
				{
					continue;
				}

				if (s > axis.separation)
				{
					axis.type = b2EPAxis.e_edgeB;
					axis.index = i;
					axis.separation = s;
				}
			}

			return axis;
		}
	}
}
