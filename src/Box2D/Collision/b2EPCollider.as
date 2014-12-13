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
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Vec2;

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
		 * TODO:
		 * @return
		 */
		public function ComputeEdgeSeparation():b2EPAxis
		{

		}

		/**
		 * TODO:
		 * @return
		 */
		public function ComputePolygonSeparation():b2EPAxis
		{

		}

	}
}
