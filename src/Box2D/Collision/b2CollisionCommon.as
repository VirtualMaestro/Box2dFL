/**
 * User: VirtualMaestro
 * Date: 21.12.2014
 * Time: 23:33
 */
package Box2D.Collision
{
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.Structures.b2ClipVertex;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Vec2;

	/**
	 *
	 */
	public class b2CollisionCommon
	{
		/**
		 * Clipping for contact manifolds.
		 *
		 * @param p_vOut
		 * @param p_vIn
		 * @param p_normalX
		 * @param p_normalY
		 * @param p_offset
		 * @param p_vertexIndexA
		 * @return
		 * TODO:
		 */
		static public function b2ClipSegmentToLine(p_vOut:Vector.<b2ClipVertex>/*2 elem*/, p_vIn:Vector.<b2ClipVertex> /*2 elem*/,
		                                           p_normalX:Number, p_normalY:Number, p_offset:Number, p_vertexIndexA:int):int
		{
			return 0;
		}

		/**
		 * Determine if two generic shapes overlap.
		 *
		 * @param p_shapeA
		 * @param p_indexA
		 * @param p_shapeB
		 * @param p_indexB
		 * @param p_xfA
		 * @param p_xfB
		 * @return
		 * TODO:
		 */
		static public function b2TestOverlap(p_shapeA:b2Shape, p_indexA:int,
		                                     p_shapeB:b2Shape, p_indexB:int,
											 p_xfA:b2Mat22, p_xfB:b2Mat22):Boolean
		{

		}

		/**
		 *
		 * @param p_a
		 * @param p_b
		 * @return
		 */
		static public function b2TestOverlapAABB(p_a:b2AABB, p_b:b2AABB):Boolean
		{
			var d1X:Number;
			var d1Y:Number;
			var d2X:Number;
			var d2Y:Number;

			d1X = p_b.lowerBoundX - p_a.upperBoundX;
			d1Y = p_b.lowerBoundY - p_a.upperBoundY;

			d2X = p_a.lowerBoundX - p_b.upperBoundX;
			d2Y = p_a.lowerBoundY - p_b.upperBoundY;

			if (d1X > 0.0 || d1Y > 0.0)
				return false;

			return !(d2X > 0.0 || d2Y > 0.0);
		}
	}
}
