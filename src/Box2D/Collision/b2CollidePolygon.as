/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 12:25
 */
package Box2D.Collision
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Common.Math.b2Mat22;

	/**
	 *
	 */
	public class b2CollidePolygon
	{

		/**
		 * Find edge normal of max separation on A - return if separating axis is found
		 * Find edge normal of max separation on B - return if separation axis is found
		 * Choose reference edge as min(minA, minB)
		 * Find incident edge
		 * Clip
		 * The normal points from 1 to 2
		 * // TODO:
 		 */
		static public function b2CollidePolygons(p_manifold:b2Manifold,
												  p_polyA:b2PolygonShape, p_xfA:b2Mat22,
												  p_polyB:b2PolygonShape, p_xfB:b2Mat22):void
		{

		}

		/**
		 *
		 * @param p_c
		 * @param p_poly1
		 * @param p_xf1
		 * @param p_edge1
		 * @param p_poly2
		 * @param p_xf2
		 * TODO:
		 */
		static public function b2FindIncidentEdge(p_c:Vector.<b2ClipVertex> /*2 elements*/,
													 p_poly1:b2PolygonShape, p_xf1:b2Mat22, p_edge1:int,
													 p_poly2:b2PolygonShape, p_xf2:b2Mat22):void
		{

		}

		/**
		 * Find the max separation between poly1 and poly2 using edge normals from poly1.
		 * @param p_edgeIndex
		 * @param p_poly1
		 * @param p_xf1
		 * @param p_poly2
		 * @param p_xf2
		 * @return
		 * TODO:
		 */
        static public function b2FindMaxSeparation(p_edgeIndex:int,
													 p_poly1:b2PolygonShape, p_xf1:b2Mat22,
													 p_poly2:b2PolygonShape, p_xf2:b2Mat22):Number
        {
	        return 0;
        }
	}
}
