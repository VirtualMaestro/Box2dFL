/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 16:35
 */
package Box2D.Collision.Shapes
{
	/**
	* A chain shape is a free form sequence of line segments.
	* The chain has two-sided collision, so you can use inside and outside collision.
	* Therefore, you may use any winding order.
	* Since there may be many vertices, they are allocated using b2Alloc.
	* Connectivity information is used to create smooth collisions.
	* WARNING: The chain will not collide properly if there are self-intersections.
	 *
	 * TODO:
	*/
	public class b2ChainShape extends b2Shape
	{
		public function b2ChainShape()
		{
			super();
		}

		/**
		 * Get a child edge.
		 * TODO:
 		 */
		public function GetChildEdge(p_edge:b2EdgeShape, p_index:int):void
		{

		}
	}
}
