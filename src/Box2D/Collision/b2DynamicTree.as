/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	/**
	 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
	 * A dynamic tree arranges data in a binary tree to accelerate
	 * queries such as volume queries and ray casts. Leafs are proxies
	 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
	 * so that the proxy AABB is bigger than the client object. This allows the client
	 * object to move by small amounts without triggering a tree update.
	 *
	 * Nodes are pooled and relocatable, so we use node indices rather than pointers.
	 *
	 * TODO: Impl
	 */
	public class b2DynamicTree
	{
		public function b2DynamicTree()
		{
		}
	}
}
