/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 14:23
 */
package Box2D.Collision.Manifold
{
	import Box2D.Collision.Contact.b2ContactID;

	/**
	* A manifold point is a contact point belonging to a contact
	* manifold. It holds details related to the geometry and dynamics
	* of the contact points.
	 *
	* The local point usage depends on the manifold type:
	* - CIRCLES: the local center of circleB
	* - FACE_A: the local center of circleB or the clip point of polygonB
	* - FACE_B: the clip point of polygonA
	*
	* This structure is stored across time steps, so we keep it small.
	*
	* Note: the impulses are used for internal caching and may not
	* provide reliable contact forces, especially for high speed collisions.
	*/
	public class b2ManifoldPoint
	{
		/**
		 * Usage depends on manifold type
		 */
		public var localPointX:Number;

		/**
		 * Usage depends on manifold type
		 */
		public var localPointY:Number;

		/**
		 * The non-penetration impulse.
		 */
		public var normalImpulse:Number;

		/**
		 * The friction impulse.
		 */
		public var tangentImpulse:Number;

		/**
		 * Uniquely identifies a contact point between two shapes.
		 */
		public var id:b2ContactID;

		/**
		 */
		public function b2ManifoldPoint()
		{
			id = new b2ContactID();
		}
	}
}
