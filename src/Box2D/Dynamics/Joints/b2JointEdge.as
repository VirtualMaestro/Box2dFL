/**
 * User: VirtualMaestro
 * Date: 12.12.2014
 * Time: 22:10
 */
package Box2D.Dynamics.Joints
{
	import Box2D.Dynamics.b2Body;

	/**
	* A joint edge is used to connect bodies and joints together
	* in a joint graph where each body is a node and each joint
	* is an edge. A joint edge belongs to a doubly linked list
	* maintained in each attached body. Each joint has two joint
	* nodes, one for each attached body.
	*/
	public class b2JointEdge
	{
		/**
		 * provides quick access to the other body attached
		 */
		public var other:b2Body;
		public var joint:b2Joint;	///< the joint

		/**
		 * the previous joint edge in the body's joint list
		 */
		public var prev:b2JointEdge;

		/**
		 * the next joint edge in the body's joint list
		 */
		public var next:b2JointEdge;
	}
}
