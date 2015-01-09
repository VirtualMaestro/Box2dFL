/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 15:38
 */
package Box2D.Dynamics.Def
{
	import Box2D.Dynamics.Joints.b2Joint;
	import Box2D.Dynamics.b2Body;

	/**
	 * Joint definitions are used to construct joints.
	 */
	public class b2JointDef
	{
		/**
		 * The joint type is set automatically for concrete joint types.
		 */
		public var type:int = b2Joint.e_unknownJoint;

		/**
		 * Use this to attach application specific data to your joints.
 		 */
		public var userData:*;

		/**
		 * The first attached body.
		 */
		public var bodyA:b2Body;

		/**
		 * The second attached body.
		 */
		public var bodyB:b2Body;

		/**
		 * Set this flag to true if the attached bodies should collide.
		 */
		public var collideConnected:Boolean = false;
	}
}
