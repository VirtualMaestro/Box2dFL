/**
 * User: VirtualMaestro
 * Date: 12.12.2014
 * Time: 22:23
 */
package Box2D.Dynamics.Joints
{
	import Box2D.Common.b2internal;

	/**
	 * The base joint class. Joints are used to constraint two bodies together in
	 * various fashions. Some joints also feature limits and motors.
	 * TODO:
	 */
	public class b2Joint
	{
		// Joint types
		static public const e_unknownJoint:int = 0;
		static public const e_revoluteJoint:int = 1;
		static public const e_prismaticJoint:int = 2;
		static public const e_distanceJoint:int = 3;
		static public const e_pulleyJoint:int = 4;
		static public const e_mouseJoint:int = 5;
		static public const e_gearJoint:int = 6;
		static public const e_wheelJoint:int = 7;
		static public const  e_weldJoint:int = 8;
		static public const e_frictionJoint:int = 9;
		static public const 	e_ropeJoint:int = 10;
		static public const e_motorJoint:int = 11;

		//
		b2internal var m_collideConnected:Boolean;

		/**
		 */
		public function b2Joint()
		{
		}
	}
}
