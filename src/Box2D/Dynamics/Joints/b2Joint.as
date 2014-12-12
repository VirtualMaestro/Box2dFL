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

		b2internal var m_collideConnected:Boolean;

		/**
		 */
		public function b2Joint()
		{
		}
	}
}
