/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 1:22
 */
package Box2D.Dynamics.Def
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Common.Math.b2Vec3;

	/**
	 */
	public class b2ContactSolverDef
	{
		public var step:b2TimeStep;
		public var contacts:Vector.<b2Contact>;
		public var count:int;
		public var positions:Vector.<b2Vec3>;
		public var velocities:Vector.<b2Vec3>;
	}
}
