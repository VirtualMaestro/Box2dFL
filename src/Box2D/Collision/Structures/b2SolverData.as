/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 1:35
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Vec3;

	/**
	 * Solver Data
	 */
	public class b2SolverData
	{
		public var step:b2TimeStep;
		public var positions:Vector.<b2Vec3>;
		public var velocities:Vector.<b2Vec3>;

		/**
		 */
		public function b2SolverData()
		{
			step = new b2TimeStep();
		}
	}
}
