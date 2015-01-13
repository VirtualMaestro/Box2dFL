/**
 * User: VirtualMaestro
 * Date: 04.01.2015
 * Time: 23:20
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Settings;

	/**
	 * Contact impulses for reporting. Impulses are used instead of forces because
	 * sub-step forces may approach infinity for rigid body collisions. These
	 * match up one-to-one with the contact points in b2Manifold.
	 * TODO: Pool
	 */
	public class b2ContactImpulse
	{
		public var normalImpulses:Vector.<Number>;
		public var tangentImpulses:Vector.<Number>;
		public var count:int;

		/**
		 */
		public function b2ContactImpulse()
		{
			normalImpulses = new Vector.<Number>(b2Settings.maxManifoldPoints);
			tangentImpulses = new Vector.<Number>(b2Settings.maxManifoldPoints);
			count = 0;
		}

		/**
		 */
		public function Clone():b2ContactImpulse
		{
			var cloneImpulse:b2ContactImpulse = new b2ContactImpulse();
			cloneImpulse.count = count;

			b2Math.copyArray(normalImpulses, cloneImpulse.normalImpulses);
			b2Math.copyArray(tangentImpulses, cloneImpulse.tangentImpulses);

			return cloneImpulse;
		}
	}
}
