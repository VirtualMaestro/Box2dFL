/**
 * User: VirtualMaestro
 * Date: 13.01.2015
 * Time: 16:17
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Sweep;

	/**
	 * Input/Output parameters for b2TimeOfImpact
	 * Merged b2TOIInput, b2TOIOutput
	 */
	public class b2TOIData
	{
		// Input params
		public var proxyA:b2DistanceProxy;
		public var proxyB:b2DistanceProxy;
		public var sweepA:b2Sweep;
		public var sweepB:b2Sweep;
		public var tMax:Number;		// defines sweep interval [0, tMax]

		// Output params
		static public const e_unknown:int = 0;
		static public const e_failed:int = 1;
		static public const e_overlapped:int = 2;
		static public const e_touching:int = 3;
		static public const e_separated:int = 4;

		public var state:int;
		public var t:Number;

		/**
		 */
		public function b2TOIData()
		{
			proxyA = new b2DistanceProxy();
			proxyB = new b2DistanceProxy();

			sweepA = b2Sweep.Get();
			sweepB = b2Sweep.Get();
		}
	}
}
