/**
 * User: VirtualMaestro
 * Date: 22.12.2014
 * Time: 22:27
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Mat22;

	/**
	 * Merge structures: b2DistanceInput, b2DistanceOutput
	 */
	public class b2DistanceData
	{
		/** input value*/
		public var proxyA:b2DistanceProxy;

		/** input value*/
		public var proxyB:b2DistanceProxy;

		/** input value*/
		public var transformA:b2Mat22;

		/** input value*/
		public var transformB:b2Mat22;

		/** input value*/
		public var useRadii:Boolean;

		/**
		 * Closest point on shapeA
		 * output value
		 * */
		public var pointAX:Number;
		/**
		 * closest point on shapeA
		 * output value
		 * */
		public var pointAY:Number;

		/**
		 * closest point on shapeB
		 * output value
		 * */
		public var pointBX:Number;
		/**
		 * closest point on shapeB
		 * output value
		 * */
		public var pointBY:Number;

		/**output value*/
		public var distance:Number;

		/**
		 * number of GJK iterations used
		 * output value
		 * */
		public var iterations:int;

		/**
		 */
		public function b2DistanceData()
		{
			proxyA = new b2DistanceProxy();
			proxyB = new b2DistanceProxy();
		}
	}
}
