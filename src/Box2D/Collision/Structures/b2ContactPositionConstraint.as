/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 1:26
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.b2Settings;

	public class b2ContactPositionConstraint
	{
		/**
		 * Every two components is x & y.
		 */
		public var localPoints:Vector.<Number>;
		public var localNormalX:Number;
		public var localNormalY:Number;
		public var localPointX:Number;
		public var localPointY:Number;
		public var indexA:int;
		public var indexB:int;
		public var invMassA:Number;
		public var invMassB:Number;
		public var localCenterAX:Number;
		public var localCenterAY:Number;
		public var localCenterBX:Number;
		public var localCenterBY:Number;
		public var invIA:Number;
		public var invIB:Number;
		public var type:int;
		public var radiusA:Number;
		public var radiusB:Number;
		public var pointCount:int;

		/**
		 */
		public function b2ContactPositionConstraint()
		{
			localPoints = new Vector.<Number>(b2Settings.maxManifoldPoints * 2);
		}
	}
}
