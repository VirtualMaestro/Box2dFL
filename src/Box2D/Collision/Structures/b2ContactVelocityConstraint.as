/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 1:10
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2Settings;

	/**
	 */
	public class b2ContactVelocityConstraint
	{
		public var points:Vector.<b2VelocityConstraintPoint>;
		public var normalX:Number;
		public var normalY:Number;
		public var normalMass:b2Mat22;
		public var K:b2Mat22;

		public var indexA:int;
		public var indexB:int;
		public var invMassA:Number;
		public var invMassB:Number;
		public var invIA:Number;
		public var invIB:Number;
		public var friction:Number;
		public var restitution:Number;
		public var tangentSpeed:Number;
		public var pointCount:int;
		public var contactIndex:int;

		/**
		 */
		public function b2ContactVelocityConstraint()
		{
			for (var i:int = 0; i < b2Settings.maxManifoldPoints; i++)
			{
				points[i] = new b2VelocityConstraintPoint();
			}

			normalMass = b2Mat22.Get();
			K = b2Mat22.Get();
		}
	}
}
