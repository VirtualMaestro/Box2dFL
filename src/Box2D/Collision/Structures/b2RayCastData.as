/**
 * User: VirtualMaestro
 * Date: 01.12.2014
 * Time: 21:39
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 */
	public class b2RayCastData extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		/**
		 */
		public var p1X:Number;

		/**
		 */
		public var p1Y:Number;

		/**
		 */
		public var p2X:Number;

		/**
		 */
		public var p2Y:Number;

		/**
		 * Ray-cast input data.
		 */
		public var maxFraction:Number = 1.0;

		/**
		 *   .
		 * Ray-cast output data
		 */
		public var normalX:Number;

		/**
		 *
		 * Ray-cast output data
		 */
		public var normalY:Number;

		/**
		 * Fraction.
		 * Ray-cast output data.
		 */
		public var fraction:Number;

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			b2Disposable.Put(this, classId);
		}

		/**
		 */
		override public function Clone():IDisposable
		{
			var rayCastData:b2RayCastData = Get(p1X, p1Y, p2X, p2Y, maxFraction);
			rayCastData.normalX = normalX;
			rayCastData.normalY = normalY;
			rayCastData.fraction = fraction;

			return rayCastData;
		}

		/**
		 */
		public function b2RayCastData(p_startX:Number = 0, p_startY:Number = 0, p_endX:Number = 0, p_endY:Number = 0, p_maxFraction:Number = 1.0)
		{
			p1X = p_startX;
			p1Y = p_startY;
			p2X = p_endX;
			p2Y = p_endY;
			maxFraction = p_maxFraction;
		}

		/**
		 * Returns new instance of b2RayCastData.
		 * @return Box2D.Collision.Structures.b2RayCastData
		 */
		[Inline]
		static public function Get(p_startX:Number = 0, p_startY:Number = 0, p_endX:Number = 0, p_endY:Number = 0, p_maxFraction:Number = 1.0):b2RayCastData
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var rayCastData:b2RayCastData;

			if (instance)
			{
				rayCastData = instance as b2RayCastData;
				rayCastData.p1X = p_startX;
				rayCastData.p1Y = p_startY;
				rayCastData.p2X = p_endX;
				rayCastData.p2Y = p_endY;
				rayCastData.maxFraction = p_maxFraction;
			}
			else rayCastData = new b2RayCastData(p_startX, p_startY, p_endX, p_endY, p_maxFraction);

			return rayCastData;
		}
	}
}
