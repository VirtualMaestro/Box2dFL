/**
 * User: VirtualMaestro
 * Date: 01.12.2014
 * Time: 21:39
 */
package Box2D.Collision
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
		 * Start position of ray. X
		 * Ray-cast input data.
		 */
		public var startX:Number;

		/**
		 * Start position of ray. Y
		 * Ray-cast input data.
		 */
		public var startY:Number;

		/**
		 * End position of ray. X
		 * Ray-cast input data.
		 */
		public var endX:Number;

		/**
		 * End position of ray. Y
		 * Ray-cast input data.
		 */
		public var endY:Number;

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
			var rayCastData:b2RayCastData = Get(startX, startY, endX, endY, maxFraction);
			rayCastData.normalX = normalX;
			rayCastData.normalY = normalY;
			rayCastData.fraction = fraction;

			return rayCastData;
		}

		/**
		 */
		public function b2RayCastData(p_startX:Number, p_startY:Number, p_endX:Number, p_endY:Number, p_maxFraction:Number = 1.0)
		{
			startX = p_startX;
			startY = p_startY;
			endX = p_endX;
			endY = p_endY;
			maxFraction = p_maxFraction;
		}

		/**
		 * Returns new instance of b2RayCastData.
		 * @return b2RayCastData
		 */
		[Inline]
		static public function Get(p_startX:Number, p_startY:Number, p_endX:Number, p_endY:Number, p_maxFraction:Number = 1.0):b2RayCastData
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var rayCastData:b2RayCastData;

			if (instance)
			{
				rayCastData = instance as b2RayCastData;
				rayCastData.startX = p_startX;
				rayCastData.startY = p_startY;
				rayCastData.endX = p_endX;
				rayCastData.endY = p_endY;
				rayCastData.maxFraction = p_maxFraction;
			}
			else rayCastData = new b2RayCastData(p_startX, p_startY, p_endX, p_endY, p_maxFraction);

			return rayCastData;
		}
	}
}
