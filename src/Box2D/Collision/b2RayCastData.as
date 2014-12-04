/**
 * User: VirtualMaestro
 * Date: 01.12.2014
 * Time: 21:39
 */
package Box2D.Collision
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	 */
	public class b2RayCastData extends b2Disposable
	{
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

			Put(this);
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


		//*************
		//**** POOL ***
		//*************
		static private var _pool:Vector.<b2RayCastData> = new <b2RayCastData>[];
		static private var _count:int = 0;

		/**
		 * Returns new instance of b2RayCastData.
		 * @return b2RayCastData
		 */
		static public function Get(p_startX:Number, p_startY:Number, p_endX:Number, p_endY:Number, p_maxFraction:Number):b2RayCastData
		{
			var instance:b2RayCastData;

			if (_count > 0)
			{
				instance = _pool[--_count];
				instance.disposed = false;
				_pool[_count] = null;

				instance.startX = p_startX;
				instance.startY = p_startY;
				instance.endX = p_endX;
				instance.endY = p_endY;
				instance.maxFraction = p_maxFraction;
			}
			else
			{
				instance = new b2RayCastData(p_startX, p_startY, p_endX, p_endY, p_maxFraction);
			}

			return instance;
		}

		/**
		 * Put instance of b2RayCastData to pool.
		 */
		[Inline]
		static private function Put(p_instance:b2RayCastData):void
		{
			p_instance.disposed = true;
			_pool[_count++] = p_instance;
		}

		/**
		 * Clear pool for GC.
		 */
		[Inline]
		static public function Rid():void
		{
			b2Disposable.clearVector(_pool);
			_count = 0;
		}
	}
}
