/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 14:32
 */
package Box2D.Common.Math
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Lightweight class represents of point.
	 */
	public class b2SPoint extends b2Disposable
	{
		public var x:Number;
		public var y:Number;

		/**
		 */
		public function b2SPoint(p_x:Number = 0, p_y:Number = 0)
		{
			x = p_x;
			y = p_y;
		}

		/**
		 * Dispose instance. After disposing there is no possible of using instance.
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
		 * Makes a clone of current instance.
		 * @return instance of b2SPoint
		 */
		override public function Clone():IDisposable
		{
			return Get(x, y);
		}


		//*************
		//**** POOL ***
		//*************
		static private var _pool:Vector.<b2SPoint> = new <b2SPoint>[];
		static private var _count:int = 0;

		/**
		 * Returns new instance of b2SPoint.
		 * @return b2SPoint
		 */
		static public function Get(p_x:Number = 0, p_y:Number = 0):b2SPoint
		{
			var instance:b2SPoint;

			if (_count > 0)
			{
				instance = _pool[--_count];
				instance.disposed = false;
				_pool[_count] = null;

				instance.x = p_x;
				instance.y = p_y;
			}
			else
			{
				instance = new b2SPoint(p_x, p_y);
			}

			return instance;
		}

		/**
		 * Put instance of b2SPoint to pool.
		 */
		[Inline]
		static private function Put(p_instance:b2SPoint):void
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
