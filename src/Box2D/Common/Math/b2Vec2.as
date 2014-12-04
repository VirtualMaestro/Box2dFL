/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 15:06
 */
package Box2D.Common.Math
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Class represents of 2d vector.
	 */
	public class b2Vec2 extends b2SPoint
	{
		public function b2Vec2(p_x:Number = 0, p_y:Number = 0)
		{
			super(p_x, p_y);
		}

		/**
		 * Copy x/y properties to current instance.
		 */
		public function Set(p_vec2:b2Vec2):void
		{
			x = p_vec2.x;
			y = p_vec2.y;
		}

		/**
		 * Copy x/y values from give object to current instance.
		 * It is possible to set any object which has x and y properties.
		 */
		public function SetObject(p_xy:*):void
		{
			x = p_xy.x;
			y = p_xy.y;
		}

		/**
		 * Returns the length of vector.
		 */
		[Inline]
		final public function get Length():Number
		{
			return Math.sqrt(x*x + y*y);
		}

		/**
		 * Returns the length squared. For performance, use this instead of b2Vec2.Length if possible.
		 */
		[Inline]
		final public function get LengthSquared():Number
		{
			return x*x + y*y;
		}

		/**
		 * Normalization of current vector.
		 * Returns length of vector.
		 * @return Number
		 */
		final public function Normalize():Number
		{
			var length:Number = Length;
			if (length < b2Math.EPSILON)
			{
				return 0.0;
			}
			var invLength:Number = 1.0 / length;
			x *= invLength;
			y *= invLength;

			return length;
		}

		/**
		 * Returns copy of current b2Vec2 instance.
		 */
		final override public function Clone():IDisposable
		{
			return Get(x, y);
		}

		/**
		 * Dispose instance. After disposing there is no possible of using instance.
		 */
		final override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			Put(this);
		}
		
        //*************
		//**** POOL ***
		//*************
		static private var _pool:Vector.<b2Vec2> = new <b2Vec2>[];
		static private var _count:int = 0;
	
		/**
		 * Returns new instance of b2Vec2.
		 * @return b2Vec2
		 */
		static public function Get(p_x:Number = 0, p_y:Number = 0):b2Vec2
		{
			var vec2:b2Vec2;
	
			if (_count > 0)
			{
				vec2 = _pool[--_count];
				vec2.disposed = false;
				_pool[_count] = null;

				vec2.x = p_x;
				vec2.y = p_y;
			}
			else
			{
				vec2 = new b2Vec2(p_x, p_y);
			}
	
			return vec2;
		}
	
		/**
		 * Put instance of b2Vec2 to pool.
		 */
		static private function Put(vec2:b2Vec2):void
		{
			vec2.disposed = true;
			_pool[_count++] = vec2;
		}
	
		/**
		 * Clear pool for GC.
		 */
		static public function Rid():void
		{
			b2Disposable.clearVector(_pool);
			_count = 0;
		}		
	}
}
