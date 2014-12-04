/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 19:03
 */
package Box2D.Common.Math
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	CONFIG::develop
	{
		import Box2D.assert;
	}

	use namespace b2internal;

	/**
	 * /**
	 * This describes the motion of a body/shape for TOI computation.
	 * Shapes are defined with respect to the body origin, which may
	 * no coincide with the center of mass. However, to support dynamics
	 * we must interpolate the center of mass position.
	 */
	public class b2Sweep extends b2Disposable
	{
		/**
		 * Local X center of mass position.
		 */
		public var localCenterX:Number = 0;
		public var localCenterY:Number = 0;

		/**
		 * Center world position.
		 */
		public var worldCenter0X:Number = 0;
		public var worldCenter0Y:Number = 0;
		public var worldCenterX:Number = 0;
		public var worldCenterY:Number = 0;

		/**
		 * World angle.
		 */
		public var worldAngle:Number = 0;
		public var worldAngle0:Number = 0;

		/**
		 * Time interval = [t0,1], where t0 is in [0,1]
		 */
		public var t0:Number = 0;

		/**
		* Get the interpolated transform at a specific time.
		* @param mat
		* @param alpha is a factor in [0,1], where 0 indicates t0.
		*/
		public function GetTransform(mat:b2Mat22, alpha:Number):void
		{
			var oneSubAlpha:Number = 1.0 - alpha;

			mat.tx = oneSubAlpha * worldCenter0X + alpha * worldCenterX;
			mat.ty = oneSubAlpha * worldCenter0Y + alpha * worldCenterY;
			mat.SetAngle(oneSubAlpha * worldAngle0 + alpha *  worldAngle);
			mat.tx -= (mat.c11 * localCenterX + mat.c21 * localCenterY);
			mat.ty -= (mat.c12 * localCenterX + mat.c22 * localCenterY);
		}

		/**
		* Advance the sweep forward, yielding a new initial state.
		* @param t the new initial time.
		*/
		public function Advance(t:Number) : void
		{
			CONFIG::develop
			{
				assert(t0 < 1.0, "t0 has to be < 1.0");
			}

			var beta:Number = (t - t0) / (1.0 - t0);
			worldCenter0X += beta * (worldCenterX - worldCenter0X);
			worldCenter0Y += beta * (worldCenterY - worldCenter0Y);

			worldAngle0 += beta * (worldAngle - worldAngle0);

			t0 = t;
		}

		/**
		 * Normalize an angle in radians to be between -pi and pi.
		 */
		[Inline]
		final public function Normalize():void
		{
			var PI_2:Number = b2Math.PI_2;
			var d:Number = PI_2 * int(worldAngle0/PI_2);
			worldAngle0 -= d;
			worldAngle -=  d;
		}

		/**
		 */
		final override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			Put(this);
		}

		/**
		 * Set properties to current instance from given.
		 */
		[Inline]
		final public function Set(sweep:b2Sweep):void
		{
			localCenterX = sweep.localCenterX;
			localCenterY = sweep.localCenterY;
			worldCenter0X = sweep.worldCenter0X;
			worldCenter0Y = sweep.worldCenter0Y;
			worldCenterX = sweep.worldCenterX;
			worldCenterY = sweep.worldCenterY;
			worldAngle = sweep. worldAngle;
			worldAngle0 = sweep.worldAngle0;
			t0 = sweep.t0;
		}

		/**
		 */
		final override public function Clone():IDisposable
		{
			var sweep:b2Sweep = Get();
			sweep.Set(this);

			return sweep;
		}

		
		//*************
		//**** POOL ***
		//*************
		static private var _pool:Vector.<b2Sweep> = new <b2Sweep>[];
		static private var _count:int = 0;

		/**
		 * Returns new instance of b2Sweep.
		 * @return b2Sweep
		 */
		static public function Get():b2Sweep
		{
			var sweep:b2Sweep;

			if (_count > 0)
			{
				sweep = _pool[--_count];
				sweep.disposed = false;
				_pool[_count] = null;
			}
			else
			{
				sweep = new b2Sweep();
			}

			return sweep;
		}

		/**
		 * Put instance of b2Sweep to pool.
		 */
		static private function Put(p_sweep:b2Sweep):void
		{
			p_sweep.disposed = true;
			_pool[_count++] = p_sweep;
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
