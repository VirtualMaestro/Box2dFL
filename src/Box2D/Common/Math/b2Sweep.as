/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 19:03
 */
package Box2D.Common.Math
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	CONFIG::debug
	{
		import Box2D.b2Assert;
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
		static b2internal var classId:uint = b2Disposable.getClassId();

		/**
		 * Local X center of mass position.
		 */
		public var localCenterX:Number = 0;
		public var localCenterY:Number = 0;

		/**
		 * Center world position.
		 */
		public var worldCenterX0:Number = 0;
		public var worldCenterY0:Number = 0;
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

			mat.tx = oneSubAlpha * worldCenterX0 + alpha * worldCenterX;
			mat.ty = oneSubAlpha * worldCenterY0 + alpha * worldCenterY;
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
			CONFIG::debug
			{
				b2Assert(t0 < 1.0, "t0 has to be < 1.0");
			}

			var beta:Number = (t - t0) / (1.0 - t0);
			worldCenterX0 += beta * (worldCenterX - worldCenterX0);
			worldCenterY0 += beta * (worldCenterY - worldCenterY0);

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

			b2Disposable.Put(this, classId);
		}

		/**
		 * Set properties to current instance from given.
		 */
		[Inline]
		final public function Set(p_sweep:b2Sweep):void
		{
			localCenterX = p_sweep.localCenterX;
			localCenterY = p_sweep.localCenterY;
			worldCenterX0 = p_sweep.worldCenterX0;
			worldCenterY0 = p_sweep.worldCenterY0;
			worldCenterX = p_sweep.worldCenterX;
			worldCenterY = p_sweep.worldCenterY;
			worldAngle = p_sweep. worldAngle;
			worldAngle0 = p_sweep.worldAngle0;
			t0 = p_sweep.t0;
		}

		/**
		 */
		final override public function Clone():IDisposable
		{
			var sweep:b2Sweep = Get();
			sweep.Set(this);

			return sweep;
		}

		/**
		 * Returns new instance of b2Sweep.
		 * @return b2Sweep
		 */
		[Inline]
		static public function Get():b2Sweep
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var sweep:b2Sweep;

			if (instance) sweep = instance as b2Sweep;
			else sweep = new b2Sweep();

			return sweep;
		}
	}
}
