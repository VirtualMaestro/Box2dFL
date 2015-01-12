/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 16:18
 */
package Box2D.Common.Math
{
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * A 2-by-2 matrix. Stored in column-major order:
	 *
	 * |c11 c12|   |cos -sin|
	 * |c21 c22|   |sin  cos|
	 */
	final public class b2Mat22 extends b2SPoint
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var c11:Number;   // col1.x
		public var c12:Number;   // col1.y
		public var c21:Number;   // col2.x
		public var c22:Number;   // col2.y

		//
		b2internal var angle:Number = 0;

		/**
		 * Constructor.
		 */
		public function b2Mat22(p_c11:Number = 1.0, p_c12:Number = 0.0, p_c21:Number = 0.0, p_c22:Number = 1.0, p_tx:Number = 0.0, p_ty:Number = 0.0)
		{
			c11 = p_c11;
			c12 = p_c12;
			c21 = p_c21;
			c22 = p_c22;
			x = p_tx;
			y = p_ty;
		}

		/**
		 * Set angle in radians.
		 * @param p_angle
		 */
		final public function SetAngle(p_angle:Number):void
		{
			var c:Number = Math.cos(p_angle);
			var s:Number = Math.sin(p_angle);
			angle = p_angle;

			SetSinCos(s, c);
		}

		/**
		 * Set sin & cos.
		 */
		[Inline]
		final public function SetSinCos(p_sin:Number, p_cos:Number):void
		{
			c11 = p_cos;
			c12 = -p_sin;
			c21 = p_sin;
			c22 = p_cos;
		}

		/**
		 * Returns angle in radians.
		 * @return Number
		 */
		[Inline]
		final public function GetAngle():Number
		{
			// TODO: Optimize - method should return stored 'angle' value
			return Math.atan2(c12, c11);
		}

		/**
		 * Set this to the identity matrix.
		 */
		[Inline]
		final public function SetIdentity():void
		{
			c11 = 1.0;
			c12 = 0.0;
			c21 = 0.0;
			c22 = 1.0;

			x = 0;
			y = 0;
		}

		[Inline]
		final public function SetZero():void
		{
			c11 = 0.0; c21 = 0.0;
			c12 = 0.0; c22 = 0.0;
		}

		[Inline]
		final public function Set22(p_c11:Number, p_c12:Number, p_c21:Number, p_c22:Number):void
		{
			c11 = p_c11;
			c12 = p_c12;
			c21 = p_c21;
			c22 = p_c22;
		}

		/**
		 * Compute the inverse of this matrix, such that inv(A) * A = identity.
		 * If given matrix not null, result will set to it. In another case new matrix created
		 */
		[Inline]
		final public function GetInverse(p_outMatrix:b2Mat22 = null):b2Mat22
		{
//				ex.x = a11; ex.y = a21;
//		 		ey.x = a12; ey.y = a22;

			var a:Number = c11;
			var b:Number = c12;
			var c:Number = c21;
			var d:Number = c22;
			var det:Number = a * d - b * c;

			if (det != 0.0)
			{
				det = 1.0 / det;
			}

			var r11:Number = det * d;
			var r12:Number = -det * b;
			var r21:Number = -det * c;
			var r22:Number = det * a;

			if (p_outMatrix)
			{
				p_outMatrix.c11 = r11;
				p_outMatrix.c12 = r12;
				p_outMatrix.c21 = r21;
				p_outMatrix.c22 = r22;
			}
			else
			{
				p_outMatrix = Get(r11, r12, r21, r22);
			}

			return p_outMatrix;
		}

		/**
		 * Solve A * x = b, where b is a column vector.
		 * This is more efficient than computing the inverse in one-shot cases.
		 * Result of calculation writes to p_outResult.
		 */
		public function Solve(p_bX:Number, p_bY:Number, p_outResult:b2Vec2):void
		{
//				ex.x = a11; ex.y = a21;
//		 		ey.x = a12; ey.y = a22;

			var a11:Number = c11;
			var a12:Number = c12;
			var a21:Number = c21;
			var a22:Number = c22;

			var det:Number = a11 * a22 - a12 * a21;
			if (det != 0.0)
			{
				det = 1.0 / det;
			}

			p_outResult.x = det * (a22 * p_bX - a12 * p_bY);
			p_outResult.y = det * (a11 * p_bY - a21 * p_bX);
		}

		[Inline]
		final public function get cos():Number
		{
			return c11;
		}

		[Inline]
		final public function get sin():Number
		{
			return c21;
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

			b2Disposable.Put(this, classId);
		}

		/**
		 *  Makes a clone of current instance and returns IDisposable.
		 * @return IDisposable
		 */
		override public function Clone():IDisposable
		{
			var mat:b2Mat22 = Get(c11, c12, c21, c22, x, y);
			mat.angle = angle;

			return mat;
		}

		/**
		 * Returns new instance of b2Mat22.
		 * @return b2Mat22
		 */
		static public function Get(p_c11:Number = 1.0, p_c12:Number = 0.0, p_c21:Number = 0.0, p_c22:Number = 1.0, p_tx:Number = 0, p_ty:Number = 0):b2Mat22
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var mat:b2Mat22;

			if (instance)
			{
				mat = instance as b2Mat22;
				mat.c11 = p_c11;
				mat.c12 = p_c12;
				mat.c21 = p_c21;
				mat.c22 = p_c22;
				mat.x = p_tx;
				mat.y = p_ty;
			}
			else mat = new b2Mat22(p_c11, p_c12, p_c21, p_c22, p_tx, p_ty);

			return mat;
		}
	}
}
