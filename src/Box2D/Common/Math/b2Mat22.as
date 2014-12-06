/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 16:18
 */
package Box2D.Common.Math
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * A 2-by-2 matrix. Stored in column-major order:
	 *
	 * |c11 c21|   |cos -sin|
	 * |c12 c22|   |sin  cos|
	 */
	final public class b2Mat22 extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var c11:Number;   // col1.x
		public var c12:Number;   // col1.y
		public var c21:Number;   // col2.x
		public var c22:Number;   // col2.y

		/**
		 * Represents x position
		 */
		public var tx:Number = 0;

		/**
		 * Represents y position
		 */
		public var ty:Number = 0;

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
			tx = p_tx;
			ty = p_ty;
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
			c11 = c;
			c21 = -s;
			c12 = s;
			c22 = c;
		}

		/**
		 * Returns angle in radians.
		 * @return Number
		 */
		final public function GetAngle():Number
		{
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
		}

		/**
		 * Compute the inverse of this matrix, such that inv(A) * A = identity.
		 * If given matrix not null, result will set to it. In another case new matrix created
		 */
		final public function GetInverse(p_outMatrix:b2Mat22 = null):b2Mat22
		{
			var a:Number = c11;
			var b:Number = c21;
			var c:Number = c12;
			var d:Number = c22;
			var det:Number = a * d - b * c;

			if (det != 0.0)
			{
				det = 1.0 / det;
			}

			var r11:Number = det * d;
			var r21:Number = -det * b;
			var r12:Number = -det * c;
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
			var mat:b2Mat22 = Get(c11, c12, c21, c22, tx, ty);
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
				mat.tx  = p_tx;
				mat.ty  = p_ty;
			}
			else mat = new b2Mat22(p_c11, p_c12, p_c21, p_c22, p_tx, p_ty);

			return mat;
		}
	}
}
