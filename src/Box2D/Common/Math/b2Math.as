/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 20:52
 */
package Box2D.Common.Math
{
	/**
	 */
	public class b2Math
	{
		static public const PI:Number = Math.PI;
		static public const PI_2:Number = 2.0 * Math.PI;
		static public const EPSILON:Number = Number.MIN_VALUE;
		static public const EPSILON_SQUARED:Number = Number.MIN_VALUE * Number.MIN_VALUE;
		static public const INV_3:Number = 1.0 / 3.0;

		/**
		 * Returns minimal of given two.
		 * @param p_a
		 * @param p_b
		 * @return
		 */
		[Inline]
		static public function Min(p_a:Number, p_b:Number):Number
		{
			return (p_b < p_a) ? p_b : p_a;
		}

		/**
		 * Returns maximal of given two.
		 * @param p_a
		 * @param p_b
		 * @return
		 */
		[Inline]
		static public function Max(p_a:Number, p_b:Number):Number
		{
			return (p_b > p_a) ? p_b : p_a;
		}

		/**
		 * Returns positive value.
		 * @return
		 */
		[Inline]
		static public function Abs(p_val:Number):Number
		{
			return (p_val * (p_val < 0 ? -1 : 1));
		}

		/**
		 * Returns distance squared.
		 * @return Number
		 */
		[Inline]
		static public function DistanceSquared(p_v1X:Number, p_v1Y:Number, p_v2X:Number, p_v2Y:Number):Number
		{
			var x:Number = p_v1X - p_v2X;
			var y:Number = p_v1Y - p_v2Y;

			return x * x + y * y;
		}

		[Inline]
		static public function LengthSquare(p_x:Number, p_y:Number):Number
		{
			return p_x * p_x + p_y * p_y;
		}

		[Inline]
		static public function Length(p_x:Number, p_y:Number):Number
		{
			return Math.sqrt(p_x * p_x + p_y * p_y);
		}

		[Inline]
		static public function invLength(p_x:Number, p_y:Number):Number
		{
			return 1/Math.sqrt(p_x * p_x + p_y * p_y);
		}

		/**
		 * Normalize vector and result save into p_outResult.
		 * Return length of vector.
		 */
		[Inline]
		static public function Normalize(p_x:Number, p_y:Number, p_outResult:b2SPoint):Number
		{
			var length:Number = Length(p_x, p_y);
			if (length < EPSILON)
			{
				return 0.0;
			}

			var invLength:Number = 1.0 / length;
			p_outResult.x = p_x * invLength;
			p_outResult.y = p_y * invLength;

			return length;
		}

		[Inline]
		static public function SubVectors(p_x1:Number, p_y1:Number, p_x2:Number, p_y2:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = p_x1 - p_x2;
			p_outResult.y = p_y1 - p_y2;
		}

		/**
		 * Multiply given vector by scalar.
		 * Result written to p_outResult.
		 */
		[Inline]
		static public function MulVectorScalar(p_vX:Number, p_vY:Number, p_scalar:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = p_vX * p_scalar;
			p_outResult.y = p_vY * p_scalar;
		}

		[Inline]
		static public function Distance(p_v1X:Number, p_v1Y:Number, p_v2X:Number, p_v2Y:Number):Number
		{
			var x:Number = p_v1X - p_v2X;
			var y:Number = p_v1Y - p_v2Y;

			return Math.sqrt(x * x + y * y);
		}

		/**
		 * Min values of two vectors.
		 * Result written to p_outResult.
		 */
		[Inline]
		static public function MinVector(p_v1X:Number, p_v1Y:Number, p_v2X:Number, p_v2Y:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = Min(p_v1X, p_v2X);
			p_outResult.y = Min(p_v1Y, p_v2Y);
		}

		/**
		 * Max values of two vectors.
		 * Result written to p_outResult.
		 */
		[Inline]
		static public function MaxVector(p_v1X:Number, p_v1Y:Number, p_v2X:Number, p_v2Y:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = Max(p_v1X, p_v2X);
			p_outResult.y = Max(p_v1Y, p_v2Y);
		}

		/**
		 * Dot product of two vectors.
		 */
		[Inline]
		static public function Dot(p_x1:Number, p_y1:Number, p_x2:Number, p_y2:Number):Number
		{
			return p_x1 * p_x2 + p_y1 * p_y2;
		}

		[Inline]
		static public function CrossVectorScalar(p_x:Number, p_y:Number, p_scalar:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = p_scalar * p_y;
			p_outResult.y = -p_scalar * p_x;
		}

		[Inline]
		static public function CrossScalarVector(p_x:Number, p_y:Number, p_scalar:Number, p_outResult:b2SPoint):void
		{
			p_outResult.x = -p_scalar * p_y;
			p_outResult.y = p_scalar * p_x;
		}

		/**
		 * Perform the cross product by two vectors. In 2D this produces a scalar.
		 */
		[Inline]
		static public function CrossVectors(p_x1:Number, p_y1:Number, p_x2:Number, p_y2:Number):Number
		{
			return p_x1 * p_y2 - p_y1 * p_x2;
		}

		/**
		 * Multiply two rotations.
		 */
		[Inline]
		static public function MulRs(p_q:b2Mat22, p_r:b2Mat22, p_outResult:b2Mat22):void
		{
			var qCos:Number = p_q.c11;
			var qSin:Number = p_q.c12;
			var rCos:Number = p_r.c11;
			var rSin:Number = p_r.c12;

			var s:Number = qSin * rCos + qCos * rSin;
			var c:Number = qCos * rCos - qSin * rSin;

			p_outResult.c11 = c;
			p_outResult.c12 = s;
			p_outResult.c21 = -s;
			p_outResult.c22 = c;
		}

		/**
		 * Multiply transpose two rotations.
		 */
		[Inline]
		static public function MulTRs(p_q:b2Mat22, p_r:b2Mat22, p_outResult:b2Mat22):void
		{
			var qCos:Number = p_q.c11;
			var qSin:Number = p_q.c12;
			var rCos:Number = p_r.c11;
			var rSin:Number = p_r.c12;

			var s:Number = qCos * rSin - qSin * rCos;
			var c:Number = qCos * rCos + qSin * rSin;

			p_outResult.c11 = c;
			p_outResult.c12 = s;
			p_outResult.c21 = -s;
			p_outResult.c22 = c;
		}

		/**
		 * Multiply transpose rotation by vector.
		 * Result written to p_outResult.
		 */
		[Inline]
		static public function MulRV(p_mat:b2Mat22, p_x:Number, p_y:Number, p_outResult:b2SPoint):void
		{
			var cos:Number = p_mat.c11;
			var sin:Number = p_mat.c12;

			p_outResult.x = cos * p_x - sin * p_y;
			p_outResult.y = sin * p_x + cos * p_y;
		}

		/**
		 * Inverse rotate a vector.
		 * Multiply transform transpose rotation by vector
		 * Result written to p_outResult.
		 */
		[Inline]
		static public function MulTRV(p_mat:b2Mat22, p_x:Number, p_y:Number, p_outResult:b2SPoint):void
		{
			var cos:Number = p_mat.c11;
			var sin:Number = p_mat.c12;

			p_outResult.x = cos * p_x + sin * p_y;
			p_outResult.y = -sin * p_x + cos * p_y;
		}

		/**
		 * Multiply transform by vector.
		 */
		[Inline]
		static public function MulTrV(p_mat:b2Mat22, p_x:Number, p_y:Number, p_outResult:b2SPoint):void
		{
			var cos:Number = p_mat.c11;
			var sin:Number = p_mat.c12;

			p_outResult.x = (cos * p_x - sin * p_y) + p_mat.x;
			p_outResult.y = (sin * p_x + cos * p_y) + p_mat.y;
		}

		/**
		 * Multiply transpose transform by vector.
		 */
		[Inline]
		static public function MulTTrV(p_mat:b2Mat22, p_x:Number, p_y:Number, p_outResult:b2SPoint):void
		{
			var cos:Number = p_mat.c11;
			var sin:Number = p_mat.c12;

			var sx:Number = p_x - p_mat.x;
			var sy:Number = p_y - p_mat.y;

			p_outResult.x =  cos * sx + sin * sy;
			p_outResult.y = -sin * sx + cos * sy;
		}

		/**
		 * Multiply two transforms.
		 */
		[Inline]
		static public function MulTrTr(p_matA:b2Mat22, p_matB:b2Mat22, p_outResult:b2Mat22):void
		{
			var qCos:Number = p_matA.c11;
			var qSin:Number = p_matA.c12;
			var rCos:Number = p_matB.c11;
			var rSin:Number = p_matB.c12;

			var s:Number = qSin * rCos + qCos * rSin;
			var c:Number = qCos * rCos - qSin * rSin;

			p_outResult.c11 = c;
			p_outResult.c12 = s;
			p_outResult.c21 = -s;
			p_outResult.c22 = c;

			var bX:Number = p_matB.x;
			var bY:Number = p_matB.y;

			p_outResult.x = (qCos * bX - qSin * bY) + p_matA.x;
			p_outResult.y = (qSin * bX + qCos * bY) + p_matA.y;
		}

		/**
		 * Multiply two transpose transforms.
		 */
		[Inline]
		static public function MulTTrTr(p_matA:b2Mat22, p_matB:b2Mat22, p_outResult:b2Mat22):void
		{
			var qCos:Number = p_matA.c11;
			var qSin:Number = p_matA.c12;
			var rCos:Number = p_matB.c11;
			var rSin:Number = p_matB.c12;

			var s:Number = qCos * rSin - qSin * rCos;
			var c:Number = qCos * rCos + qSin * rSin;

			p_outResult.c11 = c;
			p_outResult.c12 = s;
			p_outResult.c21 = -s;
			p_outResult.c22 = c;

			var bX:Number = p_matB.x - p_matA.x;   // TODO: look at the method MulTrTr, maybe bug in one of them
			var bY:Number = p_matB.y - p_matA.y;

			p_outResult.x = ( qCos * bX + qSin * bY);
			p_outResult.y = (-qSin * bX + qCos * bY);
		}

		/**
		 * Return X component by index from given array.
		 * @param p_array - Array is represented by numbers where each following two numbers are x and y.
		 * @param p_index - index of need element.
		 * @return Number
		 */
		[Inline]
		static public function getX(p_array:Vector.<Number>, p_index:int):Number
		{
			return p_array[p_index * 2];
		}

		/**
		 * Set value for X component by index for given array.
		 * @param p_value - value to set as x.
		 * @param p_array - Array is represented by numbers where each following two numbers are x and y.
		 * @param p_index - index of need element.
		 */
		[Inline]
		static public function setX(p_value:Number, p_array:Vector.<Number>, p_index:int):void
		{
			p_array[p_index * 2] = p_value;
		}

		/**
		 * Return Y component by index from given array.
		 * @param p_array - Array is represented by numbers where each following two numbers are x and y.
		 * @param p_index - index of need element.
		 * @return Number
		 */
		[Inline]
		static public function getY(p_array:Vector.<Number>, p_index:int):Number
		{
			return p_array[p_index * 2 + 1];
		}

		/**
		 * Set value for Y component by index for given array.
		 * @param p_value - value to set as y.
		 * @param p_array - Array is represented by numbers where each following two numbers are x and y.
		 * @param p_index - index of need element.
		 */
		[Inline]
		static public function setY(p_value:Number, p_array:Vector.<Number>, p_index:int):void
		{
			p_array[p_index * 2 + 1] = p_value;
		}

		/**
		 * Set value for X and Y component by index for given array.
		 * @param p_x - value to set as x.
		 * @param p_y - value to set as y.
		 * @param p_array - Array is represented by numbers where each following two numbers are x and y.
		 * @param p_index - index of need element.
		 */
		[Inline]
		static public function setXY(p_x:Number, p_y:Number, p_array:Vector.<Number>, p_index:int):void
		{
			var ind:int = p_index * 2;
			p_array[ind] = p_x;
			p_array[ind + 1] = p_y;
		}

		/**
		 * Gets xy component from given vector and writes result to p_outResult.
		 */
		[Inline]
		static public function getXY(p_array:Vector.<Number>, p_index:int, p_outResult:b2SPoint):void
		{
			p_outResult.x = getX(p_array, p_index);
			p_outResult.y = getY(p_array, p_index);
		}
	}
}
