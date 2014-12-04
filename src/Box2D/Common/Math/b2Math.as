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
		static public const PI_2:Number = 2.0*Math.PI;
		static public const EPSILON:Number = Number.MIN_VALUE;
		static public const EPSILON_SQUARED:Number = Number.MIN_VALUE * Number.MIN_VALUE;

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

			return x*x + y*y;
		}
	}
}
