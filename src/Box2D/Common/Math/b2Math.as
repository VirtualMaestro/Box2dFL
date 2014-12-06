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

			return x*x + y*y;
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
			return p_array[p_index*2];
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
			p_array[p_index*2] = p_value;
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
			return p_array[p_index*2 + 1];
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
			p_array[p_index*2 + 1] = p_value;
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
			var ind:int = p_index*2;
			p_array[ind] = p_x;
			p_array[ind+1] = p_y;
		}
	}
}
