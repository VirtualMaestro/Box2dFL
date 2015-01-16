/**
 * User: VirtualMaestro
 * Date: 26.11.2014
 * Time: 0:48
 */
package Box2D.Collision
{
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * An axis aligned bounding box.
	 */
	public class b2AABB extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var upperBoundX:Number;
		public var upperBoundY:Number;
		public var lowerBoundX:Number;
		public var lowerBoundY:Number;

		/**
		 *
		 * @param p_upperBoundX
		 * @param p_upperBoundY
		 * @param p_lowerBoundX
		 * @param p_lowerBoundY
		 */
		public function b2AABB(p_upperBoundX:Number = 0, p_upperBoundY:Number = 0, p_lowerBoundX:Number = 0, p_lowerBoundY:Number = 0)
		{
			upperBoundX = p_upperBoundX;
			upperBoundY = p_upperBoundY;
			lowerBoundX = p_lowerBoundX;
			lowerBoundY = p_lowerBoundY;
		}

		/**
		 * Copy given
		 * @param p_aabb
		 */
		public function SetTo(p_aabb:b2AABB):void
		{
			upperBoundX = p_aabb.upperBoundX;
			upperBoundY = p_aabb.upperBoundY;
			lowerBoundX = p_aabb.lowerBoundX;
			lowerBoundY = p_aabb.lowerBoundY;
		}

		/**
		 * Set all bounds to 0.
		 */
		[Inline]
		final public function SetZero():void
		{
			upperBoundX = 0;
			upperBoundY = 0;
			lowerBoundX = 0;
			lowerBoundY = 0;
		}

		/**
		 * Get the center of the AABB.
		 * IMPORTANT! Method returns new instance of b2SPoint.
		 */
		[Inline]
		final public function GetCenter(p_outResult:b2SPoint = null):b2SPoint // const
		{
			var center:b2SPoint = p_outResult;

			if (center)
			{
				center.x = (upperBoundX + lowerBoundX) * 0.5;
				center.y = (upperBoundY + lowerBoundY) * 0.5;
			}
			else
			{
				center = b2SPoint.Get((upperBoundX + lowerBoundX) * 0.5, (upperBoundY + lowerBoundY) * 0.5);
			}

			return center;
		}

		/**
		 * Get center of aabb by X axis.
		 */
		[Inline]
		final public function get centerX():Number    // const
		{
			return (upperBoundX + lowerBoundX) * 0.5;
		}

		/**
		 * Get center of aabb by Y axis.
		 */
		[Inline]
		final public function get centerY():Number    // const
		{
			return (upperBoundY + lowerBoundY) * 0.5;
		}

		/**
		 * Get the extents of the AABB (half-widths).
		 * IMPORTANT! Method returns new instance of b2SPoint if parameter wasn't set.
		 */
		[Inline]
		final public function GetExtents(p_outResult:b2SPoint = null):b2SPoint
		{
			var extents:b2SPoint = p_outResult;

			if (extents)
			{
				extents.x = (upperBoundX - lowerBoundX) * 0.5;
				extents.y = (upperBoundY - lowerBoundY) * 0.5;
			}
			else
			{
				extents = b2SPoint.Get((upperBoundX - lowerBoundX) * 0.5, (upperBoundY - lowerBoundY) * 0.5);
			}

			return extents;
		}

		/**
		 * Get the perimeter length.
		 * @return Number
		 */
		[Inline]
		final public function GetPerimeter():Number
		{
			var wx:Number = upperBoundX - lowerBoundX;
			var wy:Number = upperBoundY - lowerBoundY;

			return (wx + wy) * 2.0;
		}

		/**
		 * Combine an AABB into this one.
		 */
		[Inline]
		final public function Combine(p_aabb:b2AABB):void
		{
			lowerBoundX = b2Math.Min(lowerBoundX, p_aabb.lowerBoundX);
			lowerBoundY = b2Math.Min(lowerBoundY, p_aabb.lowerBoundY);

			upperBoundX = b2Math.Max(upperBoundX, p_aabb.upperBoundX);
			upperBoundY = b2Math.Max(upperBoundY, p_aabb.upperBoundY);
		}

		/**
		 * Combine two AABBs into this one.
		 */
		[Inline]
		final public function CombineTwo(p_aabb1:b2AABB, p_aabb2:b2AABB):void
		{
			lowerBoundX = b2Math.Min(p_aabb1.lowerBoundX, p_aabb2.lowerBoundX);
			lowerBoundY = b2Math.Min(p_aabb1.lowerBoundY, p_aabb2.lowerBoundY);

			upperBoundX = b2Math.Max(p_aabb1.upperBoundX, p_aabb2.upperBoundX);
			upperBoundY = b2Math.Max(p_aabb1.upperBoundY, p_aabb2.upperBoundY);
		}

		/**
		 * Does this aabb contain the provided AABB.
		 */
		[Inline]
		final public function Contains(p_aabb:b2AABB):Boolean
		{
			if (upperBoundX <= p_aabb.upperBoundX)
			{
				if (upperBoundY <= p_aabb.upperBoundY)
				{
					if (lowerBoundX >= p_aabb.lowerBoundX)
					{
						if (lowerBoundY >= p_aabb.lowerBoundY)
						{
							return true;
						}
					}
				}
			}

			return false;
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
		 */
		override public function Clone():IDisposable
		{
			return Get(upperBoundX, upperBoundY, lowerBoundX, lowerBoundY);
		}

		/**
		 * From Real-time Collision Detection, p179.
		 * @param p_rayCastData simultaneously input and output data.
		 */
		public function RayCast(p_rayCastData:b2RayCastData):Boolean
		{
			var tMin:Number = -Number.MAX_VALUE;
			var tMax:Number = Number.MAX_VALUE;

			var pX:Number = p_rayCastData.p1X;
			var pY:Number = p_rayCastData.p1Y;

			var dX:Number = p_rayCastData.p2X - pX;
			var dY:Number = p_rayCastData.p2Y - pY;

			var absDX:Number = b2Math.Abs(dX);
			var absDY:Number = b2Math.Abs(dY);

			var normalX:Number;
			var normalY:Number;

			var inv_d:Number;
			var t1:Number;
			var t2:Number;
			var s:Number;
			var swap:Number;

			// process X
			if (absDX < b2Math.EPSILON)
			{
				// Parallel
				if (pX < lowerBoundX || upperBoundX < pX)
				{
					return false;
				}
			}
			else
			{
				inv_d = 1.0 / dX;
				t1 = (lowerBoundX - pX) * inv_d;
				t2 = (upperBoundX - pX) * inv_d;

				// Sign of the normal vector.
				s = -1.0;

				if (t1 > t2)
				{
					swap = t1;
					t1 = t2;
					t2 = swap;

					s = 1.0;
				}

				// Push the min up
				if (t1 > tMin)
				{
					normalX = s;
					normalY = 0;
					tMin = t1;
				}

				// Pull the max down
				tMax = b2Math.Min(tMax, t2);

				if (tMin > tMax)
				{
					return false;
				}
			}

			// process Y
			if (absDY < b2Math.EPSILON)
			{
				// Parallel
				if (pY < lowerBoundY || upperBoundY < pY)
				{
					return false;
				}
			}
			else
			{
				inv_d = 1.0 / dY;
				t1 = (lowerBoundY - pY) * inv_d;
				t2 = (upperBoundY - pY) * inv_d;

				// Sign of the normal vector.
				s = -1.0;

				if (t1 > t2)
				{
					swap = t1;
					t1 = t2;
					t2 = swap;

					s = 1.0;
				}

				// Push the min up
				if (t1 > tMin)
				{
					normalX = 0;
					normalY = s;
					tMin = t1;
				}

				// Pull the max down
				tMax = b2Math.Min(tMax, t2);

				if (tMin > tMax)
				{
					return false;
				}
			}

			// Does the ray start inside the box?
			// Does the ray intersect beyond the max fraction?
			if (tMin < 0.0 || p_rayCastData.maxFraction < tMin)
			{
				return false;
			}

			// Intersection.
			p_rayCastData.fraction = tMin;
			p_rayCastData.normalX = normalX;
			p_rayCastData.normalY = normalY;

			return true;
		}

		/**
		 * Returns new instance of b2AABB.
		 * @return b2AABB
		 */
		static public function Get(p_upperBoundX:Number = 0, p_upperBoundY:Number = 0, p_lowerBoundX:Number = 0, p_lowerBoundY:Number = 0):b2AABB
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var aabb:b2AABB;

			if (instance)
			{
				aabb = instance as b2AABB;
				aabb.upperBoundX = p_upperBoundX;
				aabb.upperBoundY = p_upperBoundY;
				aabb.lowerBoundX = p_lowerBoundX;
				aabb.lowerBoundY = p_lowerBoundY;
			}
			else aabb = new b2AABB(p_upperBoundX, p_upperBoundY, p_lowerBoundX, p_lowerBoundY);

			return aabb;
		}
	}
}
