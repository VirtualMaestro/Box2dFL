/**
 * User: VirtualMaestro
 * Date: 11.01.2015
 * Time: 0:21
 */
package Box2D.Collision.Structures
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.b2Assert;

	/**
	 */
	public class b2PositionSolverManifold
	{
		public var normalX:Number;
		public var normalY:Number;
		public var pointX:Number;
		public var pointY:Number;
		public var separation:Number;

		/**
		 */
		public function Initialize(p_pc:b2ContactPositionConstraint, p_xfA:b2Mat22, p_xfB:b2Mat22, p_index:int):void
		{
			var helperPoint:b2SPoint = b2SPoint.Get();
			var planePointX:Number;
			var planePointY:Number;
			var clipPointX:Number;
			var clipPointY:Number;

			CONFIG::debug
			{
				b2Assert(p_pc.pointCount > 0, "pc.pointCount > 0");
			}

			switch (p_pc.type)
			{
				case b2Manifold.CIRCLES:
				{
					b2Math.MulTrV(p_xfA, p_pc.localPointX, p_pc.localPointY, helperPoint);
					var pointAX:Number = helperPoint.x;
					var pointAY:Number = helperPoint.y;

					b2Math.MulTrV(p_xfB, b2Math.getX(p_pc.localPoints, 0), b2Math.getY(p_pc.localPoints, 0), helperPoint);
					var pointBX:Number = helperPoint.x;
					var pointBY:Number = helperPoint.y;

					normalX = pointBX - pointAX;
					normalY = pointBY - pointAY;

					var invLength:Number = b2Math.invLength(normalX, normalY);
					normalX *= invLength;
					normalY *= invLength;

					pointX = 0.5 * (pointAX + pointBX);
					pointY = 0.5 * (pointAY + pointBY);

					separation = b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), normalX, normalY) - p_pc.radiusA - p_pc.radiusB;

					break;
				}

				case b2Manifold.FACE_A:
				{
					b2Math.MulRV(p_xfA, p_pc.localNormalX, p_pc.localNormalY, helperPoint);
					normalX = helperPoint.x;
					normalY = helperPoint.y;

					b2Math.MulTrV(p_xfA, p_pc.localPointX, p_pc.localPointY, helperPoint);
					planePointX = helperPoint.x;
					planePointY = helperPoint.y;

					b2Math.MulTrV(p_xfB, b2Math.getX(p_pc.localPoints, p_index), b2Math.getY(p_pc.localPoints, p_index), helperPoint);
					clipPointX = helperPoint.x;
					clipPointY = helperPoint.y;

					separation = b2Math.Dot((clipPointX - planePointX), (clipPointY - planePointY), normalX, normalY) - p_pc.radiusA - p_pc.radiusB;
					pointX = clipPointX;
					pointY = clipPointY;
				}
					break;

				case b2Manifold.FACE_B:
				{
					b2Math.MulRV(p_xfB, p_pc.localNormalX, p_pc.localNormalY, helperPoint);
					normalX = helperPoint.x;
					normalY = helperPoint.y;

					b2Math.MulTrV(p_xfB, p_pc.localPointX, p_pc.localPointY, helperPoint);
					planePointX = helperPoint.x;
					planePointY = helperPoint.y;

					b2Math.MulTrV(p_xfA, b2Math.getX(p_pc.localPoints, p_index), b2Math.getY(p_pc.localPoints, p_index), helperPoint);
					clipPointX = helperPoint.x;
					clipPointY = helperPoint.y;

					separation = b2Math.Dot((clipPointX - planePointX), (clipPointY - planePointY), normalX, normalY) - p_pc.radiusA - p_pc.radiusB;
					pointX = clipPointX;
					pointY = clipPointY;

					// Ensure normal points from A to B
					normalX = -normalX;
					normalY = -normalY;
				}
					break;
			}

			helperPoint.Dispose();
		}
	}
}
