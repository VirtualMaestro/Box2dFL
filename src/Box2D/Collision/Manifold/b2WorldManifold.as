/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 16:33
 */
package Box2D.Collision.Manifold
{
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Settings;

	/**
	 * This is used to compute the current state of a contact manifold.
	 */
	public class b2WorldManifold
	{
		/**
		 * X component of world vector pointing from A to B
		 */
		public var normalX:Number;

		/**
		 * Y component of world vector pointing from A to B
		 */
		public var normalY:Number;

		/**
		 * World contact points (point of intersection).
		 * Vector of Number, where every two components are 'x' and 'y' respectively.
		 */
		public var points:Vector.<Number>;

		/**
		 * Negative value indicates overlap, in meters
		 */
		public var separations:Vector.<Number>;

		/**
		 */
		public function b2WorldManifold()
		{
			points = new Vector.<Number>(b2Settings.maxManifoldPoints * 2, true);
			separations = new Vector.<Number>(b2Settings.maxManifoldPoints, true);
		}

		/**
		 * Evaluate the manifold with supplied transforms.
		 * This assumes modest motion from the original state.
		 * This does not change the point count, impulses, etc.
		 * The radii must come from the shapes that generated the manifold.
		 */
		public function Initialize(p_manifold:b2Manifold, p_xfA:b2Mat22, p_radiusA:Number, p_xfB:b2Mat22, p_radiusB:Number):void
		{
			var manifoldPointCount:Number = p_manifold.pointCount;

			if (manifoldPointCount > 0)
			{
				var cosA:Number = p_xfA.cos;
				var sinA:Number = p_xfA.sin;
				var xA:Number = p_xfA.x;
				var yA:Number = p_xfA.y;

				var cosB:Number = p_xfB.cos;
				var sinB:Number = p_xfB.sin;
				var xB:Number = p_xfB.x;
				var yB:Number = p_xfB.y;

				var cAX:Number;
				var cAY:Number;
				var cBX:Number;
				var cBY:Number;

				var manifoldPoints:Vector.<b2ManifoldPoint> = p_manifold.points;
				var manifoldLocalPointX:Number = p_manifold.localPointX;
				var manifoldLocalPointY:Number = p_manifold.localPointY;

				var manifoldLocalNormalX:Number;
				var manifoldLocalNormalY:Number;

				var planePointX:Number;
				var planePointY:Number;

				var clipPointX:Number;
				var clipPointY:Number;
				var mp:b2ManifoldPoint;
				var j:int;

				switch (p_manifold.type)
				{
					case b2Manifold.CIRCLES:
					{
						normalX = 1.0;
						normalY = 0.0;

						var pointAX:Number = (cosA * manifoldLocalPointX - sinA * manifoldLocalPointY) + xA;
						var pointAY:Number = (sinA * manifoldLocalPointX + cosA * manifoldLocalPointY) + yA;

						var manifoldPoint:b2ManifoldPoint = manifoldPoints[0];
						var mplX:Number = manifoldPoint.localPointX;
						var mplY:Number = manifoldPoint.localPointY;

						var pointBX:Number = (cosB * mplX - sinB * mplY) + xB;
						var pointBY:Number = (sinB * mplX + cosB * mplY) + yB;

						if (b2Math.DistanceSquared(pointAX, pointAY, pointBX, pointBY) > b2Math.EPSILON_SQUARED)
						{
							normalX = pointBX - pointAX;
							normalY = pointBY - pointAY;

							Normalize();
						}

						cAX = pointAX + p_radiusA * normalX;
						cAY = pointAY + p_radiusA * normalY;

						cBX = pointBX - p_radiusB * normalX;
						cBY = pointBY - p_radiusB * normalY;

						points[0] = (cAX + cBX) * 0.5;
						points[1] = (cAY + cBY) * 0.5;
						separations[0] = ((cBX - cAX) * normalX + ((cBY - cAY) * normalY));

						break;
					}

					case b2Manifold.FACE_A:
					{
						manifoldLocalNormalX = p_manifold.localNormalX;
						manifoldLocalNormalY = p_manifold.localNormalY;
						normalX = (cosA * manifoldLocalNormalX - sinA * manifoldLocalNormalY);
						normalY = (sinA * manifoldLocalNormalX + cosA * manifoldLocalNormalY);

						planePointX = (cosA * manifoldLocalPointX - sinA * manifoldLocalPointY) + xA;
						planePointY = (sinA * manifoldLocalPointX + cosA * manifoldLocalPointY) + yA;

						for (var i:int = 0; i < manifoldPointCount; i++)
						{
							mp = manifoldPoints[i];
							manifoldLocalPointX = mp.localPointX;
							manifoldLocalPointY = mp.localPointY;

							clipPointX = (cosB * manifoldLocalPointX - sinB * manifoldLocalPointY) + xB;
							clipPointY = (sinB * manifoldLocalPointX + cosB * manifoldLocalPointY) + yB;

							cAX = clipPointX + (p_radiusA - ((clipPointX - planePointX) * normalX)) * normalX;
							cAY = clipPointY + (p_radiusA - ((clipPointY - planePointY) * normalY)) * normalY;

							cBX = clipPointX - p_radiusB * normalX;
							cBY = clipPointY - p_radiusB * normalY;

							j = i * 2;
							points[j] = (cAX + cBX) * 0.5;
							points[j + 1] = (cAY + cBY) * 0.5;

							separations[i] = (cBX - cAX) * normalX + (cBY - cAY) * normalY;
						}

						break;
					}

					case b2Manifold.FACE_B:
					{
						manifoldLocalNormalX = p_manifold.localNormalX;
						manifoldLocalNormalY = p_manifold.localNormalY;

						normalX = cosB * manifoldLocalNormalX - sinB * manifoldLocalNormalY;
						normalY = sinB * manifoldLocalNormalX + cosB * manifoldLocalNormalY;

						planePointX = (cosB * manifoldLocalPointX - sinB * manifoldLocalPointY) + xB;
						planePointY = (sinB * manifoldLocalPointX + cosB * manifoldLocalPointY) + yB;

						for (i = 0; i < manifoldPointCount; i++)
						{
							mp = manifoldPoints[i];
							manifoldLocalPointX = mp.localPointX;
							manifoldLocalPointY = mp.localPointY;

							clipPointX = (cosA * manifoldLocalPointX - sinA * manifoldLocalPointY) + xA;
							clipPointY = (sinA * manifoldLocalPointX + cosA * manifoldLocalPointY) + yA;

							cBX = clipPointX + (p_radiusB - ((clipPointX - planePointX) * normalX)) * normalX;
							cBY = clipPointY + (p_radiusB - ((clipPointY - planePointY) * normalY)) * normalY;

							cAX = clipPointX - p_radiusA * normalX;
							cAY = clipPointY - p_radiusA * normalY;

							j = i * 2;
							points[j] = (cAX + cBX) * 0.5;
							points[j + 1] = (cAY + cBY) * 0.5;

							separations[i] = (cAX - cBX) * normalX + (cAY - cBY) * normalY;
						}

						// Ensure normal points from A to B.
						normalX = -normalX;
						normalY = -normalY;

						break;
					}
				}
			}
		}

		[Inline]
		final private function Normalize():void
		{
			var length:Number = Math.sqrt(normalX * normalX + normalY * normalY);
			var invLength:Number = 1.0 / length;
			normalX *= invLength;
			normalY *= invLength;
		}
	}
}
