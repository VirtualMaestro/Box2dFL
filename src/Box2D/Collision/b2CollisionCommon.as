/**
 * User: VirtualMaestro
 * Date: 21.12.2014
 * Time: 23:33
 */
package Box2D.Collision
{
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.Structures.b2ClipVertex;
	import Box2D.Collision.Structures.b2DistanceData;
	import Box2D.Collision.Structures.b2DistanceProxy;
	import Box2D.Collision.Structures.b2Simplex;
	import Box2D.Collision.Structures.b2SimplexCache;
	import Box2D.Collision.Structures.b2SimplexVertex;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 *
	 */
	public class b2CollisionCommon
	{
		/**
		 * Clipping for contact manifolds.
		 *
		 * @param p_vOut
		 * @param p_vIn
		 * @param p_normalX
		 * @param p_normalY
		 * @param p_offset
		 * @param p_vertexIndexA
		 * @return
		 * TODO:
		 */
		static public function b2ClipSegmentToLine(p_vOut:Vector.<b2ClipVertex>/*2 elem*/, p_vIn:Vector.<b2ClipVertex> /*2 elem*/,
		                                           p_normalX:Number, p_normalY:Number, p_offset:Number, p_vertexIndexA:int):int
		{
			return 0;
		}

		/**
		 * Determine if two generic shapes overlap.
		 *
		 * @param p_shapeA
		 * @param p_indexA
		 * @param p_shapeB
		 * @param p_indexB
		 * @param p_xfA
		 * @param p_xfB
		 * @return
		 * TODO:
		 */
		static public function b2TestOverlap(p_shapeA:b2Shape, p_indexA:int,
		                                     p_shapeB:b2Shape, p_indexB:int,
											 p_xfA:b2Mat22, p_xfB:b2Mat22):Boolean
		{

		}

		/**
		 *
		 * @param p_a
		 * @param p_b
		 * @return
		 */
		static public function b2TestOverlapAABB(p_a:b2AABB, p_b:b2AABB):Boolean
		{
			var d1X:Number;
			var d1Y:Number;
			var d2X:Number;
			var d2Y:Number;

			d1X = p_b.lowerBoundX - p_a.upperBoundX;
			d1Y = p_b.lowerBoundY - p_a.upperBoundY;

			d2X = p_a.lowerBoundX - p_b.upperBoundX;
			d2Y = p_a.lowerBoundY - p_b.upperBoundY;

			if (d1X > 0.0 || d1Y > 0.0)
				return false;

			return !(d2X > 0.0 || d2Y > 0.0);
		}

		/**
		* Compute the closest points between two shapes. Supports any combination of:
		* b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
		* On the first call set b2SimplexCache.count to zero.
		*/
		static public function b2Distance(p_data:b2DistanceData, p_cache:b2SimplexCache):void
		{
			var proxyA:b2DistanceProxy = p_data.proxyA;
			var proxyB:b2DistanceProxy = p_data.proxyB;

			var transformA:b2Mat22 = p_data.transformA;
			var transformB:b2Mat22 = p_data.transformB;

			// Initialize the simplex.
			var simplex:b2Simplex = new b2Simplex();
			simplex.ReadCache(p_cache, proxyA, transformA, proxyB, transformB);

			// Get simplex vertices as an array.
			var vertices:Vector.<b2SimplexVertex> = simplex.i_simpleVertexList;
			var k_maxIters:int = 20;

			// These store the vertices of the last simplex so that we
			// can check for duplicates and prevent cycling.
			var saveA:Vector.<int> = new Vector.<int>(3);
			var saveB:Vector.<int> = new Vector.<int>(3);
			var saveCount:int = 0;

			var distanceSqr1:Number = Number.MAX_VALUE;
			var distanceSqr2:Number = distanceSqr1;

			// Main iteration loop.
			var iter:int = 0;
			var tSimVert:b2SimplexVertex;
			var p:b2Vec2 = b2Vec2.Get();

			while(iter < k_maxIters)
			{
				// Copy simplex so we can identify duplicates.
				saveCount = simplex.m_count;

				for (var i:int = 0; i < saveCount; i++)
				{
					tSimVert = vertices[i];
					saveA[i] = tSimVert.indexA;
					saveB[i] = tSimVert.indexB;
				}

				if (saveCount == 2)
				{
					simplex.Solve2();
				}
				else if (saveCount == 3)
				{
					simplex.Solve3();
					// If we have 3 points, then the origin is in the corresponding triangle.
					break;
				}
				else
				{
					if (saveCount == 1)
					{
						break;
					}
					else
					{
						b2Assert(false, "simplex.m_count > 3 || < 1");
					}
				}

				 // Compute closest point.
				simplex.GetClosestPoint(p);
				distanceSqr2 = p.LengthSquared;
				distanceSqr1 = distanceSqr2;

				// Get search direction.
				simplex.GetSearchDirection(p);

				// Ensure the search direction is numerically fit.
				if (p.LengthSquared < b2Math.EPSILON_SQUARED)
				{
					// The origin is probably contained by a line segment
					// or triangle. Thus the shapes are overlapped.

					// We can't return zero here even though there may be overlap.
					// In case the simplex is a point, segment, or triangle it is difficult
					// to determine if the origin is contained in the CSO or very close to it.
					break;
				}

				// Compute a tentative new simplex vertex using support points.
				var vertex:b2SimplexVertex = vertices[simplex.m_count];

				// A
				var cos:Number = transformA.c11;
				var sin:Number = transformA.c12;

				var rX:Number =  cos * -p.x + sin * -p.y;
				var rY:Number = -sin * -p.x + cos * -p.y;

				vertex.indexA = proxyA.GetSupport(rX, rY);

				var pAX:Number = proxyA.GetVertexX(vertex.indexA);
				var pAY:Number = proxyA.GetVertexY(vertex.indexA);

				vertex.wAX = (cos * pAX - sin * pAY) + transformA.tx;
				vertex.wAY = (sin * pAX + cos * pAY) + transformA.ty;

				// B
				cos = transformB.c11;
				sin = transformB.c12;

				rX =  cos * p.x + sin * p.y;
				rY = -sin * p.x + cos * p.y;

				vertex.indexB = proxyB.GetSupport(rX, rY);

				var pBX:Number = proxyB.GetVertexX(vertex.indexB);
				var pBY:Number = proxyB.GetVertexY(vertex.indexB);

				vertex.wBX = (cos * pBX - sin * pBY) + transformB.tx;
				vertex.wBY = (sin * pBX + cos * pBY) + transformB.ty;

				vertex.wX = vertex.wBX - vertex.wAX;
				vertex.wY = vertex.wBY - vertex.wAY;

				// Iteration count is equated to the number of support point calls.
				++iter;

				// Check for duplicate support points. This is the main termination criteria.
				var duplicate:Boolean = false;

				for (i = 0; i < saveCount; i++)
				{
					if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
					{
						duplicate = true;
						break;
					}
				}

				// If we found a duplicate support point we must exit to avoid cycling.
				if (duplicate)
				{
					break;
				}

				// New vertex is ok and needed.
				++simplex.m_count;
			}

			p.Dispose();

			// Prepare output TODO: Optimize start//
			var pointA:b2Vec2 = b2Vec2.Get();
			var pointB:b2Vec2 = b2Vec2.Get();
			simplex.GetWitnessPoints(pointA, pointB);

			var pointAX:Number = pointA.x;
			var pointAY:Number = pointA.y;
			var pointBX:Number = pointB.x;
			var pointBY:Number = pointB.y;

			pointA.Dispose();
			pointB.Dispose();
			// TODO: Optimize end

			////
			rX = pointAX - pointBX;
			rY = pointAY - pointBY;

			var distance:Number = Math.sqrt(rX*rX + rY*rY);
			p_data.iterations = iter;

			// Cache the simplex.
			simplex.WriteCache(p_cache);

			// Apply radii if requested.
			if (p_data.useRadii)
			{
				var rA:Number = proxyA.m_radius;
				var rB:Number = proxyB.m_radius;

				if (distance > rA+rB && distance > b2Math.EPSILON)
				{
					// Shapes are still no overlapped.
					// Move the witness points to the outer surface.
					distance -= rA+rB;

					var normalX:Number = pointBX - pointAX;
					var normalY:Number = pointBY - pointAY;

					var invLength:Number = 1.0 / Math.sqrt(normalX*normalX + normalY*normalY);
					normalX *= invLength;
					normalY *= invLength;

					pointAX += rA*normalX;
					pointAY += rA*normalY;

					pointBX -= rB*normalX;
					pointBY -= rB*normalY;
				}
				else
				{
					// Shapes are overlapped when radii are considered.
					// Move the witness points to the middle.
					var pX:Number = 0.5 * pointAX + pointBX;
					var pY:Number = 0.5 * pointAY + pointBY;

					pointAX = pX;
					pointAY = pY;
					pointBX = pX;
					pointBY = pY;
					distance = 0.0;
				}
			}

			p_data.pointAX = pointAX;
			p_data.pointAY = pointAY;
			p_data.pointBX = pointBX;
			p_data.pointBY = pointBY;
			p_data.distance = distance;
		}
	}
}
