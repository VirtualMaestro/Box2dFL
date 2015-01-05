/**
 * User: VirtualMaestro
 * Date: 21.12.2014
 * Time: 23:33
 */
package Box2D.Collision
{
	import Box2D.Collision.Contact.b2ContactID;
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
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
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 *
	 */
	public class b2Collision
	{

		/**
		 * Find edge normal of max separation on A - return if separating axis is found
		 * Find edge normal of max separation on B - return if separation axis is found
		 * Choose reference edge as min(minA, minB)
		 * Find incident edge
		 * Clip
		 * The normal points from 1 to 2
 		 */
		static public function b2CollidePolygons(p_manifold:b2Manifold,
												  p_polyA:b2PolygonShape, p_xfA:b2Mat22,
												  p_polyB:b2PolygonShape, p_xfB:b2Mat22):void
		{
			p_manifold.pointCount = 0;

			var totalRadius:Number = p_polyA.m_radius + p_polyB.m_radius;
			var result:Object = b2FindMaxSeparation(p_polyA, p_xfA, p_polyB, p_xfB);
			var edgeA:int = result.bestIndex;
			var separationA:Number = result.maxSeparation;

			if (separationA > totalRadius)
			{
				return;
			}

			result = b2FindMaxSeparation(p_polyB, p_xfB, p_polyA, p_xfA);
			var edgeB:int = result.bestIndex;
			var separationB:Number = result.maxSeparation;

			if (separationB > totalRadius)
			{
				return;
			}

			var poly1:b2PolygonShape;
			var poly2:b2PolygonShape;
			var xf1:b2Mat22;
			var xf2:b2Mat22;
			var edge1:int;
			var flip:uint;
			var k_tol:Number = 0.1 * b2Settings.linearSlop;

			if (separationB > separationA + k_tol)
			{
				poly1 = p_polyB;
				poly2 = p_polyA;
				xf1 = p_xfB;
				xf2 = p_xfA;
				edge1 = edgeB;
				p_manifold.type = b2Manifold.FACE_B;
				flip = 1;
			}
			else
			{
				poly1 = p_polyA;
				poly2 = p_polyB;
				xf1 = p_xfA;
				xf2 = p_xfB;
				edge1 = edgeA;
				p_manifold.type = b2Manifold.FACE_A;
				flip = 0;
			}

			var incidentEdge:Vector.<b2ClipVertex> = new <b2ClipVertex>[b2ClipVertex.Get(), b2ClipVertex.Get()];

			b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

			var count1:int = poly1.m_count;
			var vertices1:Vector.<Number> = poly1.m_vertices;
			var iv1:int = edge1;
			var iv2:int = (edge1 + 1) < count1 ? edge1 + 1 : 0;
			var v11X:Number = b2Math.getX(vertices1, iv1);
			var v11Y:Number = b2Math.getY(vertices1, iv1);
			var v12X:Number = b2Math.getX(vertices1, iv2);
			var v12Y:Number = b2Math.getY(vertices1, iv2);
			var localTangentX:Number = v12X - v11X;
			var localTangentY:Number = v12Y - v11Y;

			var invLength:Number = 1.0 / Math.sqrt(localTangentX*localTangentX + localTangentY*localTangentY);
			localTangentX *= invLength;
			localTangentY *= invLength;

			var localNormalX:Number =  localTangentY;
			var localNormalY:Number = -localTangentX;

			var planePointX:Number = 0.5 * (v11X + v12X);
			var planePointY:Number = 0.5 * (v11Y + v12Y);

			var cos:Number = xf1.c11;
			var sin:Number = xf1.c12;

			var tangentX:Number = cos * localTangentX - sin * localTangentY;
			var tangentY:Number = sin * localTangentX + cos * localTangentY;

			var normalX:Number =  tangentY;
			var normalY:Number = -tangentX;

			v11X = (cos * v11X - sin * v11Y) + xf1.x;
			v11Y = (sin * v11X + cos * v11Y) + xf1.y;

			v12X = (cos * v12X - sin * v12Y) + xf1.x;
			v12Y = (sin * v12X + cos * v12Y) + xf1.y;

			// Face offset
			var frontOffset:Number = normalX * v11X + normalY * v11Y;

			//Side offsets, extended by polytope skin thickness
			var sideOffset1:Number = -(tangentX * v11X + tangentY * v11Y) + totalRadius;
			var sideOffset2:Number =  (tangentX * v12X + tangentY * v12Y) + totalRadius;

			// Clip incident edge against extruded edge1 side edges
			var clipPoints1:Vector.<b2ClipVertex> = new <b2ClipVertex>[b2ClipVertex.Get(), b2ClipVertex.Get()];
			var clipPoints2:Vector.<b2ClipVertex> = incidentEdge;
			var np:int;

			// Clip to box side 1
			np = b2Collision.b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangentX, -tangentY, sideOffset1, iv1);

			if (np < 2)
			{
				return;
			}

			// Clip to negative box side 1
			np = b2Collision.b2ClipSegmentToLine(clipPoints2, clipPoints1, tangentX, tangentY, sideOffset2, iv2);

			if (np < 2)
			{
				return;
			}

			// Now clipPoints2 contains the clipped points
			p_manifold.localNormalX = localNormalX;
			p_manifold.localNormalY = localNormalY;
			p_manifold.localPointX = planePointX;
			p_manifold.localPointY = planePointY;

			var pointCount:int = 0;
			var separation:Number;
			var clipVertex:b2ClipVertex;
			var manifoldPoints:Vector.<b2ManifoldPoint> = p_manifold.points;
			var cp:b2ManifoldPoint;
			var px:Number;
			var py:Number;
			var vX:Number;
			var vY:Number;

			cos = xf2.c11;
			sin = xf2.c12;

			var tx:Number = xf2.x;
			var ty:Number = xf2.y;

			for (var i:int = 0; i < b2Settings.maxManifoldPoints; i++)
			{
				clipVertex = clipPoints2[i];
				vX = clipVertex.vX;
				vY = clipVertex.vY;

				separation = (normalX * vX + normalY * vY) - frontOffset;

				if (separation <= totalRadius)
				{
					cp = manifoldPoints[pointCount];

					px = vX - tx;
					py = vY - ty;

					cp.localPointX =  cos * px + sin * py;
					cp.localPointY = -sin * px + cos * py;

					cp.id.Set(clipVertex.id);

					if (flip)
					{
						// Swap features
						cp.id.Swap();
					}

					++pointCount;
				}
			}

			p_manifold.pointCount = pointCount;

			// dispose temp clip points
			b2Disposable.clearVectorWithDispose(clipPoints1);
			b2Disposable.clearVectorWithDispose(clipPoints2);
		}

		/**
		 *
		 * @param p_c
		 * @param p_poly1
		 * @param p_xf1
		 * @param p_edge1
		 * @param p_poly2
		 * @param p_xf2
		 */
		static public function b2FindIncidentEdge(p_c:Vector.<b2ClipVertex> /*2 elements*/,
													 p_poly1:b2PolygonShape, p_xf1:b2Mat22, p_edge1:int,
													 p_poly2:b2PolygonShape, p_xf2:b2Mat22):void
		{
			var normals1:Vector.<Number> = p_poly1.m_normals;
			var normals2:Vector.<Number> = p_poly2.m_normals;
			var vertices2:Vector.<Number> = p_poly2.m_vertices;
			var count2:int = p_poly2.m_count;

			CONFIG::debug
			{
				b2Assert(0 <= p_edge1 && p_edge1 < p_poly1.m_count, "");
			}

			// Get the normal of the reference edge in poly2's frame.
			var nX:Number = b2Math.getX(normals1, p_edge1);
			var nY:Number = b2Math.getY(normals1, p_edge1);

			var cos:Number = p_xf1.c11;
			var sin:Number = p_xf1.c12;

			var rX:Number = cos * nX - sin * nY;
			var rY:Number = sin * nX + cos * nY;

			cos = p_xf2.c11;
			sin = p_xf2.c12;

			var normal1X:Number =  cos * rX + sin * rY;
			var normal1Y:Number = -sin * rX + cos * rY;

			// Find the incident edge on poly2
			var index:int = 0;
			var minDot:Number = Number.MAX_VALUE;
			var dot:Number;

			for (var i:int = 0; i < count2; i++)
			{
				nX = b2Math.getX(normals2, i);
				nY = b2Math.getY(normals2, i);

				dot = normal1X * nX + normal1Y * nY;

				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			// Build the clip vertices for the incident edge.
			var i1:int = index;
			var i2:int = (i1 + 1) < count2 ? i1+1 : 0;
			var clipVertex:b2ClipVertex = p_c[0];
			var id:b2ContactID = clipVertex.id;

			nX = b2Math.getX(vertices2, i1);
			nY = b2Math.getY(vertices2, i1);

			var tX:Number = p_xf2.x;
			var tY:Number = p_xf2.y;

			var vX:Number = (cos * nX - sin * nY) + tX;
			var vY:Number = (sin * nX + cos * nY) + tY;

			clipVertex.vX = vX;
			clipVertex.vY = vY;
			id.indexA = p_edge1;
			id.indexB = i1;
			id.typeA = b2ContactID.FACE_CF_TYPE;
			id.typeB = b2ContactID.VERTEX_CF_TYPE;

			//
			clipVertex = p_c[1];
			id = clipVertex.id;

			nX = b2Math.getX(vertices2, i2);
			nY = b2Math.getY(vertices2, i2);

			vX = (cos * nX - sin * nY) + tX;
			vY = (sin * nX + cos * nY) + tY;

			clipVertex.vX = vX;
			clipVertex.vY = vY;
			id.indexA = p_edge1;
			id.indexB = i2;
			id.typeA = b2ContactID.FACE_CF_TYPE;
			id.typeB = b2ContactID.VERTEX_CF_TYPE;
		}

		/**
		 * Use as holder for returning results of b2FindMaxSeparation method.
		 * Prevent creation and destroying new instances.
		 */
		static private var _maxSeparationResult:Object = {bestIndex:0, maxSeparation:0};

		/**
		 * Find the max separation between poly1 and poly2 using edge normals from poly1.
		 * @return Object where two properties - 'maxSeparation' and 'bestIndex'.
		 * Impossible to use returning object in own needs, should to copy values from it.
		 */
        static public function b2FindMaxSeparation(p_poly1:b2PolygonShape, p_xf1:b2Mat22,
                                                   p_poly2:b2PolygonShape, p_xf2:b2Mat22):Object
        {
	        var count1:int = p_poly1.m_count;
	        var count2:int = p_poly2.m_count;
	        var n1s:Vector.<Number> = p_poly1.m_normals;
	        var v1s:Vector.<Number> = p_poly1.m_vertices;
	        var v2s:Vector.<Number> = p_poly2.m_vertices;

	        // -- b2Transform xf = b2MulT(xf2, xf1); --//
	        var cos1:Number = p_xf2.c11;
	        var sin1:Number = p_xf2.c12;
	        var cos2:Number = p_xf1.c11;
	        var sin2:Number = p_xf1.c12;

	        var mSin:Number = (cos1 * sin2 - sin1 * cos2);
	        var mCos:Number = (cos1 * cos2 + sin1 * sin2);

	        var pX:Number = p_xf1.x - p_xf2.x;
	        var pY:Number = p_xf1.y - p_xf2.y;

	        var mTX:Number =  cos1 * pX + sin1 * pY;
	        var mTY:Number = -sin1 * pX + cos1 * pY;
	        //--------------------------------------//

	        var bestIndex:int = 0;
	        var maxSeparation:Number = -Number.MAX_VALUE;

	        var nX:Number;
	        var nY:Number;
	        var vX:Number;
	        var vY:Number;

	        for (var i:int = 0; i < count1; i++)
	        {
		        // Get poly1 normal in frame2.
		        nX = b2Math.getX(n1s, i);
		        nY = b2Math.getY(n1s, i);
		        vX = b2Math.getX(v1s, i);
		        vY = b2Math.getY(v1s, i);

		        var n1X:Number = mCos * nX - mSin * nY;
		        var n1Y:Number = mSin * nX + mCos * nY;

		        var v1X:Number = (mCos * vX - mSin * vY) + mTX;
		        var v1Y:Number = (mSin * vX + mCos * vY) + mTY;

		        // Find deepest point for normal i.
		        var si:Number = Number.MAX_VALUE;
		        var sij:Number;
		        var v2X:Number;
		        var v2Y:Number;

		        for (var j:int = 0; j < count2; j++)
		        {
			        v2X = b2Math.getX(v2s, j) - v1X;
			        v2Y = b2Math.getY(v2s, j) - v1Y;

			        sij = n1X * v2X + n1Y * v2Y;

			        if (sij < si)
			        {
				        si = sij;
			        }
		        }

		        if (si > maxSeparation)
		        {
			        maxSeparation = si;
			        bestIndex = i;
		        }
	        }

	        _maxSeparationResult.bestIndex = bestIndex;
	        _maxSeparationResult.maxSeparation = maxSeparation;

	        return _maxSeparationResult;
        }

		/**
		 *
		 * @param p_manifold
		 * @param p_circleA
		 * @param p_xfA
		 * @param p_circleB
		 * @param p_xfB
		 */
		static public function b2CollideCircles(p_manifold:b2Manifold,
		                                        p_circleA:b2CircleShape, p_xfA:b2Mat22,
	  	                                        p_circleB:b2CircleShape, p_xfB:b2Mat22):void
		{
			p_manifold.pointCount = 0;

			var cos:Number = p_xfA.c11;
			var sin:Number = p_xfA.c12;

			var capX:Number = p_circleA.m_pX;
			var capY:Number = p_circleA.m_pY;
			var pAX:Number = (cos * capX - sin * capY) + p_xfA.x;
			var pAY:Number = (sin * capX + cos * capY) + p_xfA.y;

			cos = p_xfB.c11;
			sin = p_xfB.c12;

			var cbpX:Number = p_circleB.m_pX;
			var cbpY:Number = p_circleB.m_pY;
			var pBX:Number = (cos * cbpX - sin * cbpY) + p_xfB.x;
			var pBY:Number = (sin * cbpX + cos * cbpY) + p_xfB.y;

			var dX:Number = pBX - pAX;
			var dY:Number = pBY - pAY;

			var distSqr:Number = dX * dX + dY * dY;
			var radius:Number = p_circleA.m_radius + p_circleB.m_radius;

			if (distSqr > radius*radius)
			{
				return;
			}

			p_manifold.type = b2Manifold.CIRCLES;
			p_manifold.localPointX = p_circleA.m_pX;
			p_manifold.localPointY = p_circleA.m_pY;
			p_manifold.localNormalX = 0;
			p_manifold.localNormalY = 0;
			p_manifold.pointCount = 1;

			var mPoint:b2ManifoldPoint = p_manifold.points[0];
			mPoint.localPointX = p_circleB.m_pX;
			mPoint.localPointY = p_circleB.m_pY;
			mPoint.id.key = 0;
		}

		/**
		 *
		 * @param p_manifold
		 * @param p_polygonA
		 * @param p_xfA
		 * @param p_circleB
		 * @param p_xfB
		 */
		static public function b2CollidePolygonAndCircle(p_manifold:b2Manifold,
				                                         p_polygonA:b2PolygonShape, p_xfA:b2Mat22,
			  	                                         p_circleB:b2CircleShape, p_xfB:b2Mat22):void
		{
			p_manifold.pointCount = 0;

			// Compute circle position in the frame of the polygon.
			var cos:Number = p_xfB.c11;
			var sin:Number = p_xfB.c12;

			var cbpX:Number = p_circleB.m_pX;
			var cbpY:Number = p_circleB.m_pY;
			var cX:Number = (cos * cbpX - sin * cbpY) + p_xfB.x;
			var cY:Number = (sin * cbpX + cos * cbpY) + p_xfB.y;

			cos = p_xfA.c11;
			sin = p_xfA.c12;

			var px:Number = cX - p_xfA.x;
			var py:Number = cY - p_xfA.y;

			var cLocalX:Number =  cos * px + sin * py;
			var cLocalY:Number = -sin * px + cos * py;

			// Find the min separating edge.
			var normalIndex:int = 0;
			var separation:Number = Number.MIN_VALUE;
			var radius:Number = p_polygonA.m_radius + p_circleB.m_radius;
			var vertexCount:int = p_polygonA.m_count;
			var vertices:Vector.<Number> = p_polygonA.m_vertices;
			var normals:Vector.<Number> = p_polygonA.m_normals;

			var s:Number;
			var nX:Number;
			var nY:Number;
			var vX:Number;
			var vY:Number;
			var rX:Number;
			var rY:Number;

			for (var i:int = 0; i < vertexCount; i++)
			{
				nX = b2Math.getX(normals, i);
				nY = b2Math.getY(normals, i);
				vX = b2Math.getX(vertices, i);
				vY = b2Math.getY(vertices, i);

				rX = cLocalX - vX;
				rY = cLocalY - vY;

				s = nX * rX + nY * rY;

				if (s > radius)
				{
					return;
				}

				if (s > separation)
				{
					separation = s;
					normalIndex = i;
				}
			}

			// Vertices that subtend the incident face.
			var vertIndex1:int = normalIndex;
			var vertIndex2:int = (vertIndex1 + 1) < vertexCount ? vertIndex1+1 : 0;
			var v1X:Number = b2Math.getX(vertices, vertIndex1);
			var v1Y:Number = b2Math.getY(vertices, vertIndex1);
			var v2X:Number = b2Math.getX(vertices, vertIndex2);
			var v2Y:Number = b2Math.getY(vertices, vertIndex2);
			var mp:b2ManifoldPoint;

			// If the center is inside the polygon ...
			if (separation < b2Math.EPSILON)
			{
				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.FACE_A;
				p_manifold.localNormalX = b2Math.getX(normals, normalIndex);
				p_manifold.localNormalY = b2Math.getY(normals, normalIndex);
				p_manifold.localPointX = 0.5 * (v1X + v2X);
				p_manifold.localPointY = 0.5 * (v1Y + v2Y);
				mp = p_manifold.points[0];
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;
				mp.id.key = 0;

				return;
			}

			// Compute barycentric coordinates
			var clv1X:Number = cLocalX - v1X;
			var clv1Y:Number = cLocalY - v1Y;

			var v2v1X:Number = v2X - v1X;
			var v2v1Y:Number = v2Y - v1Y;

			var clv2X:Number = cLocalX - v2X;
			var clv2Y:Number = cLocalY - v2Y;

			var v1v2X:Number = v1X - v2X;
			var v1v2Y:Number = v1Y - v2Y;

			var u1:Number = clv1X * v2v1X + clv1Y * v2v1Y;
			var u2:Number = clv2X * v1v2X + clv2Y * v1v2Y;

			var invLength:Number;
			var lnX:Number;
			var lnY:Number;

			if (u1 <= 0)
			{
				if (b2Math.DistanceSquared(cLocalX, cLocalY, v1X, v1Y) > radius*radius)
				{
					return;
				}

				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.FACE_A;
				lnX = cLocalX - v1X;
				lnY = cLocalY - v1Y;
				invLength = 1.0 / Math.sqrt(lnX*lnX + lnY*lnY);
				p_manifold.localNormalX = lnX*invLength;
				p_manifold.localNormalY = lnY*invLength;
				p_manifold.localPointX = v1X;
				p_manifold.localPointY = v1Y;
				mp = p_manifold.points[0];
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;
				mp.id.key = 0;
			}
			else if (u2 <= 0)
			{
				if (b2Math.DistanceSquared(cLocalX, cLocalY, v2X, v2Y) > radius*radius)
				{
					return;
				}

				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.FACE_A;
				lnX = cLocalX - v2X;
				lnY = cLocalY - v2Y;
				invLength = 1.0 / Math.sqrt(lnX*lnX + lnY*lnY);
				p_manifold.localNormalX = lnX*invLength;
				p_manifold.localNormalY = lnY*invLength;
				p_manifold.localPointX = v2X;
				p_manifold.localPointY = v2Y;
				mp = p_manifold.points[0];
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;
				mp.id.key = 0;
			}
			else
			{
				var faceCenterX:Number = 0.5 * (v1X + v2X);
				var faceCenterY:Number = 0.5 * (v1Y + v2Y);
				var clfcX:Number = cLocalX - faceCenterX;
				var clfcY:Number = cLocalY - faceCenterY;
				separation = clfcX * b2Math.getX(normals, vertIndex1) + clfcY * b2Math.getY(normals, vertIndex1);

				if (separation > radius)
				{
					return;
				}

				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.FACE_A;
				p_manifold.localNormalX = b2Math.getX(normals, vertIndex1);
				p_manifold.localNormalY = b2Math.getY(normals, vertIndex1);
				p_manifold.localPointX = faceCenterX;
				p_manifold.localPointY = faceCenterY;
				mp = p_manifold.points[0];
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;
				mp.id.key = 0;
			}
		}

		/**
		 * Compute contact points for edge versus circle.
		 * This accounts for edge connectivity.
		 */
		static public function b2CollideEdgeAndCircle(p_manifold:b2Manifold,
														p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
														p_circleB:b2CircleShape, p_xfB:b2Mat22):void
		{
			p_manifold.pointCount = 0;

			// Compute circle in frame of edge
			var cos:Number = p_xfB.c11;
			var sin:Number = p_xfB.c12;

			var cpX:Number = p_circleB.m_pX;
			var cpY:Number = p_circleB.m_pY;
			var rX:Number = (cos * cpX - sin * cpY) + p_xfB.x;
			var rY:Number = (sin * cpX + cos * cpY) + p_xfB.y;

			cos = p_xfA.c11;
			sin = p_xfA.c12;

			var px:Number = rX - p_xfA.x;
			var py:Number = rY - p_xfA.y;

			var QX:Number =  cos * px + sin * py;
			var QY:Number = -sin * px + cos * py;

			var AX:Number = p_edgeA.m_vertex1X;
			var AY:Number = p_edgeA.m_vertex1Y;

			var BX:Number = p_edgeA.m_vertex2X;
			var BY:Number = p_edgeA.m_vertex2Y;

			var eX:Number = BX - AX;
			var eY:Number = BY - AY;

			// Barycentric coordinates
			var BQX:Number = BX - QX;
			var BQY:Number = BY - QY;
			var u:Number = eX * BQX + eY * BQY;

			var QAX:Number = QX - AX;
			var QAY:Number = QY - AY;
			var v:Number = eX * QAX + eY * QAY;

			var radius:Number = p_edgeA.m_radius + p_circleB.m_radius;

			var indexB:uint = 0;
			var typeB:uint = b2ContactID.VERTEX_CF_TYPE;
			var indexA:uint = 0;
			var typeA:uint = b2ContactID.VERTEX_CF_TYPE;

			var mp:b2ManifoldPoint;
			var cID:b2ContactID;
			var dX:Number;
			var dY:Number;
			var dd:Number;

			// Region A
			if (v <= 0)
			{
				dX = QX - AX;
				dY = QY - AY;
				dd = dX * dX + dY * dY;

				if (dd > radius*radius)
				{
					return;
				}

				// Is there an edge connected to A?
				if (p_edgeA.m_hasVertex0)
				{
					var A1X:Number = p_edgeA.m_vertex0X;
					var A1Y:Number = p_edgeA.m_vertex0Y;

					var e1X:Number = AX - A1X;
					var e1Y:Number = AY - A1Y;

					var B1QX:Number = AX - QX;
					var B1QY:Number = AY - QY;

					var u1:Number = e1X * B1QX + e1Y * B1QY;

					// Is the circle in Region AB of the previous edge?
					if (u1 > 0.0)
					{
						return;
					}
				}

				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.CIRCLES;
				p_manifold.localNormalX = 0;
				p_manifold.localNormalY = 0;
				p_manifold.localPointX = AX;
				p_manifold.localPointY = AY;
				mp = p_manifold.points[0];
				cID = mp.id;
				cID.key = 0;
				cID.indexA = indexA;
				cID.typeA  = typeA;
				cID.indexB = indexB;
				cID.typeB  = typeB;
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;

				return;
			}

			// Region B
			if (u <= 0.0)
			{
				dX = QX - BX;
				dY = QY - BY;
				dd = dX * dX + dY * dY;

				if (dd > radius*radius)
				{
					return;
				}

				// Is there an edge connected to B?
				if (p_edgeA.m_hasVertex3)
				{
					var B2X:Number = p_edgeA.m_vertex3X;
					var B2Y:Number = p_edgeA.m_vertex3Y;

					var e2X:Number = B2X - BX;
					var e2Y:Number = B2Y - BY;

					var QA2X:Number = QX - BX;
					var QA2Y:Number = QY - BY;

					var v2:Number = e2X * QA2X + e2Y * QA2Y;

					// Is the circle in Region AB of the next edge?
					if (v2 > 0.0)
					{
						return;
					}
				}

				indexA = 1;
				p_manifold.pointCount = 1;
				p_manifold.type = b2Manifold.CIRCLES;
				p_manifold.localNormalX = 0;
				p_manifold.localNormalY = 0;
				p_manifold.localPointX = BX;
				p_manifold.localPointY = BY;
				mp = p_manifold.points[0];
				cID = mp.id;
				cID.key = 0;
				cID.indexA = indexA;
				cID.typeA  = typeA;
				cID.indexB = indexB;
				cID.typeB  = typeB;
				mp.localPointX = p_circleB.m_pX;
				mp.localPointY = p_circleB.m_pY;

				return;
			}

			// Region AB
			var den:Number = eX * eX + eY * eY;

			CONFIG::debug
			{
				b2Assert(den > 0, "den > 0")
			}

			var uavbX:Number = u*AX + v*BX;
			var uavbY:Number = u*AY + v*BY;
			var inv_den:Number = 1.0/den;
			var PX:Number = uavbX * inv_den;
			var PY:Number = uavbY * inv_den;

			dX = QX - PX;
			dY = QY - PY;
			dd = dX * dX + dY * dY;

			if (dd > radius*radius)
			{
				return;
			}

			var nX:Number = -eY;
			var nY:Number = eX;

			var qaX:Number = QX - AX;
			var qaY:Number = QY - AY;
			if ((nX * qaX + nY * qaY) < 0)
			{
				nX = -nX;
				nY = -nY;
			}

			var invLength:Number = 1.0 / Math.sqrt(nX*nX + nY*nY);
			nX *= invLength;
			nY *= invLength;

			indexA = 0;
			typeA = b2ContactID.FACE_CF_TYPE;
			p_manifold.pointCount = 1;
			p_manifold.type = b2Manifold.FACE_A;
			p_manifold.localNormalX = nX;
			p_manifold.localNormalY = nY;
			p_manifold.localPointX = AX;
			p_manifold.localPointY = AY;
			mp = p_manifold.points[0];
			cID = mp.id;
			cID.key = 0;
			cID.indexA = indexA;
			cID.typeA  = typeA;
			cID.indexB = indexB;
			cID.typeB  = typeB;
			mp.localPointX = p_circleB.m_pX;
			mp.localPointY = p_circleB.m_pY;
		}

		/**
		 *
		 * @param p_manifold
		 * @param p_edgeA
		 * @param p_xfA
		 * @param p_polygonB
		 * @param p_xfB
		 */
		static public function b2CollideEdgeAndPolygon(p_manifold:b2Manifold,
														 p_edgeA:b2EdgeShape, p_xfA:b2Mat22,
														 p_polygonB:b2PolygonShape, p_xfB:b2Mat22):void
		{
			var collider:b2EPCollider = new b2EPCollider();
			collider.Collide(p_manifold, p_edgeA, p_xfA, p_polygonB, p_xfB);
		}

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
		 */
		static public function b2ClipSegmentToLine(p_vOut:Vector.<b2ClipVertex>/*2 elem*/, p_vIn:Vector.<b2ClipVertex> /*2 elem*/,
		                                           p_normalX:Number, p_normalY:Number, p_offset:Number, p_vertexIndexA:int):int
		{
			// Start with no output points
			var numOut:int = 0;

			var vIn0:b2ClipVertex = p_vIn[0];
			var vIn1:b2ClipVertex = p_vIn[1];

			// Calculate the distance of end points to the line
			var distance0:Number = (p_normalX * vIn0.vX + p_normalY * vIn0.vY) - p_offset;
			var distance1:Number = (p_normalX * vIn1.vX + p_normalY * vIn1.vY) - p_offset;

			// If the points are behind the plane
			if (distance0 <= 0.0) p_vOut[numOut++].Set(vIn0);
			if (distance1 <= 0.0) p_vOut[numOut++].Set(vIn1);

			// If the points are on different sides of the plane
			if (distance0 * distance1 < 0.0)
			{
				// Find intersection point of edge and plane
				var interp:Number = distance0 / (distance0 - distance1);
				var clipVertexOut:b2ClipVertex = p_vOut[numOut];

				clipVertexOut.vX = vIn0.vX + interp * (vIn1.vX - vIn0.vX);
				clipVertexOut.vY = vIn0.vY + interp * (vIn1.vY - vIn0.vY);

				// VertexA is hitting edgeB
				var contactId:b2ContactID = clipVertexOut.id;
				contactId.indexA = p_vertexIndexA;
				contactId.indexB = vIn0.id.indexB;
				contactId.typeA = b2ContactID.VERTEX_CF_TYPE;
				contactId.typeB = b2ContactID.FACE_CF_TYPE;
				++numOut;
			}

			return numOut;
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
		 * TODO: Optimize allocation memory
		 */
		static public function b2TestOverlap(p_shapeA:b2Shape, p_indexA:int,
		                                     p_shapeB:b2Shape, p_indexB:int,
											 p_xfA:b2Mat22, p_xfB:b2Mat22):Boolean
		{
			var distanceData:b2DistanceData = new b2DistanceData();
			distanceData.proxyA.Set(p_shapeA, p_indexA);
			distanceData.proxyB.Set(p_shapeB, p_indexB);
			distanceData.transformA = p_xfA;
			distanceData.transformB = p_xfB;
			distanceData.useRadii = true;

			var cache:b2SimplexCache = new b2SimplexCache();
			b2Distance(distanceData, cache);

			return distanceData.distance < 10.0*b2Math.EPSILON
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

				vertex.wAX = (cos * pAX - sin * pAY) + transformA.x;
				vertex.wAY = (sin * pAX + cos * pAY) + transformA.y;

				// B
				cos = transformB.c11;
				sin = transformB.c12;

				rX =  cos * p.x + sin * p.y;
				rY = -sin * p.x + cos * p.y;

				vertex.indexB = proxyB.GetSupport(rX, rY);

				var pBX:Number = proxyB.GetVertexX(vertex.indexB);
				var pBY:Number = proxyB.GetVertexY(vertex.indexB);

				vertex.wBX = (cos * pBX - sin * pBY) + transformB.x;
				vertex.wBY = (sin * pBX + cos * pBY) + transformB.y;

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

			// Prepare output TODO: Optimize start//
			var pointA:b2Vec2 = p;
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
