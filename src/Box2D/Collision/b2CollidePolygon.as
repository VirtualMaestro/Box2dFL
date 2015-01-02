/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 12:25
 */
package Box2D.Collision
{
	import Box2D.Collision.Contact.b2ContactID;
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Structures.b2ClipVertex;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 *
	 */
	public class b2CollidePolygon
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

			v11X = (cos * v11X - sin * v11Y) + xf1.tx;
			v11Y = (sin * v11X + cos * v11Y) + xf1.ty;

			v12X = (cos * v12X - sin * v12Y) + xf1.tx;
			v12Y = (sin * v12X + cos * v12Y) + xf1.ty;

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
			np = b2CollisionCommon.b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangentX, -tangentY, sideOffset1, iv1);

			if (np < 2)
			{
				return;
			}

			// Clip to negative box side 1
			np = b2CollisionCommon.b2ClipSegmentToLine(clipPoints2, clipPoints1, tangentX, tangentY, sideOffset2, iv2);

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

			var tx:Number = xf2.tx;
			var ty:Number = xf2.ty;

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

			var tX:Number = p_xf2.tx;
			var tY:Number = p_xf2.ty;

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

	        var pX:Number = p_xf1.tx - p_xf2.tx;
	        var pY:Number = p_xf1.ty - p_xf2.ty;

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
	}
}
