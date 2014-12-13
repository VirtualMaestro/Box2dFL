/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 10:23
 */
package Box2D.Collision
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 */
	public class b2CollideCircle
	{
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
			var pAX:Number = (cos * capX - sin * capY) + p_xfA.tx;
			var pAY:Number = (sin * capX + cos * capY) + p_xfA.ty;

			cos = p_xfB.c11;
			sin = p_xfB.c12;

			var cbpX:Number = p_circleB.m_pX;
			var cbpY:Number = p_circleB.m_pY;
			var pBX:Number = (cos * cbpX - sin * cbpY) + p_xfB.tx;
			var pBY:Number = (sin * cbpX + cos * cbpY) + p_xfB.ty;

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
			var cX:Number = (cos * cbpX - sin * cbpY) + p_xfB.tx;
			var cY:Number = (sin * cbpX + cos * cbpY) + p_xfB.ty;

			cos = p_xfA.c11;
			sin = p_xfA.c12;
			
			var px:Number = cX - p_xfA.tx;
			var py:Number = cY - p_xfA.ty;
			
			var cLocalX:Number =  cos * px + sin * py;
			var cLocalY:Number = -sin * px + cos * py;

			// Find the min separating edge.
			var normalIndex:int = 0;
			var separation:Number = Number.MIN_VALUE;
			var radius:Number = p_polygonA.m_radius + p_circleB.m_radius;
			var vertexCount:int = p_polygonA.m_vertexCount;
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
	}
}
