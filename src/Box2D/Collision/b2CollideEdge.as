/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:32
 */
package Box2D.Collision
{
	import Box2D.Collision.Contact.b2ContactID;
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 */
	public class b2CollideEdge
	{
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
			var rX:Number = (cos * cpX - sin * cpY) + p_xfB.tx;
			var rY:Number = (sin * cpX + cos * cpY) + p_xfB.ty;

			cos = p_xfA.c11;
			sin = p_xfA.c12;

			var px:Number = rX - p_xfA.tx;
			var py:Number = rY - p_xfA.ty;

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
	}
}
