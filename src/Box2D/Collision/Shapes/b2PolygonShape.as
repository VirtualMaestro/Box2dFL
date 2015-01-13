/**
 * User: VirtualMaestro
 * Date: 03.12.2014
 * Time: 21:55
 */
package Box2D.Collision.Shapes
{
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Collision.b2AABB;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;

	CONFIG::debug
	{
		import Box2D.b2Assert;

		import flash.utils.getQualifiedClassName;
	}

	use namespace b2internal;

	/**
	 * A convex polygon. It is assumed that the interior of the polygon is to
	 * the left of each edge.
	 * Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
	 * In most cases you should not need many vertices for a convex polygon.
	 */
	public class b2PolygonShape extends b2Shape
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		b2internal var m_centroidX:Number;
		b2internal var m_centroidY:Number;

		/**
		 * Vertices of polygon.
		 * Every following two elements is x and y.
		 */
		b2internal var m_vertices:Vector.<Number>;

		/**
		 * Normals of polygon.
		 * Every following two elements is x and y.
		 */
		b2internal var m_normals:Vector.<Number>;

		/**
		 * Vertex count.
		 */
		b2internal var m_count:int;

		/**
		 */
		public function b2PolygonShape()
		{
			m_type = b2Shape.POLYGON;
			m_radius = b2Settings.polygonRadius;
			m_centroidX = 0;
			m_centroidY = 0;
			m_count = 0;
			m_vertices = new <Number>[];
			m_normals = new <Number>[];
		}

		/**
		 * Init current instance with given (copy properties from given to current).
		 * @param p_shape b2PolygonShape
		 */
		override public function SetTo(p_shape:b2Shape):void
		{
			CONFIG::debug
			{
				b2Assert((p_shape is b2PolygonShape),
				         "given parameter has to be b2PolygonShape class. Current instance has type: " + getQualifiedClassName(p_shape));
			}

			var polygonShape:b2PolygonShape = p_shape as b2PolygonShape;
			m_radius = polygonShape.m_radius;
			m_centroidX = polygonShape.m_centroidX;
			m_centroidY = polygonShape.m_centroidY;
			m_count = polygonShape.m_count;

			var vertices:Vector.<Number> = polygonShape.m_vertices;
			var normals:Vector.<Number> = polygonShape.m_normals;

			for (var i:int = 0; i < m_count; i++)
			{
				m_vertices[i] = vertices[i];
				m_normals[i] = normals[i];
			}
		}

		/** Create a convex hull from the given array of local points.
		 * The count must be in the range [3, b2_maxPolygonVertices].
		 * @warning the points may be re-ordered, even if they form a convex polygon
		 * @warning collinear points are handled but not removed.
		 * Collinear points may lead to poor stacking behavior.
		 */
		public function Set(p_vertices:Vector.<Number>, p_count:int):void
		{
			CONFIG::debug
			{
				b2Assert(p_count <= b2Settings.maxPolygonVertices,
				         "too many vertices of polygon (" + p_count + "). Acceptable count is " + b2Settings.maxPolygonVertices);
				b2Assert(p_count > 2, "Polygon is degenerate");
			}

			var n:int = b2Math.Min(p_count, b2Settings.maxPolygonVertices);

			// Perform welding and copy vertices into local buffer.
			var vX:Number;
			var vY:Number;
			var unique:Boolean;
			var tempCount:int = 0;
			var ps:Vector.<Number> = new <Number>[];

			for (var i:int = 0; i < n; i++)
			{
				vX = b2Math.getX(p_vertices, i);
				vY = b2Math.getY(p_vertices, i);

				unique = true;

				for (var j:int = 0; j < tempCount; j++)
				{
					if (b2Math.DistanceSquared(vX, vY, b2Math.getX(ps, j), b2Math.getY(ps, j)) < (b2Settings.linearSlop * 0.5))
					{
						unique = false;
						break;
					}
				}

				if (unique)
				{
					b2Math.setXY(vX, vY, ps, tempCount);
					tempCount++;
				}
			}

			n = tempCount;

			CONFIG::debug
			{
				b2Assert(n > 2, "Polygon is degenerate");
			}

			// Create the convex hull using the Gift wrapping algorithm
			// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

			// Find the right most point on the hull
			var i0:int = 0;
			var x0:Number = ps[0];
			var x:Number;
			for (i = 1; i < n; i++)
			{
				x = b2Math.getX(ps, i);

				if (x > x0 || (x == x0 && b2Math.getY(ps, i) < b2Math.getY(ps, i0)))
				{
					i0 = i;
					x0 = x;
				}
			}

			var hull:Vector.<int> = new <int>[];
			var m:int = 0;
			var ih:int = i0;
			var ie:int;

			for (; ;)
			{
				hull[m] = ih;
				ie = 0;

				for (j = 1; j < n; j++)
				{
					if (ie == ih)
					{
						ie = j;
						continue;
					}

					var psieX:Number = b2Math.getX(ps, ie);
					var psieY:Number = b2Math.getY(ps, ie);
					var hullM:int = hull[m];

					var pshullX:Number = b2Math.getX(ps, hullM);
					var pshullY:Number = b2Math.getY(ps, hullM);

					var rX:Number = psieX - pshullX;
					var rY:Number = psieY - pshullY;

					var psjX:Number = b2Math.getX(ps, j);
					var psjY:Number = b2Math.getY(ps, j);

					vX = psjX - pshullX;
					vY = psjY - pshullY;

					var c:Number = rX * vY - rY * vX;

					// Collinearity check
					if (c < 0.0 || (c == 0.0 && (vX * vX + vY * vY) > (rX * rX + rY * rY)))
					{
						ie = j;
					}
				}

				++m;
				ih = ie;

				if (ie == i0)
				{
					break;
				}
			}

			CONFIG::debug
			{
				b2Assert(m > 2, "Polygon is degenerate");
			}

			m_count = m;

			// Copy vertices
			for (i = 0; i < m; i++)
			{
				hullM = hull[i];
				b2Math.setXY(b2Math.getX(ps, hullM), b2Math.getY(ps, hullM), m_vertices, i);
			}

			// Compute normals. Ensure the edges have non-zero length.
			for (i = 0; i < m; i++)
			{
				var i1:int = i;
				var i2:int = (i + 1) < m ? i + 1 : 0;
				var edgeX:Number = b2Math.getX(m_vertices, i2) - b2Math.getX(m_vertices, i1);
				var edgeY:Number = b2Math.getY(m_vertices, i2) - b2Math.getY(m_vertices, i1);

				CONFIG::debug
				{
					b2Assert((edgeX * edgeX + edgeY * edgeY) > b2Math.EPSILON_SQUARED, "edge has zero length");
				}

				var nX:Number = edgeY;
				var nY:Number = -edgeX;
				var invLength:Number = 1.0 / Math.sqrt(nX * nX + nY * nY);
				nX *= invLength;
				nY *= invLength;

				b2Math.setXY(nX, nY, m_normals, i);
			}

			// Compute the polygon centroid.
			var point:b2SPoint = ComputeCentroid(m_vertices, m);
			m_centroidX = point.x;
			m_centroidY = point.y;
			point.Dispose();
		}

		/** Build vertices to represent an axis-aligned box centered on the local origin.
		 *
		 * @param p_hx the half-width.
		 * @param p_hy the half-height
		 */
		[Inline]
		final public function SetAsBox(p_hx:Number, p_hy:Number):void
		{
			m_count = 4;

			m_vertices[0] = -p_hx;
			m_vertices[1] = -p_hy;
			m_vertices[2] = p_hx;
			m_vertices[3] = -p_hy;
			m_vertices[4] = p_hx;
			m_vertices[5] = p_hy;
			m_vertices[6] = -p_hx;
			m_vertices[7] = p_hy;

			m_normals[0] = 0.0;
			m_normals[1] = -1.0;
			m_normals[2] = 1.0;
			m_normals[3] = 0.0;
			m_normals[4] = 0.0;
			m_normals[5] = 1.0;
			m_normals[6] = -1.0;
			m_normals[7] = 0.0;

			m_centroidX = 0;
			m_centroidY = 0;
		}

		/**
		 * Build vertices to represent an oriented box.
		 * @param p_hx the half-width.
		 * @param p_hy the half-height.
		 * @param p_centerX the center by X of the box in local coordinates.
		 * @param p_centerY the center by Y of the box in local coordinates.
		 * @param p_angle the rotation of the box in local coordinates.
		 */
		public function SetAsOrientedBox(p_hx:Number, p_hy:Number, p_centerX:Number, p_centerY:Number, p_angle:Number):void
		{
			SetAsBox(p_hx, p_hy);

			m_centroidX = p_centerX;
			m_centroidY = p_centerY;

			var sin:Number;
			var cos:Number;

			if (p_angle == 0.0)
			{
				sin = 0.0;
				cos = 1.0;
			}
			else
			{
				sin = Math.sin(p_angle);
				cos = Math.cos(p_angle);
			}

			var vecX:Number;
			var vecY:Number;
			var len:int = m_count * 2;

			for (var i:int = 0, i1:int = 1; i < len; i += 2, i1 += 2)
			{
				vecX = m_vertices[i];
				vecY = m_vertices[i1];

				m_vertices[i] = (cos * vecX - sin * vecY) + p_centerX;
				m_vertices[i1] = (sin * vecX + cos * vecY) + p_centerY;

				vecX = m_normals[i];
				vecY = m_normals[i1];

				m_normals[i] = (cos * vecX - sin * vecY);
				m_normals[i1] = (sin * vecX + cos * vecY);
			}
		}

		/**
		 *
		 * @param p_transform
		 * @param p_pointX
		 * @param p_pointY
		 * @return
		 */
		override public function TestPoint(p_transform:b2Mat22, p_pointX:Number, p_pointY:Number):Boolean
		{
			var rX:Number = p_pointX - p_transform.x;
			var rY:Number = p_pointY - p_transform.y;
			var cos:Number = p_transform.cos;
			var sin:Number = p_transform.sin;

			var localX:Number = cos * rX + sin * rY;
			var localY:Number = -sin * rX + cos * rY;

			for (var i:int = 0; i < m_count; i++)
			{
				var tvX:Number = b2Math.getX(m_vertices, i);
				var tvY:Number = b2Math.getY(m_vertices, i);

				var lX:Number = localX - tvX;
				var lY:Number = localY - tvY;

				var nX:Number = b2Math.getX(m_normals, i);
				var nY:Number = b2Math.getY(m_normals, i);

				var dot:Number = nX * lX + nY * lY;

				if (dot > 0.0)
				{
					return false;
				}
			}

			return true;
		}

		/**
		 *
		 * @param p_rayCastData
		 * @param p_transform
		 * @param p_childIndex
		 * @return
		 */
		override public function RayCast(p_rayCastData:b2RayCastData, p_transform:b2Mat22, p_childIndex:int):Boolean
		{
			// Put the ray into the polygon's frame of reference.
			var sX:Number = p_rayCastData.p1X - p_transform.x;
			var sY:Number = p_rayCastData.p1Y - p_transform.y;
			var eX:Number = p_rayCastData.p2X - p_transform.x;
			var eY:Number = p_rayCastData.p2Y - p_transform.y;
			var cos:Number = p_transform.cos;
			var sin:Number = p_transform.sin;

			var p1X:Number = cos * sX + sin * sY;
			var p1Y:Number = -sin * sX + cos * sY;

			var p2X:Number = cos * eX + sin * eY;
			var p2Y:Number = -sin * eX + cos * eY;

			var dX:Number = p2X - p1X;
			var dY:Number = p2Y - p1Y;

			var lower:Number = 0.0;
			var upper:Number = p_rayCastData.maxFraction;
			var index:int = -1;

			for (var i:int = 0; i < m_count; i++)
			{
				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0

				var viX:Number = b2Math.getX(m_vertices, i);
				var viY:Number = b2Math.getY(m_vertices, i);
				var niX:Number = b2Math.getX(m_normals, i);
				var niY:Number = b2Math.getY(m_normals, i);
				var rX:Number = viX - p1X;
				var rY:Number = viY - p1Y;

				var numerator:Number = niX * rX + niY * rY;
				var denominator:Number = niX * dX + niY * dY;

				if (denominator == 0.0)
				{
					if (numerator < 0.0)
					{
						return false;
					}
				}
				else
				{
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower > numerator.
					if (denominator < 0.0 && numerator < lower * denominator)
					{
						// Increase lower.
						// The segment enters this half-space.
						lower = numerator / denominator;
						index = i;
					}
					else if (denominator > 0.0 && numerator < upper * denominator)
					{
						// Decrease upper.
						// The segment exits this half-space.
						upper = numerator / denominator;
					}
				}

				// The use of epsilon here causes the assert on lower to trip
				// in some cases. Apparently the use of epsilon was to make edge
				// shapes work, but now those are handled separately.
				//if (upper < lower - b2_epsilon)
				if (upper < lower)
				{
					return false;
				}
			}

			CONFIG::debug
			{
				b2Assert(0.0 <= lower && lower <= p_rayCastData.maxFraction, "");
			}

			if (index >= 0)
			{
				p_rayCastData.fraction = lower;

				var normalX:Number = b2Math.getX(m_normals, index);
				var normalY:Number = b2Math.getY(m_normals, index);

				p_rayCastData.normalX = cos * normalX - sin * normalY;
				p_rayCastData.normalY = sin * normalX + cos * normalY;

				return true;
			}

			return false;
		}

		/**
		 *
		 * @param p_aabb
		 * @param p_transform
		 * @param p_childIndex
		 */
		override public function ComputeAABB(p_aabb:b2AABB, p_transform:b2Mat22, p_childIndex:int):void
		{
			var v0X:Number = b2Math.getX(m_vertices, 0);
			var v0Y:Number = b2Math.getY(m_vertices, 0);
			var cos:Number = p_transform.cos;
			var sin:Number = p_transform.sin;
			var tX:Number = p_transform.x;
			var tY:Number = p_transform.y;

			var lowerX:Number = (cos * v0X - sin * v0Y) + tX;
			var lowerY:Number = (sin * v0X + cos * v0Y) + tY;
			var upperX:Number = lowerX;
			var upperY:Number = lowerY;

			for (var i:int = 1; i < m_count; i++)
			{
				var viX:Number = b2Math.getX(m_vertices, i);
				var viY:Number = b2Math.getY(m_vertices, i);

				var vX:Number = (cos * viX - sin * viY) + tX;
				var vY:Number = (sin * viX + cos * viY) + tY;

				lowerX = b2Math.Min(lowerX, vX);
				lowerY = b2Math.Min(lowerY, vY);

				upperX = b2Math.Max(upperX, vX);
				upperY = b2Math.Max(upperY, vY);
			}

			p_aabb.lowerBoundX = lowerX - m_radius;
			p_aabb.lowerBoundY = lowerY - m_radius;
			p_aabb.upperBoundX = upperX + m_radius;
			p_aabb.upperBoundY = upperY + m_radius;
		}

		/**
		 *
		 * @param p_massData
		 * @param p_density
		 */
		override public function ComputeMass(p_massData:b2MassData, p_density:Number):void
		{
			// Polygon mass, centroid, and inertia.
			// Let rho be the polygon density in mass per unit area.
			// Then:
			// mass = rho * int(dA)
			// centroid.x = (1/mass) * rho * int(x * dA)
			// centroid.y = (1/mass) * rho * int(y * dA)
			// I = rho * int((x*x + y*y) * dA)
			//
			// We can compute these integrals by summing all the integrals
			// for each triangle of the polygon. To evaluate the integral
			// for a single triangle, we make a change of variables to
			// the (u,v) coordinates of the triangle:
			// x = x0 + e1x * u + e2x * v
			// y = y0 + e1y * u + e2y * v
			// where 0 <= u && 0 <= v && u + v <= 1.
			//
			// We integrate u from [0,1-v] and then v from [0,1].
			// We also need to use the Jacobian of the transformation:
			// D = cross(e1, e2)
			//
			// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
			//
			// The rest of the derivation is handled by computer algebra.

			CONFIG::debug
			{
				b2Assert(m_count > 2, "degenerate polygon");
			}

			var centerX:Number = 0;
			var centerY:Number = 0;
			var area:Number = 0;
			var I:Number = 0;

			// s is the reference point for forming triangles.
			// It's location doesn't change the result (except for rounding error).
			var sX:Number = 0;
			var sY:Number = 0;

			// This code would put the reference point inside the polygon.
			var mCount:int = m_count;
			for (var i:int = 0; i < mCount; i++)
			{
				sX += b2Math.getX(m_vertices, i);
				sY += b2Math.getY(m_vertices, i);
			}

			var inv:Number = 1.0 / mCount;

			sX *= inv;
			sY *= inv;

			for (i = 0; i < mCount; i++)
			{
				var e1X:Number = b2Math.getX(m_vertices, i) - sX;
				var e1Y:Number = b2Math.getY(m_vertices, i) - sY;

				var e2X:Number;
				var e2Y:Number;

				if ((i + 1) < mCount)
				{
					e2X = b2Math.getX(m_vertices, i + 1) - sX;
					e2Y = b2Math.getY(m_vertices, i + 1) - sY;
				}
				else
				{
					e2X = b2Math.getX(m_vertices, 0) - sX;
					e2Y = b2Math.getY(m_vertices, 0) - sY;
				}

				var D:Number = e1X * e2Y - e1Y * e2X;
				var triangleArea:Number = D * 0.5;
				area += triangleArea;

				centerX = triangleArea * b2Math.INV_3 * (e1X + e2X);
				centerY = triangleArea * b2Math.INV_3 * (e1Y + e2Y);

				var intx2:Number = e1X * e1X + e2X * e1X + e2X * e2X;
				var inty2:Number = e1Y * e1Y + e2Y * e1Y + e2Y * e2Y;

				I += (0.25 * b2Math.INV_3 * D) * (intx2 + inty2);
			}

			// Total mass
			p_massData.mass = p_density * area;

			CONFIG::debug
			{
				b2Assert(area > b2Math.EPSILON, "area too small: " + area);
			}

			var invArea:Number = 1.0 / area;

			centerX *= invArea;
			centerY *= invArea;

			var centerSX:Number = centerX + sX;
			var centerSY:Number = centerY + sY;

			p_massData.centerX = centerSX;
			p_massData.centerY = centerSY;

			// Shift to center of mass then to original body origin.
			var d1:Number = centerSX * centerSX + centerSY * centerSY;
			var d2:Number = centerX * centerX + centerY * centerY;

			// Inertia tensor relative to the local origin (point s).
			p_massData.I = p_density * I + p_massData.mass * (d1 - d2);
		}

		/**
		 * Get the vertex count
		 * @return int
		 */
		[Inline]
		final public function GetVertexCount():int
		{
			return m_count;
		}

		/**
		 * Return vertex by given index.
		 * @param p_index
		 * @return b2Vec2
		 * IMPORTANT! Produce new b2Vec2 instance.
		 */
		[Inline]
		final public function GetVertex(p_index:int):b2Vec2
		{
			CONFIG::debug
			{
				b2Assert((0 <= p_index && p_index < m_count), "index out of range");
			}

			return b2Vec2.Get(m_vertices[p_index], m_vertices[p_index + 1]);
		}

		/**
		 * Return X component of vertex.
		 * @param p_index - number of vertex.
		 * @return Number
		 */
		[Inline]
		final public function GetVertexX(p_index:int):Number
		{
			return m_vertices[p_index];
		}

		/**
		 * Return Y component of vertex.
		 * @param p_index - number of vertex.
		 * @return Number
		 */
		[Inline]
		final public function GetVertexY(p_index:int):Number
		{
			return m_vertices[p_index + 1];
		}

		/**
		 * Validate convexity. This is a very time consuming operation.
		 * @returns 'true' if valid
		 */
		final public function Validate():Boolean
		{
			var i1:int;
			var i2:int;
			var pX:Number;
			var pY:Number;
			var eX:Number;
			var eY:Number;
			var vX:Number;
			var vY:Number;
			var c:Number;
			var mCount:int = m_count;

			for (var i:int = 0; i < mCount; i++)
			{
				i1 = i;
				i2 = (i < mCount - 1) ? i1 + 1 : 0;

				pX = b2Math.getX(m_vertices, i1);
				pY = b2Math.getY(m_vertices, i1);

				eX = b2Math.getX(m_vertices, i2) - pX;
				eY = b2Math.getY(m_vertices, i2) - pY;

				for (var j:int = 0; j < mCount; j++)
				{
					if (j == i1 || j == i2)
					{
						continue;
					}

					vX = b2Math.getX(m_vertices, j) - pX;
					vY = b2Math.getY(m_vertices, j) - pY;
					c = eX * vY - eY * vX;

					if (c < 0.0)
					{
						return false;
					}
				}
			}

			return true;
		}

		/**
		 */
		override public function GetChildCount():int
		{
			return 1;
		}

		/**
		 * Returns copy of current shape.
		 * @return IDisposable (actually b2PolygonShape)
		 */
		override public function Clone():IDisposable
		{
			var polygon:b2PolygonShape = Get();
			polygon.m_count = m_count;
			polygon.m_radius = m_radius;
			polygon.m_centroidX = m_centroidX;
			polygon.m_centroidY = m_centroidY;

			for (var i:int = 0; i < m_count; i++)
			{
				b2Math.setXY(b2Math.getX(m_vertices, i), b2Math.getY(m_vertices, i), polygon.m_vertices, i);
				b2Math.setXY(b2Math.getX(m_normals, i), b2Math.getY(m_normals, i), polygon.m_normals, i);
			}

			return polygon;
		}

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			m_normals.length = 0;
			m_vertices.length = 0;
			m_count = 0;

			b2Disposable.Put(this, classId);
		}

		/**
		 * Computes the centroid of the given polygon
		 * @param    p_vs vector of Number specifying a polygon. Every two following numbers represents x and y.
		 * @param    p_count    length of vs
		 * @return the polygon centroid.
		 * IMPORTANT! Method produces new instance of b2SPoint as return value, so it is need manually to dispose it for free memory.
		 */
		static public function ComputeCentroid(p_vs:Vector.<Number>, p_count:int):b2SPoint
		{
			CONFIG::debug
			{
				b2Assert(p_count >= 3, "Given vertices count is " + p_count + ". Polygon should has at least 3 vertices");
			}

			var cX:Number = 0;
			var cY:Number = 0;
			var area:Number = 0;
			var inv3:Number = 1.0 / 3.0;
			var p1X:Number = 0;
			var p1Y:Number = 0;
			var p2X:Number;
			var p2Y:Number;
			var p3X:Number;
			var p3Y:Number;
			var e1X:Number;
			var e1Y:Number;
			var e2X:Number;
			var e2Y:Number;
			var D:Number;
			var triangleArea:Number;
			var temp:Number;
			var j:int = 0;

			for (var i:int = 0; i < p_count; i++)
			{
				j = i * 2;

				p2X = p_vs[j];
				p2Y = p_vs[j + 1];

				if ((i + 1) < p_count)
				{
					p3X = p_vs[j + 2];
					p3Y = p_vs[j + 3];
				}
				else
				{
					p3X = p_vs[0];
					p3Y = p_vs[1];
				}

				e1X = p2X - p1X;
				e1Y = p2Y - p1Y;

				e2X = p3X - p1X;
				e2Y = p3Y - p1Y;

				D = e1X * e2Y - e1Y * e2X;

				triangleArea = D * 0.5;
				area += triangleArea;

				// Area weighted centroid
				temp = triangleArea * inv3;
				cX += temp * (p1X + p2X + p3X);
				cY += temp * (p1Y + p2Y + p3Y);
			}

			// Centroid
			CONFIG::debug
			{
				b2Assert(area > b2Math.EPSILON, "area too small. Area: " + area);
			}

			temp = 1.0 / area;
			cX *= temp;
			cY *= temp;

			return b2SPoint.Get(cX, cY);
		}

		/**
		 * Returns new instance of b2PolygonShape.
		 * @return b2PolygonShape
		 */
		static public function Get():b2PolygonShape
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var polygon:b2PolygonShape;

			if (instance) polygon = instance as b2PolygonShape;
			else polygon = new b2PolygonShape();

			return polygon;
		}
	}
}
