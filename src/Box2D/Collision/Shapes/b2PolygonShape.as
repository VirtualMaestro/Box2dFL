/**
 * User: VirtualMaestro
 * Date: 03.12.2014
 * Time: 21:55
 */
package Box2D.Collision.Shapes
{
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2RayCastData;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;

	CONFIG::debug
	{
		import Box2D.assert;
	}

	use namespace b2internal;

	/**
	 * A convex polygon. It is assumed that the interior of the polygon is to
	 * the left of each edge.
	 * Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
	 * In most cases you should not need many vertices for a convex polygon.
	 *
	 * TODO: Implement
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
		b2internal var m_vertexCount:int;

		/**
		 */
		public function b2PolygonShape()
		{
			m_type = b2Shape.POLYGON;
			m_centroidX = 0;
			m_centroidY = 0;
			m_vertexCount = 0;
			m_vertices = new <Number>[];
			m_normals = new <Number>[];
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
				assert(p_count >= 3 && p_count <= b2Settings.maxPolygonVertices, "unacceptable count of polygon vertices");
			}

			if (p_count < 3)
			{
				SetAsBox(1.0, 1.0);
				return;
			}

			var n:int = b2Math.Min(p_count, b2Settings.maxPolygonVertices);


		}

		/** Build vertices to represent an axis-aligned box centered on the local origin.
		 *
		 * @param p_hx the half-width.
		 * @param p_hy the half-height
		 */
		[Inline]
		final public function SetAsBox(p_hx:Number, p_hy:Number):void
		{
			m_vertexCount = 4;

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

			var sin:Number = Math.sin(p_angle);
			var cos:Number = Math.cos(p_angle);

			var vecX:Number;
			var vecY:Number;
			var len:int = m_vertexCount*2;

			for (var i:int=0, i1:int=1; i < len; i+=2, i1+=2)
			{
				vecX = m_vertices[i];
				vecY = m_vertices[i1];

				m_vertices[i]   = (cos * vecX - sin * vecY) + p_centerX;
				m_vertices[i1] = (sin * vecX + cos * vecY) + p_centerY;

				vecX = m_normals[i];
				vecY = m_normals[i1];

				m_normals[i]   = (cos * vecX - sin * vecY);
				m_normals[i1]  = (sin * vecX + cos * vecY);
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
			return false;
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

		}

		/**
		 *
		 * @param p_massData
		 * @param p_density
		 */
		override public function ComputeMass(p_massData:b2MassData, p_density:Number):void
		{

		}

		/**
		 * Get the vertex count
		 * @return int
		 */
		[Inline]
		final public function GetVertexCount():int
		{
			return m_vertexCount;
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
				assert((0 <= p_index && p_index < m_vertexCount), "index out of range");
			}

			return b2Vec2.Get(m_vertices[p_index], m_vertices[p_index+1]);
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
			return m_vertices[p_index+1];
		}

		/**
		 * Validate convexity. This is a very time consuming operation.
		 * @returns 'true' if valid
 		 */
		final public function Validate():Boolean
		{
			return false;
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
			return Get();
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
			m_vertexCount = 0;

			b2Disposable.Put(this, classId);
		}

		/**
		 * Computes the centroid of the given polygon
		 * @param	p_vs vector of Number specifying a polygon. Every two following numbers represents x and y.
		 * @param	p_count	length of vs
		 * @return the polygon centroid.
		 * IMPORTANT! Method produces new instance of b2Vec2 as return value, so it is need manually to dispose it for free memory.
		 */
		static public function ComputeCentroid(p_vs:Vector.<Number>, p_count:int):b2Vec2
		{
			CONFIG::debug
			{
				assert(p_count >= 3, "Given vertices count is " + p_count + ". Polygon should has at least 3 vertices");
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
				j = i*2;

				p2X = p_vs[j];
				p2Y = p_vs[j+1];

				if ((i+1) < p_count)
				{
					p3X = p_vs[j+2];
					p3Y = p_vs[j+3];
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
				assert(area > b2Math.EPSILON, "area too small. Area: " + area);
			}

			temp = 1.0/area;
			cX *= temp;
			cY *= temp;

			return b2Vec2.Get(cX, cY);
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
			// TODO:

			return polygon;
		}
	}
}
