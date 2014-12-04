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
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;
	import Box2D.assert;

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
		public function Set(p_points:b2Vec2, p_count:int):void
		{

		}

		/** Build vertices to represent an axis-aligned box centered on the local origin.
		 *
		 * @param p_hx the half-width.
		 * @param p_hy the half-height
		 */
		public function SetAsBox(p_hx:Number, p_hy:Number):void
		{

		}

		/** Build vertices to represent an oriented box.
		 * @param p_hx the half-width.
		 * @param p_hy the half-height.
		 * @param p_centerX the center by X of the box in local coordinates.
		 * @param p_centerY the center by Y of the box in local coordinates.
		 * @param p_angle the rotation of the box in local coordinates.
		*/
		public function SetAsOrientedBox(p_hx:Number, p_hy:Number, p_centerX:Number, p_centerY:Number, p_angle:Number):void
		{

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
				assert((0 <= p_index && p_index < m_vertexCount), "index out of range", "b2PolygonShape.GetVertex");
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
			return b2Shape.GetPolygon();
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

			b2Shape.Put(this);
		}
	}
}
