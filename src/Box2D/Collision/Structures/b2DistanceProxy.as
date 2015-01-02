/**
 * User: VirtualMaestro
 * Date: 22.12.2014
 * Time: 22:33
 */
package Box2D.Collision.Structures
{
	import Box2D.Collision.Shapes.b2ChainShape;
	import Box2D.Collision.Shapes.b2CircleShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 * A distance proxy is used by the GJK algorithm.
	 * It encapsulates any shape.
	 */
	public class b2DistanceProxy
	{
		/**
		 * Every two elements are x & y.
		 */
		public var m_buffer:Vector.<Number>;

		/**
		 * Every two elements are x & y.
		 */
		public var m_vertices:Vector.<Number>;

		/**
		 * vertex count
		 */
		public var m_count:int;

		/**
		 */
		public var m_radius:Number;

		/**
		 */
		public function b2DistanceProxy()
		{
			m_buffer = new <Number>[];
			m_vertices = new <Number>[];
			m_count = 0;
			m_radius = 0;
		}

		/**
		 * Initialize the proxy using the given shape.
		 * The shape must remain in scope while the proxy is in use.
		 */
		final public function Set(p_shape:b2Shape, p_index:int):void
		{
			switch (p_shape.GetType)
			{
				case b2Shape.CIRCLE:
				{
					var circle:b2CircleShape = p_shape as b2CircleShape;
					m_vertices[0] = circle.m_pX;
					m_vertices[1] = circle.m_pY;

					m_count = 1;
					m_radius = circle.m_radius;

					break;
				}

				case b2Shape.POLYGON:
				{
					var polygon:b2PolygonShape = p_shape as b2PolygonShape;
					m_vertices = polygon.m_vertices;
					m_count = polygon.m_count;
					m_radius = polygon.m_radius;

					break;
				}

				case b2Shape.CHAIN:
				{
					var chain:b2ChainShape = p_shape as b2ChainShape;

					CONFIG::debug
					{
						b2Assert(0 <= p_index && p_index < chain.m_count, "incorrect index");
					}

					var chainVertices:Vector.<Number> = chain.m_vertices;
					m_buffer[0] = b2Math.getX(chainVertices, 0);
					m_buffer[1] = b2Math.getY(chainVertices, 0);

					if (p_index + 1 < chain.m_count)
					{
						m_buffer[2] = b2Math.getX(chainVertices, p_index+1);
						m_buffer[3] = b2Math.getY(chainVertices, p_index+1);
					}
					else
					{
						m_buffer[2] = b2Math.getX(chainVertices, 0);
						m_buffer[3] = b2Math.getY(chainVertices, 0);
					}

					m_vertices = m_buffer;  // TODO: Clarify
					m_count = 2;
					m_radius = chain.m_radius;

					break;
				}

				case b2Shape.EDGE:
				{
					var edge:b2EdgeShape = p_shape as b2EdgeShape;
					m_vertices[0] = edge.m_vertex1X;
					m_vertices[1] = edge.m_vertex1Y;
					m_count = 2; // TODO: Check, maybe bug
					m_radius = edge.m_radius;

					break;
				}

				default:
					b2Assert(false, "")
			}
		}

		/**
		 * Get the supporting vertex index in the given direction.
		 * @return int
		 */
		[Inline]
		final public function GetSupport(p_dX:Number, p_dY:Number):int
		{
		    var bestIndex:int = 0;
			var vX:Number = b2Math.getX(m_vertices, 0);
			var vY:Number = b2Math.getY(m_vertices, 0);

			var bestValue:Number = vX * p_dX + vY * p_dY;
			var value:Number;

			for (var i:int = 1; i < m_count; i++)
			{
				vX = b2Math.getX(m_vertices, i);
				vY = b2Math.getY(m_vertices, i);

				value = vX * p_dX + vY * p_dY;

				if (value > bestValue)
				{
					bestIndex = i;
					bestValue = value;
				}
			}

			return bestIndex;
		}

		/**
		 * Get the supporting vertex in the given direction.
		 * @return new instance of b2Vec2.
		 */
		[Inline]
		final public function GetSupportVertex(p_dX:Number, p_dY:Number):b2Vec2
		{
			var bestIndex:int = GetSupport(p_dX, p_dY);
			return b2Vec2.Get(b2Math.getX(m_vertices, bestIndex), b2Math.getY(m_vertices, bestIndex));
		}

		/**
		 * Get a vertex by index. Used by b2Distance.
		 * @param p_index
		 * @return return new instance of b2Vec2
		 */
		[Inline]
		final public function GetVertex(p_index:int):b2Vec2
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_index && p_index < m_count, "incorrect index");
			}

			return b2Vec2.Get(b2Math.getX(m_vertices, p_index), b2Math.getY(m_vertices, p_index));
		}

		/**
		 * Get a vertex by index. Used by b2Distance.
		 * @param p_index
		 * @return  X component of vertex.
		 */
		[Inline]
		final public function GetVertexX(p_index:int):Number
		{
			return b2Math.getX(m_vertices, p_index);
		}

			/**
		 * Get a vertex by index. Used by b2Distance.
		 * @param p_index
		 * @return  Y component of vertex.
		 */
		[Inline]
		final public function GetVertexY(p_index:int):Number
		{
			return b2Math.getY(m_vertices, p_index);
		}
	}
}
