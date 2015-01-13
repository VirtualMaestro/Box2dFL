/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 16:35
 */
package Box2D.Collision.Shapes
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * A chain shape is a free form sequence of line segments.
	 * The chain has two-sided collision, so you can use inside and outside collision.
	 * Therefore, you may use any winding order.
	 * Since there may be many vertices, they are allocated using b2Alloc.
	 * Connectivity information is used to create smooth collisions.
	 * WARNING: The chain will not collide properly if there are self-intersections.
	 *
	 * TODO:
	 */
	public class b2ChainShape extends b2Shape
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		/**
		 * The vertices. Owned by this class.
		 * Every two elements are x and y.
		 */
		b2internal var m_vertices:Vector.<Number>;

		/**
		 * The vertex count.
		 */
		b2internal var m_count:int;

		b2internal var m_prevVertexX:Number;
		b2internal var m_prevVertexY:Number;
		b2internal var m_nextVertexX:Number;
		b2internal var m_nextVertexY:Number;
		b2internal var m_hasPrevVertex:Boolean;
		b2internal var m_hasNextVertex:Boolean;

		public function b2ChainShape()
		{
			m_type = b2Shape.CHAIN;
			m_radius = b2Settings.polygonRadius;

			init();
		}

		/**
		 */
		[Inline]
		final private function init():void
		{
			m_count = 0;
			m_hasPrevVertex = false;
			m_hasNextVertex = false;
		}

		/**
		 * Get a child edge.
		 * TODO:
		 */
		public function GetChildEdge(p_edge:b2EdgeShape, p_index:int):void
		{

		}

		/**\
		 */
		override public function GetChildCount():int
		{
			// edge count = vertex count - 1
			return m_count - 1;
		}

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			m_vertices = null;

			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns new instance of b2ChainShape.
		 * @return b2ChainShape
		 */
		static public function Get():b2ChainShape
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var chain:b2ChainShape;

			if (instance)
			{
				chain = instance as b2ChainShape;
				chain.init();
			}
			else chain = new b2ChainShape();

			return chain;
		}
	}
}
