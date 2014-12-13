/**
 * User: VirtualMaestro
 * Date: 26.11.2014
 * Time: 1:18
 */
package Box2D.Collision.Shapes
{
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2RayCastData;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;

	use namespace b2internal;

	/**
	 * Abstract base class for all shapes.
	 */
	public class b2Shape extends b2Disposable
	{
		/**
		 * Shape type circle.
		 */
		static public const CIRCLE:int = 0;

		/**
		 * Shape type edge.
		 */
		static public const EDGE:int = 1;

		/**
		 * Shape type polygon.
		 */
		static public const POLYGON:int = 2;
		static public const CHAIN:int = 3;
		static public const TYPE_COUNT:int = 4;

		//
		b2internal var m_radius:int = 0;

		//
		b2internal var m_type:int = 0;

		/**
		 * Init instance with given one.
		 */
		public function SetTo(p_shape:b2Shape):void
		{
			// override in children
		}

		/**
		 * Get the number of child primitives.
		 * @return
		 */
		public function GetChildCount():int
		{
			return 0;
		}

		/**
		 * Return type of shape - static const CIRCLE, POLYGON and so on.
		 * @return
		 */
		[Inline]
		final public function GetType():int
		{
			return m_type;
		}

		/**
		 * Test a point for containment in this shape. This only works for convex shapes.
		 * @param p_transform the shape matrix.
		 * @param p_pointX - point in world coordinates.
		 * @param p_pointY - point in world coordinates.
		 * @return
		 */
		public function TestPoint(p_transform:b2Mat22, p_pointX:Number, p_pointY:Number):Boolean
		{
			return false;
		}

		/** Cast a ray against a child shape.
		 *  @param p_rayCastData the ray-cast input/output data.
		 *  @param p_transform the matrix to be applied to the shape (just rotation).
		 *  @param p_childIndex the child shape index
		 */
		public function RayCast(p_rayCastData:b2RayCastData, p_transform:b2Mat22, p_childIndex:int):Boolean
		{
	        // override in children
			return false;
		}

		/** Given a transform, compute the associated axis aligned bounding box for a child shape.
		 *  @param p_aabb returns the axis aligned box.
		 *  @param p_transform the world transform matrix of the shape.
		 *  @param p_childIndex the child shape
		 */
		public function ComputeAABB(p_aabb:b2AABB, p_transform:b2Mat22, p_childIndex:int):void
		{
			 // override in children
		}

		/** Compute the mass properties of this shape using its dimensions and density.
		 *  The inertia tensor is computed about the local origin.
		 *  @param p_massData - returns the mass data for this shape.
		 *  @param p_density - the density in kilograms per meter squared.
		 */
		public function ComputeMass(p_massData:b2MassData, p_density:Number):void
		{
			// override in children

		}
	}
}
