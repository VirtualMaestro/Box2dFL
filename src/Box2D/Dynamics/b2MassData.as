/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 23:45
 */
package Box2D.Dynamics
{
	/**
	 * This holds the mass data computed for a shape.
	 */
	public class b2MassData
	{
		/**
		 * The mass of the shape, usually in kilograms.
		 */
		public var mass:Number = 0;

		/**
		 * The position of the shape's centroid relative to the shape's origin.
		 */
		public var centerX:Number = 0;

		/**
		 * The position of the shape's centroid relative to the shape's origin.
		 */
		public var centerY:Number = 0;

		/**
		 * The rotational inertia of the shape about the local origin.
		 */
		public var I:Number = 0;
	}
}
