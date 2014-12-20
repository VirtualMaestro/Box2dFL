/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 16:04
 */
package Box2D.Common
{
	/**
	 * Global tuning constants based on meters-kilograms-seconds (MKS) units.
	 */
	public class b2Settings
	{
		/**
		 * The maximum number of contact points between two convex shapes.
		 * Do not change this value.
 		 */
		static public const maxManifoldPoints:int = 2;

		/**
		 * The maximum number of vertices on a convex polygon.
		 * You cannot increase this too much because b2BlockAllocator has a maximum object size.
		 */
		static public const maxPolygonVertices:int = 8;

		/**
		 * A small length used as a collision and constraint tolerance.
		 * Usually it is chosen to be numerically significant, but visually insignificant.
		 */
		static public const linearSlop:Number = 0.005;

		/**
		 * The radius of the polygon/edge shape skin. This should not be modified.
		 * Making this smaller means polygons will have an insufficient buffer for continuous collision.
		 * Making it larger may create artifacts for vertex collision.
 		 */
		static public const polygonRadius:Number = 2.0 * linearSlop;

		/**
		 * This is used to fatten AABBs in the dynamic tree.
		 * This allows proxies to move by a small amount without triggering a tree adjustment.
		 * This is in meters.
		 */
		static public const aabbExtension:Number = 0.1;

		/**
		 * This is used to fatten AABBs in the dynamic tree.
		 * This is used to predict the future position based on the current displacement.
		 * This is a dimensionless multiplier.
 		 */
		static public const aabbMultiplier:Number = 2.0;
	}
}
