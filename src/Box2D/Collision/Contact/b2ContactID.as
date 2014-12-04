/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 15:38
 */
package Box2D.Collision.Contact
{
	/**
	 * Contact ids to facilitate warm starting.
	 */
	public class b2ContactID
	{
		/**
		 * Used to quickly compare contact ids
		 */
		public var key:uint;

		/**********************
		 * Following functionality merged from structure 'b2ContactFeature'.
		 * The features that intersect to form the contact point
		 * This must be 4 bytes or less.
		 **********************/

		/**
		 * Vertex contact feature type.
 		 */
		static public const VERTEX_CF_TYPE:int = 0;

		/**
		 * Face contact feature type.
 		 */
		static public const FACE_CF_TYPE:int = 1;

		/**
		 * Feature index on shapeA
 		 */
		public var indexA:uint;

		/**
		 * Feature index on shapeB
 		 */
		public var indexB:uint;

		/**
		 * The feature type on shapeA
		 */
		public var typeA:uint;

		/**
		 * The feature type on shapeB
		 */
		public var typeB:uint;
	}
}
