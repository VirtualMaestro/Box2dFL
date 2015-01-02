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
		 * NOTE: Following functionality merged from structure 'b2ContactFeature'.
		 *
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

		/**
		 * Initialize instance by given one.
		 * (Copying values from given structure to this one)
		 */
		public function Set(p_contactId:b2ContactID):void
		{
			key = p_contactId.key;
			indexA = p_contactId.indexA;
			indexB = p_contactId.indexB;
			typeA = p_contactId.typeA;
			typeB = p_contactId.typeB;
		}

		/**
		 * Swap the Contact Feature:
		 * indexA = indexB;
		 * indexB = indexA;
		 * typeA = typeB;
		 * typeB = typeA;
		 */
		[Inline]
		final public function Swap():void
		{
			var temp:uint = indexA;
			indexA = indexB;
			indexB = temp;

			temp = typeA;
			typeA = typeB;
			typeB = temp;
		}
	}
}
