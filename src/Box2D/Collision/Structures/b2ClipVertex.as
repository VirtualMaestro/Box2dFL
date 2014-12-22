/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:05
 */
package Box2D.Collision.Structures
{
	import Box2D.Collision.Contact.b2ContactID;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Used for computing contact manifolds.
	 */
	public class b2ClipVertex extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		public var vX:Number;
		public var vY:Number;
		public var id:b2ContactID;

		/**
		 */
		public function b2ClipVertex()
		{
			id = new b2ContactID();
		}

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns new instance of b2ClipVertex.
		 * @return Box2D.Collision.Structures.b2ClipVertex
		 */
		static public function Get():b2ClipVertex
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var clipVertex:b2ClipVertex;

			if (instance)
			{
				clipVertex = instance as b2ClipVertex;
			}
			else clipVertex = new b2ClipVertex();

			return clipVertex;
		}
	}
}
