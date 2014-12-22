/**
 * User: VirtualMaestro
 * Date: 22.12.2014
 * Time: 23:31
 */
package Box2D.Collision.Structures
{
	/**
	 * Used to warm start b2Distance.
	 * Set count to zero on first call.
	 */
	public class b2SimplexCache
	{
		public var metric:Number;  //< length or area
		public var count:uint;
		public var indexA:Vector.<uint>;  //< vertices on shape A
		public var indexB:Vector.<uint>;  //< vertices on shape B

		public function b2SimplexCache()
		{
			indexA = new <uint>[];
			indexB = new <uint>[];
			count = 0;
		}
	}
}
