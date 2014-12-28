/**
 * User: VirtualMaestro
 * Date: 23.12.2014
 * Time: 0:16
 */
package Box2D.Collision.Structures
{
	/**
	 */
	public class b2SimplexVertex
	{
		/**support point in proxyA*/
		public var wAX:Number;
		public var wAY:Number;

		/**support point in proxyB*/
		public var wBX:Number;
		public var wBY:Number;

		/** wB - wA*/
		public var wX:Number;
		public var wY:Number;

		/**barycentric coordinate for closest point*/
		public var a:Number;

		/** wA index*/
		public var indexA:int;
		/** wB index*/
		public var indexB:int;

		/**
		 */
		public function Set(p_simpleVertex:b2SimplexVertex):void
		{
			wAX = p_simpleVertex.wAX;
			wAY = p_simpleVertex.wAY;

			wBX = p_simpleVertex.wBX;
			wBY = p_simpleVertex.wBY;

			wX = p_simpleVertex.wX;
			wY = p_simpleVertex.wY;

			a = p_simpleVertex.a;

			indexA = p_simpleVertex.indexA;
			indexB = p_simpleVertex.indexB;
		}
	}
}
