/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 16:03
 */
package Box2D.Collision
{
	/**
	 * This structure is used to keep track of the best separating axis.
	 */
	public class b2EPAxis
	{
		static public const e_unknown:int = 0;
		static public const e_edgeA:int = 0;
		static public const e_edgeB:int = 0;

		//
		public var type:int;
		public var index:int;
		public var separation:Number;
	}
}
