/**
 * User: VirtualMaestro
 * Date: 03.01.2015
 * Time: 21:55
 */
package Box2D.Collision.Structures
{
	/**
	 * Reference face used for clipping.
	 */
	public class b2ReferenceFace
	{
		public var i1:int;
		public var i2:int;

		public var v1X:Number;
		public var v1Y:Number;

		public var v2X:Number;
		public var v2Y:Number;

		public var normalX:Number;
		public var normalY:Number;

		public var sideNormal1X:Number;
		public var sideNormal1Y:Number;

		public var sideNormal2X:Number;
		public var sideNormal2Y:Number;

		public var sideOffset1:Number;
		public var sideOffset2:Number;
	}
}
