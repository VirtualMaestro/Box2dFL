/**
 * User: VirtualMaestro
 * Date: 29.11.2014
 * Time: 23:21
 */
package Box2D.Collision.Manifold
{
	import Box2D.Common.b2Settings;

	/**
	* A manifold for two touching convex shapes.
	* Box2D supports multiple types of contact:
	* - clip point versus plane with radius
	* - point versus point with radius (circles)
	* The local point usage depends on the manifold type:
	* -CIRCLES: the local center of circleA
	* -FACE_A: the center of faceA
	* -FACE_B: the center of faceB
	* Similarly the local normal usage:
	* -CIRCLES: not used
	* -FACE_A: the normal on polygonA
	* -FACE_B: the normal on polygonB
	* We store contacts in this way so that position correction can
	* account for movement, which is critical for continuous physics.
	* All contact scenarios must be expressed in one of these types.
	* This structure is stored across time steps, so we keep it small.
	*/
	public class b2Manifold
	{
		// Manifold types
		static public const CIRCLES:int = 0x0001;
		static public const FACE_A:int = 0x0002;
		static public const FACE_B:int = 0x0004;

		/**
		 * Manifold type.
		 */
		public var type:int;
		public var localNormalX:Number;
		public var localNormalY:Number;
		public var localPointX:Number;
		public var localPointY:Number;

		/**
		 * The number of manifold points.
		 */
		public var pointCount:int;

		/**
		 * The points of contact.
		 * Num of points controlled by b2Settings.maxManifoldPoints parameter.
		 * For now value for maxManifoldPoints is 2.
		 */
		public var points:Vector.<b2ManifoldPoint>;

		/**
		 */
		public function b2Manifold()
		{
			points = new Vector.<b2ManifoldPoint>(b2Settings.maxManifoldPoints, true);
		}
	}
}
