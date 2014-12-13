/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 16:09
 */
package Box2D.Collision
{
	import Box2D.Common.b2Settings;

	/**
	 * This holds polygon B expressed in frame A.
	 */
	public class b2TempPolygon
	{
		/**
		 * Every two following elements are X and Y.
		 */
		public var vertices:Vector.<Number>;

		/**
		 * Every two following elements are X and Y.
		 */
		public var normals:Vector.<Number>;

		/**
		 */
		public var count:int;

		/**
		 */
		public function b2TempPolygon()
		{
			vertices = new Vector.<Number>(b2Settings.maxPolygonVertices);
			normals = new Vector.<Number>(b2Settings.maxPolygonVertices);
		}
	}
}
