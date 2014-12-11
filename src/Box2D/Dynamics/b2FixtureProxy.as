/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.b2AABB;

	/**
	 * This proxy is used internally to connect fixtures to the broad-phase
	 * TODO: Think about pooling this
	 */
	internal class b2FixtureProxy
	{
		public var aabb:b2AABB;
		public var fixture:b2Fixture;
		public var childIndex:int;
		public var proxyId:int = -1;
	}
}
