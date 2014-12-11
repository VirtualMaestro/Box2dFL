/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
	 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
	 * It is up to the client to consume the new pairs and to track subsequent overlap.
	 * TODO: Impl
	 */
	public class b2BroadPhase
	{
		static b2internal const e_nullProxy:int = -1;

		b2internal var m_tree/*:b2DynamicTree*/;

		b2internal var m_proxyCount:int;

		b2internal var m_moveBuffer:Vector.<int>;
		b2internal var m_moveCapacity:int;
		b2internal var m_moveCount:int;

		b2internal var m_pairBuffer:Vector.<b2Pair>;
		b2internal var m_pairCapacity:int;
		b2internal var m_pairCount:int;

		b2internal var m_queryProxyId:int;


		public function b2BroadPhase()
		{
			m_proxyCount = 0;

			m_pairCapacity = 16;
			m_pairCount = 0;
			m_pairBuffer = new Vector.<b2Pair>(m_pairCapacity);

			m_moveCapacity = 16;
			m_moveCount = 0;
			m_moveBuffer = new Vector.<int>(m_moveCapacity);
		}


	}
}

/**
 */
internal class b2Pair
{
	public var proxyIdA:int;
	public var proxyIdB:int;
}