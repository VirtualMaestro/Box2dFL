/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	import Box2D.Collision.b2Collision;
	import Box2D.Common.b2internal;
	import Box2D.b2Assert;

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

		b2internal var m_tree:b2DynamicTree;

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

		/**
		 * Create a proxy with an initial AABB. Pairs are not reported until
		 * UpdatePairs is called.
 		 */
		public function CreateProxy(p_aabb:b2AABB, p_userData:*):int
		{
			return 0; // TODO:
		}

		/**
		 * Call MoveProxy as many times as you like, then when you are done
		 * call UpdatePairs to finalized the proxy pairs (for your time step).
		 * @param p_proxyId
		 * @param p_aabb
		 * @param p_displacementX
		 * @param p_displacementY
		 * TODO
		 */
		public function MoveProxy(p_proxyId:int, p_aabb:b2AABB, p_displacementX:Number, p_displacementY:Number):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * Destroy a proxy. It is up to the client to remove any pairs.
		 * @param p_proxyId
		 * TODO
		 */
		public function DestroyProxy(p_proxyId:int):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
		 * @param p_proxyId
		 * TODO
		 */
		public function TouchProxy(p_proxyId:int):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * Update the pairs. This results in pair callbacks. This can only add pairs.
		 * @param p_callback
		 * TODO
		 */
		public function UpdatePairs(p_callback:*):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * Test overlap of fat AABBs.
		 * @param p_proxyIdA
		 * @param p_proxyIdB
		 * @return
		 */
		public function TestOverlap(p_proxyIdA:int, p_proxyIdB:int):Boolean
		{
			var aabbA:b2AABB = m_tree.GetFatAABB(p_proxyIdA);
			var aabbB:b2AABB = m_tree.GetFatAABB(p_proxyIdB);

			return b2Collision.b2TestOverlapAABB(aabbA, aabbB);
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