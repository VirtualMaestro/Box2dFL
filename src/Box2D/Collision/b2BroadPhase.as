/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	import Box2D.Collision.Structures.b2RayCastData;
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
		 */
		public function UpdatePairs(p_callback:*):void
		{
			// Reset pair buffer
			m_pairCount = 0;

			var locMoveCount:int = m_moveCount;

			// Perform tree queries for all moving proxies.
			for (var i:int = 0; i < locMoveCount; ++i)
			{
				m_queryProxyId = m_moveBuffer[i];
				if (m_queryProxyId == e_nullProxy)
				{
					continue;
				}

				// We have to query the tree with the fat AABB so that
				// we don't fail to create a pair that may touch later.
				var fatAABB:b2AABB = m_tree.GetFatAABB(m_queryProxyId);

				// Query tree, create pairs and add them pair buffer.
				m_tree.Query(this, fatAABB);
			}

			// Reset move buffer
			m_moveCount = 0;

			// Sort the pair buffer to expose duplicates.
			m_pairBuffer.sort(b2PairLessThan);

			// Send the pairs back to the client.
			var primaryPairList:Vector.<b2Pair> = m_pairBuffer;
			var primaryPair:b2Pair;
			var pair:b2Pair;
			var userDataA:*;
			var userDataB:*;

			i = 0;

			while (i < m_pairCount)
			{
				primaryPair = primaryPairList[i];

				userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
				userDataB = m_tree.GetUserData(primaryPair.proxyIdB);

				p_callback.AddPair(userDataA, userDataB);
				++i;

				// Skip any duplicate pairs.
				while (i < m_pairCount)
				{
					pair = primaryPairList[i];

					if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
					{
						break;
					}

					++i;
				}
			}

			// Try to keep the tree balanced.
			//m_tree.Rebalance(4);
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

		/**
		 * This is used to sort pairs.
		 * @param p_pair1
		 * @param p_pair2
		 * @return
		 */
		[Inline]
		final public function b2PairLessThan(p_pair1:b2Pair, p_pair2:b2Pair):Boolean
		{
			var p1A:int = p_pair1.proxyIdA;
			var p2A:int = p_pair2.proxyIdA;

			if (p1A < p2A)
			{
				return true;
			}

			if (p1A == p2A)
			{
				return p_pair1.proxyIdB < p_pair2.proxyIdB;
			}

			return false;
		}

		/**
		 */
		[Inline]
		final public function Query(p_callback:*, p_aabb:b2AABB):void
		{
			m_tree.Query(p_callback, p_aabb);
		}

		/**
		 */
		[Inline]
		final public function RayCast(p_callback:*, p_input:b2RayCastData):void
		{
			m_tree.RayCast(p_callback, p_input);
		}

		/**
		 */
		[Inline]
		final public function ShiftOrigin(p_newOriginX:Number, p_newOriginY:Number):void
		{
			m_tree.ShiftOrigin(p_newOriginX, p_newOriginY);
		}

		/**
		 */
		[Inline]
		final public function GetUserData(p_proxyId:int):*
		{
			return m_tree.GetUserData(p_proxyId);
		}

		/**
		 */
		[Inline]
		final public function GetFatAABB(p_proxyId:int):b2AABB
		{
			return m_tree.GetFatAABB(p_proxyId);
		}

		/**
		 */
		[Inline]
		final public function GetProxyCount():int
		{
			return m_proxyCount;
		}

		/**
		 */
		[Inline]
		final public function GetTreeHeight():int
		{
			return m_tree.GetHeight();
		}

		/**
		 */
		[Inline]
		final public function GetTreeBalance():int
		{
			return m_tree.GetMaxBalance();
		}

		/**
		 */
		[Inline]
		final public function GetTreeQuality():Number
		{
			return m_tree.GetAreaRatio();
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