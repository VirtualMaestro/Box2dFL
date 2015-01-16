/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
	 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
	 * It is up to the client to consume the new pairs and to track subsequent overlap.
	 */
	public class b2BroadPhase
	{
		static b2internal const e_nullProxy:int = -1;

		b2internal var m_tree:b2DynamicTree;

		b2internal var m_proxyCount:int;

		b2internal var m_moveBuffer:Vector.<int>;
		b2internal var m_moveCount:int;

		b2internal var m_pairBuffer:Vector.<b2Pair>;
		b2internal var m_pairCapacity:int;
		b2internal var m_pairCount:int;

		b2internal var m_queryProxyId:int;

		/**
		 */
		public function b2BroadPhase()
		{
			m_proxyCount = 0;

			m_pairCount = 0;
			m_pairCapacity = 16;
			m_pairBuffer = new <b2Pair>[];

			for (var i:int = 0; i < m_pairCapacity; i++)
			{
				m_pairBuffer[i] = new b2Pair();
			}

			m_moveCount = 0;
			m_moveBuffer = new <int>[];

			m_tree = new b2DynamicTree();
		}

		/**
		 * Create a proxy with an initial AABB. Pairs are not reported until
		 * UpdatePairs is called.
		 */
		public function CreateProxy(p_aabb:b2AABB, p_userData:*):int
		{
			var proxyId:int = m_tree.CreateProxy(p_aabb, p_userData);
			++m_proxyCount;
			BufferMove(proxyId);

			return proxyId;
		}

		/**
		 * Call MoveProxy as many times as you like, then when you are done
		 * call UpdatePairs to finalized the proxy pairs (for your time step).
		 * @param p_proxyId
		 * @param p_aabb
		 * @param p_displacementX
		 * @param p_displacementY
		 */
		[Inline]
		final public function MoveProxy(p_proxyId:int, p_aabb:b2AABB, p_displacementX:Number, p_displacementY:Number):void
		{
			var buffer:Boolean = m_tree.MoveProxy(p_proxyId, p_aabb, p_displacementX, p_displacementY);
			if (buffer)
			{
				BufferMove(p_proxyId);
			}
		}

		/**
		 * Destroy a proxy. It is up to the client to remove any pairs.
		 * @param p_proxyId
		 */
		[Inline]
		final public function DestroyProxy(p_proxyId:int):void
		{
			UnBufferMove(p_proxyId);
			--m_proxyCount;
			m_tree.DestroyProxy(p_proxyId);
		}

		/**
		 * Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
		 * @param p_proxyId
		 */
		[Inline]
		final public function TouchProxy(p_proxyId:int):void
		{
			BufferMove(p_proxyId);
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
		[Inline]
		final public function TestOverlap(p_proxyIdA:int, p_proxyIdB:int):Boolean
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
		final private function BufferMove(p_proxyId:int):void
		{
			m_moveBuffer[m_moveCount] = p_proxyId;
			++m_moveCount;
		}

		/**
		 */
		[Inline]
		final private function UnBufferMove(p_proxyId:int):void
		{
			for (var i:int = 0; i < m_moveCount; ++i)
			{
				if (m_moveBuffer[i] == p_proxyId)
				{
					m_moveBuffer[i] = e_nullProxy;
				}
			}
		}

		/**
		 * This is called from b2DynamicTree::Query when we are gathering pairs.
		 * @param p_proxyId
		 * @return
		 */
		[Inline]
		final public function QueryCallback(p_proxyId:int):Boolean
		{
			// A proxy form a pair
			if (p_proxyId != m_queryProxyId)
			{
				if (m_pairCount >= m_pairCapacity)
				{
					extendsPairBuffer();
				}

				m_pairBuffer[m_pairCount].proxyIdA = b2Math.Min(p_proxyId, m_queryProxyId);
				m_pairBuffer[m_pairCount].proxyIdB = b2Math.Min(p_proxyId, m_queryProxyId);
				++m_pairCount;
			}

			return true;
		}

		[Inline]
		final private function extendsPairBuffer():void
		{
			var newSize:int = m_pairCapacity * 2;
			for (var i:int = m_pairCapacity; i < newSize; i++)
			{
				m_pairBuffer[i] = new b2Pair();
			}
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