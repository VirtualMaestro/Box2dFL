/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Settings;
	import Box2D.b2Assert;

	/**
	 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
	 * A dynamic tree arranges data in a binary tree to accelerate
	 * queries such as volume queries and ray casts. Leafs are proxies
	 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
	 * so that the proxy AABB is bigger than the client object. This allows the client
	 * object to move by small amounts without triggering a tree update.
	 *
	 * Nodes are pooled and relocatable, so we use node indices rather than pointers.
	 *
	 * TODO: Optimize methods - Balance, RemoveLeaf, InsertLeaf
	 */
	public class b2DynamicTree
	{
		private var m_root:int;

		private var m_nodes:Vector.<b2TreeNode>;
		private var m_nodeCount:int;
		private var m_nodeCapacity:int;

		private var m_freeList:int;

		/// This is used to incrementally traverse the tree for re-balancing.
		private var m_path:uint;

		private var m_insertionCount:int;

		/**
		 */
		public function b2DynamicTree()
		{
			m_root = b2TreeNode.b2_nullNode;
			m_nodeCount = 0;     // nodes in use
			m_nodeCapacity = 0;  // total existing nodes, in pool
			m_freeList = 0;
			m_path = 0;
			m_insertionCount = 0;
			m_nodes = new <b2TreeNode>[];
		}

		/**
		 * @return
		 */
		[Inline]
		final private function AllocateNode():int
		{
			var node:b2TreeNode;
			var nodeId:int;

			if (m_freeList == m_nodeCapacity) // element doesn't exist
			{
				++m_nodeCapacity;

				nodeId = m_freeList;
				++m_freeList;

				node = new b2TreeNode();
				m_nodes[nodeId] = node;

				node.next = m_freeList;
			}
			else   // some element from exist
			{
				nodeId = m_freeList;
				node = m_nodes[nodeId];
				m_freeList = node.next;
			}

			node.parent = b2TreeNode.b2_nullNode;
			node.child1 = b2TreeNode.b2_nullNode;
			node.child2 = b2TreeNode.b2_nullNode;
			node.height = 0;

			m_nodeCount++;

			return nodeId;
		}

		/**
		 */
		[Inline]
		final private function FreeNode(p_nodeId:int):void
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_nodeId || 0 < m_nodeCount, "incorrect nodeId");
			}

			var node:b2TreeNode = m_nodes[p_nodeId];
			node.next = m_freeList;
			node.height = -1;
			node.userData = null;
			m_freeList = p_nodeId;
			--m_nodeCount;
		}

		/**
		 *  Create a proxy in the tree as a leaf node.
		 *  We return the index of the node instead of a pointer so that we can grow the node pool.
		 * @param p_aabb
		 * @param p_userData
		 * @return
		 */
		public function CreateProxy(p_aabb:b2AABB, p_userData:*):int
		{
			var proxyId:int = AllocateNode();

			var rX:Number = b2Settings.aabbExtension;
			var rY:Number = b2Settings.aabbExtension;
			var node:b2TreeNode = m_nodes[proxyId];
			var	aabb:b2AABB = node.aabb;
			aabb.lowerBoundX = aabb.lowerBoundX - rX;
			aabb.lowerBoundY = aabb.lowerBoundY - rY;
			aabb.upperBoundX = aabb.upperBoundX + rX;
			aabb.upperBoundY = aabb.upperBoundY + rY;
			node.userData = p_userData;
			node.height = 0;

			InsertLeaf(proxyId);

			return proxyId;
		}

		/**
		 * Destroy a proxy. This asserts if the id is invalid.
		 * @param p_proxyId
		 */
		public function DestroyProxy(p_proxyId:int):void
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_proxyId && p_proxyId < m_nodeCapacity, "proxyId is invalid");
				b2Assert(m_nodes[p_proxyId].IsLeaf, "proxyId is not leaf");
			}

			RemoveLeaf(p_proxyId);
			FreeNode(p_proxyId);
		}

		/**
		 * Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
		 * then the proxy is removed from the tree and re-inserted.
		 * Otherwise the function returns immediately.
		 *
		 * @param p_proxyId
		 * @param p_aabb
		 * @param p_displacementX
		 * @param p_displacementY
		 * @return true if the proxy was re-inserted.
		 */
		public function MoveProxy(p_proxyId:int, p_aabb:b2AABB, p_displacementX:Number, p_displacementY:Number):Boolean
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_proxyId && p_proxyId < m_nodeCapacity, "proxyId is invalid");
				b2Assert(m_nodes[p_proxyId].IsLeaf, "proxyId is not leaf");
			}

			var nodeAABB:b2AABB = m_nodes[p_proxyId].aabb;

			if (!nodeAABB.Contains(p_aabb))
			{
				RemoveLeaf(p_proxyId);

				// Extend AABB
				var lbX:Number = p_aabb.lowerBoundX - b2Settings.aabbExtension;
				var lbY:Number = p_aabb.lowerBoundY - b2Settings.aabbExtension;
				var ubX:Number = p_aabb.upperBoundX + b2Settings.aabbExtension;
				var ubY:Number = p_aabb.upperBoundY + b2Settings.aabbExtension;

				// Predict AABB displacement.
				var dX:Number = b2Settings.aabbMultiplier * p_displacementX;
				var dY:Number = b2Settings.aabbMultiplier * p_displacementY;

				if (dX < 0.0)
				{
					lbX += dX;
				}
				else
				{
					ubX += dX;
				}

				if (dY < 0.0)
				{
					lbY += dY;
				}
				else
				{
					ubY += dY;
				}

				nodeAABB.lowerBoundX = lbX;
				nodeAABB.lowerBoundY = lbY;
				nodeAABB.upperBoundX = ubX;
				nodeAABB.upperBoundY = ubY;

				InsertLeaf(p_proxyId);

				return true;
			}

			return false;
		}

		/**
		 * TODO:
		 */
		private function InsertLeaf(p_node:int):void
		{

		}

		/**
		 */
		private function RemoveLeaf(p_leaf:int):void
		{
			if (p_leaf == m_root)
			{
				m_root = b2TreeNode.b2_nullNode;
				return;
			}

			var parent:int = m_nodes[p_leaf].parent;
			var grandParent:int = m_nodes[parent].parent;
			var sibling:int;

			if (m_nodes[parent].child1 == p_leaf)
			{
				sibling = m_nodes[parent].child2;
			}
			else
			{
				sibling = m_nodes[parent].child1;
			}

			//
			if (grandParent != b2TreeNode.b2_nullNode)
			{
				// Destroy parent and connect sibling to grandParent.
				if (m_nodes[grandParent].child1 == parent)
				{
					m_nodes[grandParent].child1 = sibling;
				}
				else
				{
					m_nodes[grandParent].child2 = sibling;
				}

				m_nodes[sibling].parent = grandParent;

				FreeNode(parent);

				// Adjust ancestor bounds.
				var index:int = grandParent;
				var child1:int;
				var child2:int;

				while(index != b2TreeNode.b2_nullNode)
				{
					index = Balance(index);

					child1 = m_nodes[index].child1;
					child2 = m_nodes[index].child2;

					m_nodes[index].aabb.CombineTwo(m_nodes[child1].aabb, m_nodes[child2].aabb);
					m_nodes[index].height = 1 + b2Math.Max(m_nodes[child1].height, m_nodes[child2].height);

					index = m_nodes[index].parent;
				}
			}
			else
			{
				m_root = sibling;
				m_nodes[sibling].parent = b2TreeNode.b2_nullNode;
				FreeNode(parent);
			}
		}

		/**
		 * Perform a left or right rotation if node A is imbalanced.
		 * Returns the new root index.
		 */
		private function Balance(p_iA:int):int
		{
			CONFIG::debug
			{
				b2Assert(p_iA != b2TreeNode.b2_nullNode, "incorrect node, iA == " + p_iA);
			}

			var A:b2TreeNode = m_nodes[p_iA];
			if (A.IsLeaf || A.height < 2)
			{
				return p_iA;
			}

			var iB:int = A.child1;
			var iC:int = A.child2;

			CONFIG::debug
			{
				b2Assert(0 <= iB && iB < m_nodeCapacity, "incorrect node index iB");
				b2Assert(0 <= iC && iC < m_nodeCapacity, "incorrect node index iC");
			}

			var B:b2TreeNode = m_nodes[iB];
			var C:b2TreeNode = m_nodes[iC];

			var balance:int = C.height - B.height;

			// Rotate C up
			if (balance > 1)
			{
				var iF:int = C.child1;
				var iG:int = C.child2;

				var F:b2TreeNode = m_nodes[iF];
				var G:b2TreeNode = m_nodes[iG];

				CONFIG::debug
				{
					b2Assert(0 <= iF && iF < m_nodeCapacity, "incorrect node index iF");
					b2Assert(0 <= iG && iG < m_nodeCapacity, "incorrect node index iG");
				}

				// Swap A and C
				C.child1 = p_iA;
				C.parent = A.parent;
				A.parent = iC;

				// A's old parent should point to C
				if (C.parent != b2TreeNode.b2_nullNode)
				{
					if (m_nodes[C.parent].child1 == p_iA)
					{
						m_nodes[C.parent].child1 = iC;
					}
					else
					{
						 CONFIG::debug
						 {
							 b2Assert(m_nodes[C.parent].child2 == p_iA, "m_nodes[C.parent].child2 == p_iA");
						 }

						m_nodes[C.parent].child2 = iC;
					}
				}
				else
				{
					m_root = iC;
				}

				// Rotate
				if (F.height > G.height)
				{
					C.child2 = iF;
					A.child2 = iG;
					G.parent = p_iA;

					A.aabb.CombineTwo(B.aabb, G.aabb);
					C.aabb.CombineTwo(A.aabb, F.aabb);

					A.height = 1 + b2Math.Max(B.height, G.height);
					C.height = 1 + b2Math.Max(A.height, F.height);
				}
				else
				{
					C.child2 = iG;
					A.child2 = iF;
					F.parent = p_iA;

					A.aabb.CombineTwo(B.aabb, F.aabb);
					C.aabb.CombineTwo(A.aabb, G.aabb);

					A.height = 1 + b2Math.Max(B.height, F.height);
					C.height = 1 + b2Math.Max(A.height, G.height);
				}

				return iC;
			}

			// Rotate B up
			if (balance < -1)
			{
				var iD:int = B.child1;
				var iE:int = B.child2;

				var D:b2TreeNode = m_nodes[iD];
				var E:b2TreeNode = m_nodes[iE];

				CONFIG::debug
				{
					b2Assert(0 <= iD && iD < m_nodeCapacity, "0 <= iD && iD < m_nodeCapacity");
					b2Assert(0 <= iE && iE < m_nodeCapacity, "0 <= iE && iE < m_nodeCapacity");
				}

				// Swap A and B
				B.child1 = p_iA;
				B.parent = A.parent;
				A.parent = iB;

				// A's old parent should point to B
				if (B.parent != b2TreeNode.b2_nullNode)
				{
					if (m_nodes[B.parent].child1 == p_iA)
					{
						m_nodes[B.parent].child1 = iB;
					}
					else
					{
						CONFIG::debug
						{
							b2Assert(m_nodes[B.parent].child2 == p_iA, "m_nodes[B.parent].child2 == p_iA");
						}

						m_nodes[B.parent].child2 = iB;
					}
				}
				else
				{
					m_root = iB;
				}

				// Rotate
				if (D.height > E.height)
				{
					B.child2 = iD;
					A.child1 = iE;
					E.parent = p_iA;

					A.aabb.CombineTwo(C.aabb, E.aabb);
					B.aabb.CombineTwo(A.aabb, D.aabb);

					A.height = 1 + b2Math.Max(C.height, E.height);
					B.height = 1 + b2Math.Max(A.height, D.height);
				}
				else
				{
					B.child2 = iE;
					A.child1 = iD;
					D.parent = p_iA;

					A.aabb.CombineTwo(C.aabb, D.aabb);
					B.aabb.CombineTwo(A.aabb, E.aabb);

					A.height = 1 + b2Math.Max(C.height, D.height);
					B.height = 1 + b2Math.Max(A.height, E.height);
				}

				return iB;
			}

			return p_iA;
		}

		/**
		 * Compute the height of the binary tree in O(N) time. Should not be called often.
 		 * @return
		 */
		public function GetHeight():int
		{
			if (m_root == b2TreeNode.b2_nullNode)
			{
				return 0;
			}

			return m_nodes[m_root].height;
		}

		/**
		 * Get the ratio of the sum of the node areas to the root area.
		 * @return  Number
		 */
		public function GetAreaRatio():Number
		{
			if (m_root == b2TreeNode.b2_nullNode)
			{
				return 0.0;
			}

			var root:b2TreeNode = m_nodes[m_root];
			var rootArea:Number = root.aabb.GetPerimeter();
			var numNodes:int = m_nodeCapacity;

			var totalArea:Number = 0.0;
			for (var i:int = 0; i < numNodes; ++i)
			{
				var node:b2TreeNode = m_nodes[i];

				if (node.height < 0)
				{
					// Free node in pool
					continue;
				}

				totalArea += node.aabb.GetPerimeter();
			}

			return totalArea / rootArea;
		}

		/**
		 * @return
		 */
		[Inline]
		final private function ComputeHeightRoot():int
		{
			return ComputeHeight(m_root);
		}

		/**
		 * Compute the height of a sub-tree.
		 * @param p_nodeId
		 * @return
		 */
		[Inline]
		final private function ComputeHeight(p_nodeId:int):int
		{
			CONFIG::debug
			{
				b2Assert(0 <= p_nodeId && p_nodeId < m_nodeCapacity, "p_nodeId is invalid");
			}

			var node:b2TreeNode = m_nodes[p_nodeId];

			if (node.IsLeaf)
			{
				return 0;
			}

			var height1:int = ComputeHeight(node.child1);
			var height2:int = ComputeHeight(node.child2);

			return 1 + b2Math.Max(height1, height2);
		}

		/**
		 * Shift the world origin. Useful for large worlds.
		 * The shift formula is: position -= newOrigin
		 * @param p_newOriginX the new origin with respect to the old origin
		 * @param p_newOriginY the new origin with respect to the old origin
 		 */
		[Inline]
		final public function ShiftOrigin(p_newOriginX:Number, p_newOriginY:Number):void
		{
			var aabb:b2AABB;
			var numNodes:int = m_nodeCapacity;

			// Build array of leaves. Free the rest.
			for (var i:int = 0; i < numNodes; ++i)
			{
				aabb = m_nodes[i].aabb;
				aabb.lowerBoundX -= p_newOriginX;
				aabb.lowerBoundY -= p_newOriginY;
				aabb.upperBoundX -= p_newOriginX;
				aabb.upperBoundY -= p_newOriginY;
			}
		}

		/**
		 * Get the maximum balance of an node in the tree.
		 * The balance is the difference in height of the two children of a node.
		 * @return int
		 */
		[Inline]
		final public function GetMaxBalance():int
		{
			var maxBalance:int = 0;
			var node:b2TreeNode;
			var numNodes:int = m_nodeCapacity;

			for (var i:int = 0; i < numNodes; i++)
			{
				node = m_nodes[i];

				if (node.height <= 1)
				{
					continue;
				}

				CONFIG::debug
				{
					b2Assert(node.IsLeaf == false, "node isn't leaf");
				}

				var balance:int = b2Math.Abs(m_nodes[node.child2].height - m_nodes[node.child1].height);
				maxBalance = b2Math.Max(maxBalance, balance);
			}

			return maxBalance;
		}

		/**
		 * TODO: Not necessary
		 * @param p_index
		 */
		private function ValidateStructure(p_index:int):void
		{

		}

		/**
		 * TODO: Not necessary
		 * @param p_index
		 */
		private function ValidateMetrics(p_index:int):void
		{

		}

		/**
		 * Build an optimal tree. Very expensive.
		 * For testing.
		 * TODO: Not necessary
 		 */
		private function RebuildBottomUp():void
		{

		}
	}
}

/**
 *
 */
internal class Stack
{
	private var _stack:Vector.<int>;
	private var _count:int = 0;

	/**
	 */
	public function Stack()
	{
		_stack = new <int>[];
	}

	/**
	 */
	[Inline]
	final public function Push(p_element:int):void
	{
		_stack[_count++] = p_element;
	}

	/**
	 */
	[Inline]
	final public function Pop():int
	{
		return _stack[--_count];
	}

	/**
	 */
	[Inline]
	final public function GetCount():int
	{
		return _count;
	}

	/**
	 * Clear stack.
	 */
	public function Free():void
	{
		_stack.length = 0;
		_count = 0;
	}
}

import Box2D.Collision.b2AABB;

/**
 * A node in the dynamic tree. The client does not interact with this directly.
 */
internal class b2TreeNode
{
	static public const b2_nullNode:int = -1;

	public var child1:int;
	public var child2:int;

	// leaf = 0, free node = -1
	public var height:int;

	public var parent:int;
	public var next:int;

	public var aabb:b2AABB;

	public var userData:*;

	/**
	 */
	[Inline]
	final public function get IsLeaf():Boolean
	{
		return child1 == b2_nullNode;
	}

	/**
	 */
	public function b2TreeNode()
	{
		aabb = b2AABB.Get();
	}
}
