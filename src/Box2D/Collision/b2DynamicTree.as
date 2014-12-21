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
//		private var m_path:uint;

		private var m_insertionCount:int;

		/**
		 */
		public function b2DynamicTree()
		{
			m_root = b2TreeNode.b2_nullNode;
			m_nodeCount = 0;     // nodes in use
			m_nodeCapacity = 0;  // total existing nodes, in pool
			m_freeList = 0;
//			m_path = 0;
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
		 */
		private function InsertLeaf(p_leaf:int):void
		{
			++m_insertionCount;

			if (m_root == b2TreeNode.b2_nullNode)
			{
				m_root = p_leaf;
				m_nodes[m_root].parent = b2TreeNode.b2_nullNode;
				return;
			}

			// Find the best sibling for this node
			var leafAABB:b2AABB = m_nodes[p_leaf].aabb.Clone() as b2AABB;
			var index:int = m_root;
			var combinedAABB:b2AABB = b2AABB.Get();

			while (m_nodes[index].IsLeaf == false)
			{
				var child1:int = m_nodes[index].child1;
				var child2:int = m_nodes[index].child2;

				var area:Number = m_nodes[index].aabb.GetPerimeter();


				combinedAABB.CombineTwo(m_nodes[index].aabb, leafAABB);
				var combinedArea:Number = combinedAABB.GetPerimeter();

				// Cost of creating a new parent for this node and the new leaf
				var cost:Number = 2.0 * combinedArea;

				// Minimum cost of pushing the leaf further down the tree
				var inheritanceCost:Number = 2.0 * (combinedArea - area);

				// Cost of descending into child1
				var cost1:Number;
				if (m_nodes[child1].IsLeaf)
				{
					combinedAABB.CombineTwo(leafAABB, m_nodes[child1].aabb);
					cost1 = combinedAABB.GetPerimeter() + inheritanceCost;
				}
				else
				{
					combinedAABB.CombineTwo(leafAABB, m_nodes[child1].aabb);
					var oldArea:Number = m_nodes[child1].aabb.GetPerimeter();
					var newArea:Number = combinedAABB.GetPerimeter();
					cost1 = (newArea - oldArea) + inheritanceCost;
				}

				// Cost of descending into child2
				var cost2:Number;

				if (m_nodes[child2].IsLeaf)
				{
					combinedAABB.CombineTwo(leafAABB, m_nodes[child2].aabb);
					cost2 = combinedAABB.GetPerimeter() + inheritanceCost;
				}
				else
				{
					combinedAABB.CombineTwo(leafAABB, m_nodes[child2].aabb);
					var oldArea:Number = m_nodes[child2].aabb.GetPerimeter();
					var newArea:Number = combinedAABB.GetPerimeter();
					cost2 = newArea - oldArea + inheritanceCost;
				}

				// Descend according to the minimum cost.
				if (cost < cost1 && cost < cost2)
				{
					break;
				}

				// Descend
				if (cost1 < cost2)
				{
					index = child1;
				}
				else
				{
					index = child2;
				}
			}

			combinedAABB.Dispose();

			var sibling:int = index;

			// Create a new parent.
			var oldParent:int = m_nodes[sibling].parent;
			var newParent:int = AllocateNode();
			m_nodes[newParent].parent = oldParent;
			m_nodes[newParent].userData = null;
			m_nodes[newParent].aabb.CombineTwo(leafAABB, m_nodes[sibling].aabb);
			m_nodes[newParent].height = m_nodes[sibling].height + 1;

			if (oldParent != b2TreeNode.b2_nullNode)
			{
				// The sibling was not the root.
				if (m_nodes[oldParent].child1 == sibling)
				{
					m_nodes[oldParent].child1 = newParent;
				}
				else
				{
					m_nodes[oldParent].child2 = newParent;
				}

				m_nodes[newParent].child1 = sibling;
				m_nodes[newParent].child2 = p_leaf;
				m_nodes[sibling].parent = newParent;
				m_nodes[p_leaf].parent = newParent;
			}
			else
			{
				// The sibling was the root.
				m_nodes[newParent].child1 = sibling;
				m_nodes[newParent].child2 = p_leaf;
				m_nodes[sibling].parent = newParent;
				m_nodes[p_leaf].parent = newParent;
				m_root = newParent;
			}

			// Walk back up the tree fixing heights and AABBs
			index = m_nodes[p_leaf].parent;
			while (index != b2TreeNode.b2_nullNode)
			{
				index = Balance(index);

				var child1:int = m_nodes[index].child1;
				var child2:int = m_nodes[index].child2;

				CONFIG::debug
				{
					b2Assert(child1 != b2TreeNode.b2_nullNode, "child1 can't be nullNode");
					b2Assert(child2 != b2TreeNode.b2_nullNode, "child2 can't be nullNode");
				}

				m_nodes[index].height = 1 + b2Math.Max(m_nodes[child1].height, m_nodes[child2].height);
				m_nodes[index].aabb.CombineTwo(m_nodes[child1].aabb, m_nodes[child2].aabb);

				index = m_nodes[index].parent;
			}
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
		 * Query an AABB for overlapping proxies. The callback class
		 * is called for each proxy that overlaps the supplied AABB.
		 * @param p_callback
		 * @param p_aabb
		 * TODO:
		 */
		public function Query(p_callback:Function, p_aabb:b2AABB):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		* Ray-cast against the proxies in the tree. This relies on the callback
		* to perform a exact ray-cast in the case were the proxy contains a shape.
		* The callback also performs the any collision filtering. This has performance
		* roughly equal to k * log(n), where k is the number of collisions and n is the
		* number of proxies in the tree.
		* @param p_input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
		* @param p_callback a callback class that is called for each proxy that is hit by the ray.
		* TODO:
		*/
		public function RayCast(p_callback:Function, p_input:b2RayCastData):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
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
