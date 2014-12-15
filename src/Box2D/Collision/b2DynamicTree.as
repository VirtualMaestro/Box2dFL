/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision
{
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
	 * TODO: Impl
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
		 * TODO:
		 */
		private function InsertLeaf(p_node:int):void
		{

		}

		/**
		 * TODO:
		 */
		private function RemoveLeaf(p_node:int):void
		{

		}

		/**
		 * TODO:
		 */
		private function Balance(p_index:int):int
		{
			return 0;
		}

//		/**
//		 * @return
//		 */
//		private function ComputeHeight():int
//		{
//
//		}

		/**
		 * @param p_nodeId
		 * @return
		 * TODO:
		 */
		private function ComputeHeight(p_nodeId:int):int
		{
			return 0;
		}

		/**
		 * TODO:
		 * @param p_index
		 */
		private function ValidateStructure(p_index:int):void
		{

		}

		/**
		 * TODO
		 * @param p_index
		 */
		private function ValidateMetrics(p_index:int):void
		{

		}



	}
}

import Box2D.Collision.b2AABB;

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
	final public function IsLeaf():Boolean
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
