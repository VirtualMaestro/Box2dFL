/**
 * User: VirtualMaestro
 * Date: 23.12.2014
 * Time: 0:17
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.b2Assert;

	/**
	 */
	public class b2Simplex
	{
		public var m_v1:b2SimplexVertex;
		public var m_v2:b2SimplexVertex;
		public var m_v3:b2SimplexVertex;

		public var m_count:int;

		public function b2Simplex()
		{
			m_v1 = new b2SimplexVertex();
			m_v2 = new b2SimplexVertex();
			m_v3 = new b2SimplexVertex();

			m_count = 0;
		}

		/**
		 * TODO:
		 */
		public function ReadCache(p_cache:b2SimplexCache,
		                          p_proxyA:b2DistanceProxy, p_transformA:b2Mat22,
		                          p_proxyB:b2DistanceProxy, p_transformB:b2Mat22):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function WriteCache(p_cache:b2SimplexCache):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 *
		 * @return new instance of b2Vec2
		 * TODO
		 */
		public function GetSearchDirection():b2Vec2
		{
		   	b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 *
		 * @return new instance of b2Vec2
		 * TODO
		 */
		public function GetClosestPoint():b2Vec2
		{
		   	b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO
		 */
		public function GetWitnessPoints(p_pAX:Number, p_pAY:Number, p_pBX:Number, p_pBY:Number):void
		{
		   	b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function GetMetric():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function Solve2():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function Solve3():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}
	}
}
