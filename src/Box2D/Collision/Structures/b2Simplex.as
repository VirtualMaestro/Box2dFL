/**
 * User: VirtualMaestro
 * Date: 23.12.2014
 * Time: 0:17
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2Settings;
	import Box2D.b2Assert;

	/**
	 */
	public class b2Simplex
	{
		public var m_v1:b2SimplexVertex;
		public var m_v2:b2SimplexVertex;
		public var m_v3:b2SimplexVertex;

		public var m_count:int;

		private var _simpleVertexList:Vector.<b2SimplexVertex>;

		/**
		 */
		public function b2Simplex()
		{
			m_v1 = new b2SimplexVertex();
			m_v2 = new b2SimplexVertex();
			m_v3 = new b2SimplexVertex();

			m_count = 0;

			_simpleVertexList = new <b2SimplexVertex>[m_v1, m_v2, m_v3];
		}

		/**
		 */
		public function ReadCache(p_cache:b2SimplexCache,
		                          p_proxyA:b2DistanceProxy, p_transformA:b2Mat22,
		                          p_proxyB:b2DistanceProxy, p_transformB:b2Mat22):void
		{
			CONFIG::debug
			{
				b2Assert(p_cache.count <= 3, "incorrect simple cache count: " + p_cache.count);
			}

			// Copy data from cache.
			m_count = p_cache.count;
			var v:b2SimplexVertex;
			var wALocalX:Number;
			var wALocalY:Number;
			var wBLocalX:Number;
			var wBLocalY:Number;
			var iA:int;
			var iB:int;
			var cos:Number;
			var sin:Number;

			for (var i:int = 0; i < m_count; i++)
			{
				v = _simpleVertexList[i];
				iA = p_cache.indexA[i];
				iB = p_cache.indexB[i];

				v.indexA = iA;
				v.indexB = iB;

				wALocalX = p_proxyA.GetVertexX(iA);
				wALocalY = p_proxyA.GetVertexY(iA);
				wBLocalX = p_proxyB.GetVertexX(iB);
				wBLocalY = p_proxyB.GetVertexY(iB);
				
				cos = p_transformA.c11;
				sin = p_transformA.c12;
				
				v.wAX = (cos * wALocalX - sin * wALocalY) + p_transformA.tx;
				v.wAY = (sin * wALocalX + cos * wALocalY) + p_transformA.ty;

				cos = p_transformB.c11;
				sin = p_transformB.c12;

				v.wBX = (cos * wBLocalX - sin * wBLocalY) + p_transformB.tx;
				v.wBY = (sin * wBLocalX + cos * wBLocalY) + p_transformB.ty;

				v.wX = v.wBX - v.wAX;
				v.wY = v.wBY - v.wAY;
				v.a = 0.0;
			}

			 // Compute the new simplex metric, if it is substantially different than
			 // old metric then flush the simplex.
			if (m_count > 1)
			{
				var metric1:Number = p_cache.metric;
				var metric2:Number = GetMetric();

				if (metric2 < 0.5*metric1 || 2.0*metric1 < metric2 || metric2 < b2Math.EPSILON)
				{
					// Reset the simplex.
					m_count = 0;
				}
			}

			// If the cache is empty or invalid ...
			if (m_count == 0)
			{
				v = _simpleVertexList[0];
				v.indexA = 0;
				v.indexB = 0;

				wALocalX = p_proxyA.GetVertexX(0);
				wALocalY = p_proxyA.GetVertexY(0);
				wBLocalX = p_proxyB.GetVertexX(0);
				wBLocalY = p_proxyB.GetVertexY(0);

				cos = p_transformA.c11;
				sin = p_transformA.c12;

				v.wAX = (cos * wALocalX - sin * wALocalY) + p_transformA.tx;
				v.wAY = (sin * wALocalX + cos * wALocalY) + p_transformA.ty;

				cos = p_transformB.c11;
				sin = p_transformB.c12;

				v.wBX = (cos * wBLocalX - sin * wBLocalY) + p_transformB.tx;
				v.wBY = (sin * wBLocalX + cos * wBLocalY) + p_transformB.ty;

				v.wX = v.wBX - v.wAX;
				v.wY = v.wBY - v.wAY;
				v.a = 1.0;
				m_count = 1;
			}
		}

		/**
		 */
		public function WriteCache(p_cache:b2SimplexCache):void
		{
			p_cache.metric = GetMetric();
			p_cache.count = m_count;

			var v:b2SimplexVertex;

			for (var i:int = 0; i < m_count; i++)
			{
				v = _simpleVertexList[i];
				p_cache.indexA[i] = v.indexA;
				p_cache.indexB[i] = v.indexB;
			}
		}

		/**
		 *
		 * @return new instance of b2Vec2
		 */
		public function GetSearchDirection():b2Vec2
		{
			CONFIG::debug
			{
				b2Assert(m_count == 1 || m_count == 2, "m_count is incorrect: " + m_count);
			}

			var result:b2Vec2;

			if (m_count == 1)
			{
				result = b2Vec2.Get(-m_v1.wX, -m_v1.wY);
			}
			else
			{
				var e12X:Number = m_v2.wX - m_v1.wX;
				var e12Y:Number = m_v2.wY - m_v1.wY;
				
				var sign:Number = e12X * -m_v1.wY - e12Y * -m_v1.wX;
				
				if (sign > 0.0)
				{
					//Origin is left of e12
					result = b2Vec2.Get(-e12Y, e12X);
				}
				else
				{
					// Origin is right of e12
					result = b2Vec2.Get(e12Y, -e12X);
				}
			}

		   	return result;
		}

		/**
		 *
		 * @return new instance of b2Vec2
		 */
		public function GetClosestPoint():b2Vec2
		{
			CONFIG::debug
			{
				b2Assert(m_count == 1 || m_count == 2 || m_count == 3, "m_count is incorrect: " + m_count);
			}

			var result:b2Vec2;

			if (m_count == 1)
			{
				result = b2Vec2.Get(m_v1.wX, m_v1.wY);
			}
			else if (m_count == 2)
			{
				var r1X:Number = m_v1.wX * m_v1.a;
				var r1Y:Number = m_v1.wY * m_v1.a;
				var r2X:Number = m_v2.wX * m_v2.a;
				var r2Y:Number = m_v2.wY * m_v2.a;

				var rX:Number = r1X + r2X;
				var rY:Number = r1Y + r2Y;

				result = b2Vec2.Get(rX, rY);
			}
			else
			{
				result = b2Vec2.Get();
			}

			return result;
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
		public function GetMetric():Number
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
			return 0.0;
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
