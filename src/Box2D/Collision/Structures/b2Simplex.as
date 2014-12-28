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
		 */
		public function GetWitnessPoints(p_pA:b2Vec2,p_pB:b2Vec2):void
		{
			CONFIG::debug
			{
				b2Assert(m_count == 1 || m_count == 2 || m_count == 3, "m_count is incorrect: " + m_count);
			}

			if (m_count == 1)
			{
				p_pA.x = m_v1.wAX;
				p_pA.y = m_v1.wAY;

				p_pB.x = m_v1.wBX;
				p_pB.y = m_v1.wBY;
			}
			else if (m_count == 2)
			{
				var a1:Number = m_v1.a;
				var a2:Number = m_v2.a;

				p_pA.x = a1*m_v1.wAX + a2*m_v2.wAX;
				p_pA.y = a1*m_v1.wAY + a2*m_v2.wAY;

				p_pB.x = a1*m_v1.wBX + a2*m_v2.wBX;
				p_pB.y = a1*m_v1.wBY + a2*m_v2.wBY;
			}
			else
			{
				p_pA.x = a1*m_v1.wAX + a2*m_v2.wAX + m_v3.a*m_v3.wAX;
				p_pA.y = a1*m_v1.wAY + a2*m_v2.wAY + m_v3.a*m_v3.wAY;

				p_pB.x = p_pA.x;
				p_pB.y = p_pA.y;
			}
		}

		/**
		 */
		public function GetMetric():Number
		{
			CONFIG::debug
			{
				b2Assert(m_count == 1 || m_count == 2 || m_count == 3, "m_count is incorrect: " + m_count);
			}

			var result:Number = 0.0;

			if (m_count == 2)
			{
				// distance two vectors
				var rX:Number = m_v1.wX - m_v2.wX;
				var rY:Number = m_v1.wY - m_v2.wY;

				result =  Math.sqrt(rX*rX + rY*rY);
			}
			else if (m_count == 3)
			{
				var r1X:Number = m_v2.wX - m_v1.wX;
				var r1Y:Number = m_v2.wY - m_v1.wY;
				var r2X:Number = m_v3.wX - m_v1.wX;
				var r2Y:Number = m_v3.wY - m_v1.wY;

				result = r1X * r2Y - r1Y * r2X;
			}

			return result;
		}

		/**
		* Solve a line segment using barycentric coordinates.
		*
		* p = a1 * w1 + a2 * w2
		* a1 + a2 = 1
		*
		* The vector from the origin to the closest point on the line is
		* perpendicular to the line.
		* e12 = w2 - w1
		* dot(p, e) = 0
		* a1 * dot(w1, e) + a2 * dot(w2, e) = 0
		*
		* 2-by-2 linear system
		* [1      1     ][a1] = [1]
		* [w1.e12 w2.e12][a2] = [0]
		*
		* Define
		* d12_1 =  dot(w2, e12)
		* d12_2 = -dot(w1, e12)
		* d12 = d12_1 + d12_2
		*
		* Solution
		* a1 = d12_1 / d12
		* a2 = d12_2 / d12
		*/
		public function Solve2():void
		{
			var e12X:Number = m_v2.wX - m_v1.wX;
			var e12Y:Number = m_v2.wY - m_v1.wY;

			 // w1 region
			var d12_2:Number = -(m_v1.wX * e12X + m_v1.wY * e12Y);

			if (d12_2 <= 0.0)
			{
				// a2 <= 0, so we clamp it to 0
				m_v1.a = 1.0;
				m_count = 1;

				return;
			}

			// w2 region
			var d12_1:Number = (m_v2.wX * e12X + m_v2.wY * e12Y);

			if (d12_1 <= 0.0)
			{
				// a1 <= 0, so we clamp it to 0
				m_v2.a = 1.0;
				m_count = 1;
				m_v1.Set(m_v2);

				return;
			}

			// Must be in e12 region.
			var inv_d12:Number = 1.0 / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_2 * inv_d12;
			m_count = 2;
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
