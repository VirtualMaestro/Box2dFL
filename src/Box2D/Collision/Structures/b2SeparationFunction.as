/**
 * User: VirtualMaestro
 * Date: 13.01.2015
 * Time: 17:32
 */
package Box2D.Collision.Structures
{
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.b2Assert;

	/**
	 *
	 */
	public class b2SeparationFunction
	{

		static public const e_points:int = 0;
		static public const e_faceA:int = 1;
		static public const e_faceB:int = 2;

		public var m_proxyA:b2DistanceProxy;
		public var m_proxyB:b2DistanceProxy;
		public var m_sweepA:b2Sweep;
		public var m_sweepB:b2Sweep;
		public var m_type:int;
		public var m_localPointX:Number;
		public var m_localPointY:Number;
		public var m_axisX:Number;
		public var m_axisY:Number;

		private var xfA:b2Mat22;
		private var xfB:b2Mat22;
		private var _pointHelper:b2SPoint;

		/**
		 */
		public function b2SeparationFunction()
		{
			m_sweepA = b2Sweep.Get();
			m_sweepB = b2Sweep.Get();
			xfA = b2Mat22.Get();
			xfB = b2Mat22.Get();
			_pointHelper = b2SPoint.Get();
		}

		/**
		 */
		public function Initialize(p_cache:b2SimplexCache,
		                           p_proxyA:b2DistanceProxy, p_sweepA:b2Sweep,
		                           p_proxyB:b2DistanceProxy, p_sweepB:b2Sweep, p_t1:Number):Number
		{
			m_proxyA = p_proxyA;
			m_proxyB = p_proxyB;

			var count:int = p_cache.count;

			CONFIG::debug
			{
				b2Assert(0 < count && count < 3, "!(0 < count && count < 3)");
			}

			m_sweepA.Set(p_sweepA);
			m_sweepB.Set(p_sweepB);

			m_sweepA.GetTransform(xfA, p_t1);
			m_sweepB.GetTransform(xfB, p_t1);

			//
			var localPointAX:Number;
			var localPointAY:Number;
			var localPointBX:Number;
			var localPointBY:Number;
			var pointAX:Number;
			var pointAY:Number;
			var pointBX:Number;
			var pointBY:Number;
			var s:Number;
			var localPointB1X:Number;
			var localPointB1Y:Number;
			var localPointB2X:Number;
			var localPointB2Y:Number;
			var localPointA1X:Number;
			var localPointA1Y:Number;
			var localPointA2X:Number;
			var localPointA2Y:Number;
			var normalX:Number;
			var normalY:Number;
			var invLength:Number;

			//
			if (count == 1)
			{
				m_type = e_points;

				m_proxyA.GetVertex(p_cache.indexA[0], _pointHelper);
				localPointAX = _pointHelper.x;
				localPointAY = _pointHelper.y;

				m_proxyB.GetVertex(p_cache.indexB[0], _pointHelper);
				localPointBX = _pointHelper.x;
				localPointBY = _pointHelper.y;

				b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
				pointAX = _pointHelper.x;
				pointAY = _pointHelper.y;

				b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
				pointBX = _pointHelper.x;
				pointBY = _pointHelper.y;

				m_axisX = pointBX - pointAX;
				m_axisY = pointBY - pointAY;

				s = b2Math.Normalize(m_axisX, m_axisY, _pointHelper);
				m_axisX = _pointHelper.x;
				m_axisY = _pointHelper.y;

				return s;
			}
			else if (p_cache.indexA[0] == p_cache.indexA[1])
			{
				// Two points on B and one on A.
				m_type = e_faceB;

				p_proxyB.GetVertex(p_cache.indexB[0], _pointHelper);
				localPointB1X = _pointHelper.x;
				localPointB1Y = _pointHelper.y;

				p_proxyB.GetVertex(p_cache.indexB[1], _pointHelper);
				localPointB2X = _pointHelper.x;
				localPointB2Y = _pointHelper.y;

				b2Math.CrossVectorScalar((localPointB2X - localPointB1X), (localPointB2Y - localPointB1Y), 1.0, _pointHelper);
				m_axisX = _pointHelper.x;
				m_axisY = _pointHelper.y;

				invLength = b2Math.invLength(m_axisX, m_axisY);
				m_axisX *= invLength;
				m_axisY *= invLength;

				b2Math.MulRV(xfB, m_axisX, m_axisY, _pointHelper);
				normalX = _pointHelper.x;
				normalY = _pointHelper.y;

				m_localPointX = 0.5 * (localPointB1X + localPointB2X);
				m_localPointY = 0.5 * (localPointB1Y + localPointB2Y);

				b2Math.MulTrV(xfB, m_localPointX, m_localPointY, _pointHelper);
				pointBX = _pointHelper.x;
				pointBY = _pointHelper.y;

				p_proxyA.GetVertex(p_cache.indexA[0], _pointHelper);
				localPointAX = _pointHelper.x;
				localPointAY = _pointHelper.y;

				b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
				pointAX = _pointHelper.x;
				pointAY = _pointHelper.y;

				s = b2Math.Dot((pointAX - pointBX), (pointAY - pointBY), normalX, normalY);

				if (s < 0.0)
				{
					m_axisX = -m_axisX;
					m_axisY = -m_axisY;
					s = -s;
				}

				return s;
			}
			else
			{
				// Two points on A and one or two points on B.
				m_type = e_faceA;

				m_proxyA.GetVertex(p_cache.indexA[0], _pointHelper);
				localPointA1X = _pointHelper.x;
				localPointA1Y = _pointHelper.y;

				m_proxyA.GetVertex(p_cache.indexA[1], _pointHelper);
				localPointA2X = _pointHelper.x;
				localPointA2Y = _pointHelper.y;

				b2Math.CrossVectorScalar((localPointA2X - localPointA1X), (localPointA2Y - localPointA1Y), 1.0, _pointHelper);
				m_axisX = _pointHelper.x;
				m_axisY = _pointHelper.y;

				invLength = b2Math.invLength(m_axisX, m_axisY);
				m_axisX *= invLength;
				m_axisY *= invLength;

				b2Math.MulRV(xfA, m_axisX, m_axisY, _pointHelper);
				normalX = _pointHelper.x;
				normalY = _pointHelper.y;

				m_localPointX = 0.5 * (localPointA1X + localPointA2X);
				m_localPointY = 0.5 * (localPointA1Y + localPointA2Y);

				b2Math.MulTrV(xfA, m_localPointX, m_localPointY, _pointHelper);
				pointAX = _pointHelper.x;
				pointAY = _pointHelper.y;

				m_proxyB.GetVertex(p_cache.indexB[0], _pointHelper);
				localPointBX = _pointHelper.x;
				localPointBY = _pointHelper.y;

				b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
				pointBX = _pointHelper.x;
				pointBY = _pointHelper.y;

				s = b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), normalX, normalY);

				if (s < 0.0)
				{
					m_axisX = -m_axisX;
					m_axisY = -m_axisY;
					s = -s;
				}

				return s;
			}
		}

		/**
		 */
		public function FindMinSeparation(p_indexA:int, p_indexB:int, p_t:Number, p_outResult:b2SPoint):Number
		{
			m_sweepA.GetTransform(xfA, p_t);
			m_sweepB.GetTransform(xfB, p_t);

			var axisAX:Number;
			var axisAY:Number;
			var axisBX:Number;
			var axisBY:Number;
			var localPointAX:Number;
			var localPointAY:Number;
			var localPointBX:Number;
			var localPointBY:Number;
			var pointAX:Number;
			var pointAY:Number;
			var pointBX:Number;
			var pointBY:Number;
			var normalX:Number;
			var normalY:Number;

			switch (m_type)
			{
				case e_points:
				{
					b2Math.MulTRV(xfA, m_axisX, m_axisY, _pointHelper);
					axisAX = _pointHelper.x;
					axisAY = _pointHelper.y;

					b2Math.MulTRV(xfB, -m_axisX, -m_axisY, _pointHelper);
					axisBX = _pointHelper.x;
					axisBY = _pointHelper.y;

					p_indexA = m_proxyA.GetSupport(axisAX, axisAY);
					p_indexB = m_proxyB.GetSupport(axisBX, axisBY);

					p_outResult.x = p_indexA;
					p_outResult.y = p_indexB;

					m_proxyA.GetVertex(p_indexA, _pointHelper);
					localPointAX = _pointHelper.x;
					localPointAY = _pointHelper.y;

					m_proxyB.GetVertex(p_indexB, _pointHelper);
					localPointBX = _pointHelper.x;
					localPointBY = _pointHelper.y;

					b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					return b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), m_axisX, m_axisY);
				}

				case e_faceA:
				{
					b2Math.MulRV(xfA, m_axisX, m_axisY, _pointHelper);
					normalX = _pointHelper.x;
					normalY = _pointHelper.y;

					b2Math.MulTrV(xfA, m_localPointX, m_localPointY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					b2Math.MulTRV(xfB, -normalX, -normalY, _pointHelper);
					axisBX = _pointHelper.x;
					axisBY = _pointHelper.y;

					p_indexA = -1;
					p_indexB = m_proxyB.GetSupport(axisBX, axisBY);

					p_outResult.x = p_indexA;
					p_outResult.y = p_indexB;

					m_proxyB.GetVertex(p_indexB, _pointHelper);
					localPointBX = _pointHelper.x;
					localPointBY = _pointHelper.y;

					b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					return b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), normalX, normalY);
				}

				case e_faceB:
				{
					b2Math.MulRV(xfB, m_axisX, m_axisY, _pointHelper);
					normalX = _pointHelper.x;
					normalY = _pointHelper.y;

					b2Math.MulTrV(xfB, m_localPointX, m_localPointY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					b2Math.MulTRV(xfA, -normalX, -normalY, _pointHelper);
					axisAX = _pointHelper.x;
					axisAY = _pointHelper.y;

					p_indexA = m_proxyA.GetSupport(axisAX, axisAY);
					p_indexB = -1;

					p_outResult.x = p_indexA;
					p_outResult.y = p_indexB;

					m_proxyA.GetVertex(p_indexA, _pointHelper);
					localPointAX = _pointHelper.x;
					localPointAY = _pointHelper.y;

					b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					return b2Math.Dot((pointAX - pointBX), (pointAY - pointBY), normalX, normalY);
				}

				default:
					b2Assert(false, "default statement");
					p_outResult.x = -1;
					p_outResult.y = -1;

					return 0.0;
			}
		}

		/**
		 */
		public function Evaluate(p_indexA:int, p_indexB:int, p_t:Number):Number
		{
			var localPointAX:Number;
			var localPointAY:Number;
			var localPointBX:Number;
			var localPointBY:Number;
			var pointAX:Number;
			var pointAY:Number;
			var pointBX:Number;
			var pointBY:Number;
			var normalX:Number;
			var normalY:Number;

			m_sweepA.GetTransform(xfA, p_t);
			m_sweepB.GetTransform(xfB, p_t);

			switch (m_type)
			{
				case e_points:
				{
					m_proxyA.GetVertex(p_indexA, _pointHelper);

					localPointAX = _pointHelper.x;
					localPointAY = _pointHelper.y;

					m_proxyB.GetVertex(p_indexB, _pointHelper);
					localPointBX = _pointHelper.x;
					localPointBY = _pointHelper.y;

					b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					return b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), m_axisX, m_axisY);
				}

				case e_faceA:
				{

					b2Math.MulRV(xfA, m_axisX, m_axisY, _pointHelper);
					normalX = _pointHelper.x;
					normalY = _pointHelper.y;

					b2Math.MulTrV(xfA, m_localPointX, m_localPointY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					m_proxyB.GetVertex(p_indexB, _pointHelper);
					localPointBX = _pointHelper.x;
					localPointBY = _pointHelper.y;

					b2Math.MulTrV(xfB, localPointBX, localPointBY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					return b2Math.Dot((pointBX - pointAX), (pointBY - pointAY), normalX, normalY);
				}

				case e_faceB:
				{
					b2Math.MulRV(xfB, m_axisX, m_axisY, _pointHelper);
					normalX = _pointHelper.x;
					normalY = _pointHelper.y;

					b2Math.MulTRV(xfB, m_localPointX, m_localPointY, _pointHelper);
					pointBX = _pointHelper.x;
					pointBY = _pointHelper.y;

					m_proxyA.GetVertex(p_indexA, _pointHelper);
					localPointAX = _pointHelper.x;
					localPointAY = _pointHelper.y;

					b2Math.MulTrV(xfA, localPointAX, localPointAY, _pointHelper);
					pointAX = _pointHelper.x;
					pointAY = _pointHelper.y;

					return b2Math.Dot((pointAX - pointBX), (pointAY - pointBY), normalX, normalY);
				}

				default:
					b2Assert(false, "default statement");
					return 0.0;
			}
		}
	}
}
