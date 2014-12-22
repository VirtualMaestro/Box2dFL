/**
 * User: VirtualMaestro
 * Date: 13.12.2014
 * Time: 15:35
 */
package Box2D.Collision.Shapes
{
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.Structures.b2RayCastData;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;

	use namespace b2internal;

	/**
	 * A line segment (edge) shape. These can be connected in chains or loops
	 * to other edge shapes. The connectivity information is used to ensure
	 * correct contact normals.
	 */
	public class b2EdgeShape extends b2Shape
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		/// These are the edge vertices
		b2internal var m_vertex1X:Number;
		b2internal var m_vertex1Y:Number;

		b2internal var m_vertex2X:Number;
		b2internal var m_vertex2Y:Number;

		/// Optional adjacent vertices. These are used for smooth collision.
		b2internal var m_vertex0X:Number;
		b2internal var m_vertex0Y:Number;

		b2internal var m_vertex3X:Number;
		b2internal var m_vertex3Y:Number;

		b2internal var m_hasVertex0:Boolean;
		b2internal var m_hasVertex3:Boolean;

		/**
		 */
		public function b2EdgeShape()
		{
			m_type = b2Shape.EDGE;
			m_radius = b2Settings.polygonRadius;

			init();
		}

		/**
		 */
		[Inline]
		final private function init():void
		{
			m_vertex0X = 0.0;
			m_vertex0Y = 0.0;
			m_vertex3X = 0.0;
			m_vertex3Y = 0.0;
			m_hasVertex0 = false;
			m_hasVertex3 = false;
		}

		/**
		 */
		override public function SetTo(p_shape:b2Shape):void
		{
			var edge:b2EdgeShape = p_shape as b2EdgeShape;
			m_vertex1X = edge.m_vertex1X;
			m_vertex1Y = edge.m_vertex1Y;
			m_vertex2X = edge.m_vertex2X;
			m_vertex2Y = edge.m_vertex2Y;
			m_vertex0X = edge.m_vertex0X;
			m_vertex0Y = edge.m_vertex0Y;
			m_vertex3X = edge.m_vertex3X;
			m_vertex3Y = edge.m_vertex3Y;
			m_hasVertex0 = edge.m_hasVertex0;
			m_hasVertex3 = edge.m_hasVertex3;
		}

		/**
		 */
		override public function GetChildCount():int
		{
			return 1;
		}

		/**
		 *
		 * @param p_rayCastData
		 * @param p_xf
         * @param p_childIndex
		 * @return
		 */
		override public function RayCast(p_rayCastData:b2RayCastData, p_xf:b2Mat22, p_childIndex:int):Boolean
		{
			// Put the ray into the edge's frame of reference.
			var rX:Number = p_rayCastData.startX - p_xf.tx;
			var rY:Number = p_rayCastData.startY - p_xf.ty;
			var cos:Number = p_xf.c11;
			var sin:Number = p_xf.c12;
			
			var p1X:Number =  cos * rX + sin * rY;
			var p1Y:Number = -sin * rX + cos * rY;

			rX = p_rayCastData.endX - p_xf.tx;
			rY = p_rayCastData.endY - p_xf.ty;

			var p2X:Number =  cos * rX + sin * rY;
			var p2Y:Number = -sin * rX + cos * rY;

			var dX:Number = p2X - p1X;
			var dY:Number = p2Y - p1Y;

			var normalX:Number = m_vertex2X - m_vertex1X;
			var normalY:Number = m_vertex2Y - m_vertex1Y;
			rX = normalX;
			normalX = normalY;
			normalY = -rX;

			var invLength:Number = 1.0 / Math.sqrt(normalX*normalX + normalY*normalY);
			normalX *= invLength;
			normalY *= invLength;

			// q = p1 + t * d
			// dot(normal, q - v1) = 0
			// dot(normal, p1 - v1) + t * dot(normal, d) = 0
			rX = m_vertex1X - p1X;
			rY = m_vertex1Y - p1Y;

			var numerator:Number = normalX * rX + normalY * rY;
			var denominator:Number = normalX * dX + normalY * dY;

			if (denominator == 0)
			{
				return false;
			}

			var t:Number = numerator / denominator;

			if (t < 0 || p_rayCastData.maxFraction < t)
			{
				return false;
			}

			var qX:Number = p1X + dX*t;
			var qY:Number = p1Y + dY*t;

			// q = v1 + s * r
			// s = dot(q - v1, r) / dot(r, r)
			rX = m_vertex2X - m_vertex1X;
			rY = m_vertex2Y - m_vertex1Y;

			var rr:Number = rX * rX + rY * rY;

			if (rr == 0)
			{
				return false;
			}

			var qv1X:Number = qX - m_vertex1X;
			var qv1Y:Number = qY - m_vertex1Y;
			var s:Number = (qv1X * rX + qv1Y * rY) / rr;

			if (s < 0 || s > 1)
			{
				return false;
			}

			p_rayCastData.fraction = t;

			rX = cos * normalX - sin * normalY;
			rY = sin * normalX + cos * normalY;

			if (numerator > 0)
			{
				p_rayCastData.normalX = -rX;
				p_rayCastData.normalY = -rY;
			}
			else
			{
				p_rayCastData.normalX = rX;
				p_rayCastData.normalY = rY;
			}

			return true;
		}

		/**
		 *
		 * @param p_aabb
		 * @param p_xf
         * @param p_childIndex
		 */
		override public function ComputeAABB(p_aabb:b2AABB, p_xf:b2Mat22, p_childIndex:int):void
		{
			var cos:Number = p_xf.c11;
			var sin:Number = p_xf.c12;

			var tX:Number = p_xf.tx;
			var tY:Number = p_xf.ty;
			var v1X:Number = (cos * m_vertex1X - sin * m_vertex1Y) + tX;
			var v1Y:Number = (sin * m_vertex1X + cos * m_vertex1Y) + tY;
			var v2X:Number = (cos * m_vertex2X - sin * m_vertex2Y) + tX;
			var v2Y:Number = (sin * m_vertex2X + cos * m_vertex2Y) + tY;

			var lowerX:Number = b2Math.Min(v1X, v2X);
			var lowerY:Number = b2Math.Min(v1Y, v2Y);
			var upperX:Number = b2Math.Max(v1X, v2X);
			var upperY:Number = b2Math.Max(v1Y, v2Y);

			var rX:Number = m_radius;
			var rY:Number = m_radius;

			p_aabb.lowerBoundX = lowerX - rX;
			p_aabb.lowerBoundY = lowerY - rY;

			p_aabb.upperBoundX = upperX + rX;
			p_aabb.upperBoundY = upperY + rY;
		}

		/**
		 *
		 * @param p_massData
		 * @param p_density
		 */
		override public function ComputeMass(p_massData:b2MassData, p_density:Number):void
		{
			p_massData.mass = 0;
			p_massData.centerX = 0.5 * (m_vertex1X + m_vertex2X);
			p_massData.centerY = 0.5 * (m_vertex1Y + m_vertex2Y);
			p_massData.I = 0;
		}

		/**
		 */
		override public function TestPoint(p_transform:b2Mat22, p_pointX:Number, p_pointY:Number):Boolean
		{
			return false;
		}

		/**
		 */
		override public function Clone():IDisposable
		{
			var edge:b2EdgeShape = Get();
			edge.SetTo(this);

			return edge;
		}

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns new instance of b2EdgeShape.
		 * @return b2EdgeShape
		 */
		static public function Get():b2EdgeShape
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var edge:b2EdgeShape;

			if (instance)
			{
				edge = instance as b2EdgeShape;
				edge.init();
			}
			else edge = new b2EdgeShape();

			return edge;
		}
	}
}
