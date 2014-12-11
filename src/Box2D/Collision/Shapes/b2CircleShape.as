/**
 * User: VirtualMaestro
 * Date: 01.12.2014
 * Time: 23:43
 */
package Box2D.Collision.Shapes
{
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2RayCastData;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.b2MassData;
	import Box2D.assert;

	CONFIG::debug
	{
		import avmplus.getQualifiedClassName;
	}

	use namespace b2internal;

	/**
	 * Represents circle shape.
	 */
	public class b2CircleShape extends b2Shape
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var m_pX:Number;
		public var m_pY:Number;

		/**
		 */
		public function b2CircleShape(p_radius:Number, p_x:Number = 0, p_y:Number = 0)
		{
			m_type = b2Shape.CIRCLE;
			m_radius = p_radius;
			m_pX = p_x;
			m_pY = p_y;
		}

		/**
		 * Init current instance with given (copy properties from given to current).
		 * @param p_shape b2CircleShape
		 */
		override public function SetTo(p_shape:b2Shape):void
		{
			CONFIG::debug
			{
				assert((p_shape is b2CircleShape), "given parameter has to be b2CircleShape class. Current instance has type: " + getQualifiedClassName(p_shape));
			}

			var circleShape:b2CircleShape = p_shape as b2CircleShape;
			m_pX = circleShape.m_pX;
			m_pX = circleShape.m_pX;
			m_radius = circleShape.m_radius;
		}

		/**
		 */
		override public function TestPoint(p_transform:b2Mat22, p_pointX:Number, p_pointY:Number):Boolean
		{
			var cos:Number = p_transform.c11;
			var sin:Number = p_transform.c12;

			var centerX:Number = p_transform.tx + (cos * m_pX - sin * m_pY);
			var centerY:Number = p_transform.ty + (sin * m_pX + cos * m_pY);

			var dX:Number = p_pointX - centerX;
			var dY:Number = p_pointY - centerY;

			return (dX * dX + dY * dY) <= (m_radius * m_radius);
		}

		/** Given a transform, compute the associated axis aligned bounding box for a child shape.
		 *  @param p_aabb returns the axis aligned box.
		 *  @param p_transform the world transform matrix of the shape.
		 *  @param p_childIndex the child shape
		 */
		override public function ComputeAABB(p_aabb:b2AABB, p_transform:b2Mat22, p_childIndex:int):void
		{
			var cos:Number = p_transform.c11;
			var sin:Number = p_transform.c12;
			
			var pX:Number = (cos * m_pX - sin * m_pY) + p_transform.tx;
			var pY:Number = (sin * m_pX + cos * m_pY) + p_transform.ty;

			p_aabb.lowerBoundX = pX - m_radius;
			p_aabb.lowerBoundY = pY - m_radius;
			p_aabb.upperBoundX = pX + m_radius;
			p_aabb.upperBoundY = pY + m_radius;
		}

		/**
		 */
		override public function ComputeMass(p_massData:b2MassData, p_density:Number):void
		{
			var dRadius:Number = m_radius*m_radius;
			var tMass:Number = p_density * b2Math.PI * dRadius;

			p_massData.mass = tMass;
			p_massData.centerX = m_pX;
			p_massData.centerY = m_pY;

			// inertia about the local origin
			p_massData.I = tMass * (0.5 * dRadius + (m_pX * m_pX + m_pY * m_pY));
		}

		/**
		 */
		[Inline]
		final override public function GetChildCount():int
		{
			return 1;
		}

		/**
		 */
		override public function RayCast(p_rayCastData:b2RayCastData, p_transform:b2Mat22, p_childIndex:int):Boolean
		{
			var cos:Number = p_transform.c11;
			var sin:Number = p_transform.c12;

			var positionX:Number = (cos * m_pX - sin * m_pY) + p_transform.tx;
			var positionY:Number = (sin * m_pX + cos * m_pY) + p_transform.ty;

			var sX:Number = p_rayCastData.startX - positionX;
			var sY:Number = p_rayCastData.startY - positionY;

			var b:Number = sX * sX + sY * sY - m_radius*m_radius;

			// Solve quadratic equation.
			var rX:Number = p_rayCastData.endX - p_rayCastData.startX;
			var rY:Number = p_rayCastData.endY - p_rayCastData.startY;

			var c:Number = sX * rX + sY * rY;
			var rr:Number = rX * rX + rY * rY;
			var sigma:Number = c*c - rr*b;

			// Check for negative discriminant and short segment.
			if (sigma < 0.0 || rr < b2Math.EPSILON)
			{
				return false;
			}

			// Find the point of intersection of the line with the circle.
			var a:Number = -(c + Math.sqrt(sigma));

			// Is the intersection point on the segment?
			if (a >= 0.0 && a <= p_rayCastData.maxFraction * rr)
			{
				a /= rr;
				p_rayCastData.fraction = a;
				var arX:Number = rX * a;
				var arY:Number = rY * a;
				var outX:Number = sX + arX;
				var outY:Number = sY + arY;

				var invLength:Number = 1.0 / Math.sqrt(outX*outX + outY*outY);
				p_rayCastData.normalX = outX * invLength;
				p_rayCastData.normalY = outY * invLength;

				return true;
			}

			return false;
		}

		/**
		 * Dispose the shape and back to pool.
		 */
		override public function Dispose():void
		{
			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns clone of current instance b2CircleShape.
		 * @return IDisposable
		 */
		override public function Clone():IDisposable
		{
			var circle:b2CircleShape = Get(m_radius, m_pX, m_pY);
			circle.m_radius = m_radius;
			circle.m_pX = m_pX;
			circle.m_pY = m_pY;

			return circle;
		}

		/**
		 * Returns new instance of b2CircleShape.
		 * @return b2CircleShape
		 */
		[Inline]
		static public function Get(p_radius:Number, p_x:Number = 0, p_y:Number = 0):b2CircleShape
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var circle:b2CircleShape;

			if (instance)
			{
				circle = instance as b2CircleShape;
				circle.m_radius = p_radius;
				circle.m_pX = p_x;
				circle.m_pY = p_y;
			}
			else circle = new b2CircleShape(p_radius, p_x, p_y);

			return circle;
		}
	}
}
