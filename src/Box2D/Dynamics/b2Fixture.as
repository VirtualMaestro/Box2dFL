/**
 * User: VirtualMaestro
 * Date: 26.11.2014
 * Time: 0:32
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.b2AABB;
	import Box2D.Collision.b2RayCastData;
	import Box2D.Common.IDisposable;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Def.b2FixtureDef;
	import Box2D.Dynamics.filter.b2Filter;
	import Box2D.assert;

	use namespace b2internal;

	/**
	 * A fixture is used to attach a shape to a body for collision detection. A fixture
	 * inherits its transform from its parent. Fixtures hold additional non-geometric data
	 * such as friction, collision filters, etc.
	 * Fixtures are created via b2Body::CreateFixture.
	 *
	 * TODO: Impl b2Fixture
	 */
	public class b2Fixture extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		/**
		 * Use for anu user data.
		 */
		public var userData:*;

		private var _friction:Number;
		private var _restitution:Number;
		b2internal var m_density:Number;
		private var _isSensor:Boolean;
		private var _filter:b2Filter;

		private var _body:b2Body;
		private var _shape:b2Shape;

		b2internal var m_next:b2Fixture;

		private var _proxyCount:int;

//		b2FixtureProxy* m_proxies; TODO:


		/**
		 */
		public function b2Fixture()
		{
			_friction = 0.0;
			_restitution = 0.0;
			m_density = 0.0;
			_isSensor = false;
			_filter = b2Filter.Get();
			_proxyCount = 0;
		}

		/**
		 */
		b2internal function Create(p_body:b2Body, p_def:b2FixtureDef):void
		{

		}

		b2internal function CreateProxies(p_broadPhase/*:b2BroadPhase*/, p_xf:b2Mat22):void
		{
			// TODO:
		}

		b2internal function Synchronize(p_broadPhase/*:b2BroadPhase*/, p_xf1:b2Mat22, p_xf2:b2Mat22):void
		{
			// TODO:
		}

		/**
		 * Get the type of the child shape.
		 * You can use this to down cast to the concrete shape.
		 * @return the shape type.
 		 */
		[Inline]
		final public function GetType():int
		{
			return _shape.GetType();
		}

		/**
		 * Get the child shape. You can modify the child shape, however you should not change the
		 * number of vertices because this will crash some collision caching mechanisms.
		 * Manipulating the shape may lead to non-physical behavior.
 		 */
		[Inline]
		final public function GetShape():b2Shape
		{
			return _shape;
		}

		/**
		 * Set if this fixture is a sensor.
		 */
		public function SetSensor(p_sensor:Boolean):void
		{
			// TODO:
		}

		/**
		 * Is this fixture a sensor (non-solid)?
		 * @return the true if the shape is a sensor.
 		 */
		[Inline]
		final public function IsSensor():Boolean
		{
			return _isSensor;
		}

		/**
		 * Set the contact filtering data.
		 * This will not update contacts until the next time step when either parent body is active and awake.
		 * This automatically calls Refilter.
 		 */
		public function SetFilter(p_filter:b2Filter):void
		{
			// TODO:
		}

		/**
		 * Get the contact filtering data.
		 */
		public function GetFilter():b2Filter
		{
			return _filter;
		}

		/**
		 * Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
		 */
		public function Refilter():void
		{
			// TODO:
		}

		/**
		 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
		 * @return the parent body.
		 */
		[Inline]
		final public function GetBody():b2Body
		{
			return _body;
		}

		/**
		 * Get the next fixture in the parent body's fixture list.
		 * @return the next shape.
 		 */
		[Inline]
		final public function GetNext():b2Fixture
		{
			return m_next;
		}

		/**
		 * Test a point for containment in this fixture.
		 * @param p_pX a X component of point in world coordinates.
		 * @param p_pY a Y component of point in world coordinates.
		 */
		public function TestPoint(p_pX:Number, p_pY:Number):Boolean
		{
			_shape.TestPoint(_body.GetTransform(), p_pX, p_pY);
		}

		/**
		 * Cast a ray against this shape.
		 * @param p_rayCastData - input/output the ray-cast data.
		 * @param p_childIndex
		 */
		public function RayCast(p_rayCastData:b2RayCastData, p_childIndex:int):Boolean
		{
			_shape.RayCast(p_rayCastData, _body.GetTransform(), p_childIndex);
		}

		/**
		 * Get the mass data for this fixture.
		 * The mass data is based on the density and the shape.
		 * The rotational inertia is about the shape's origin.
		 * This operation may be expensive.
		 * NOTICE! If p_massData isn't set method produces new instance of b2MassData.
		 */
		public function GetMassData(p_massData:b2MassData = null):b2MassData
		{
			if (p_massData == null)
			{
				p_massData = new b2MassData();
			}

			_shape.ComputeMass(p_massData, m_density);

			return p_massData;
		}

		/**
		 * Set the density of this fixture. This will _not_ automatically adjust the mass of the body.
		 * You must call b2Body::ResetMassData to update the body's mass.
		 */
		public function SetDensity(p_density:Number):void
		{
			CONFIG::debug
			{
				assert(p_density >= 0.0, "value for density has to be positive");
			}

			m_density = p_density;
		}

		/**
		 * Get the density of this fixture.
		 */
		[Inline]
		final public function GetDensity():Number
		{
			return m_density;
		}

		/**
		 * Get the coefficient of friction.
		 */
		[Inline]
		final public function GetFriction():Number
		{
			return _friction;
		}

		/**
		 * Set the coefficient of friction. This will _not_ change the friction of existing contacts.
		 */
		[Inline]
		final public function SetFriction(p_friction:Number):void
		{
			_friction = p_friction;
		}

		/**
		 * Get the coefficient of restitution.
		 */
		[Inline]
		final public function GetRestitution():Number
		{
			return _restitution;
		}

		/**
		 * Set the coefficient of restitution.
		 * This will _not_ change the restitution of existing contacts.
		 */
		[Inline]
		final public function SetRestitution(p_restitution:Number):void
		{
			_restitution = p_restitution;
		}

		/**
		 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
		 * If you need a more accurate AABB, compute it using the shape and the body transform.
 		 */
		public function GetAABB(p_childIndex:int):b2AABB
		{
			assert(p_childIndex >= 0 && p_childIndex < _proxyCount, "incorrect number of childIndex: " + p_childIndex);

		}

		/**
		 */
		override public function Clone():IDisposable
		{
			// TODO: Clone b2Fixture
			var fixture:b2Fixture = Get();
			return fixture;
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
		 * Returns new instance of b2Fixture.
		 * @return b2Fixture
		 */
		static public function Get():b2Fixture
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var fixture:b2Fixture;

			if (instance)
			{
				fixture = instance as b2Fixture;
			}
			else fixture = new b2Fixture();

			return fixture;
		}
	}
}
