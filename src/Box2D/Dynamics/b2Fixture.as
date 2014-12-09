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
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.filter.b2Filter;

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
		 */
		public function b2Fixture()
		{
		}

		/**
		 * Get the type of the child shape.
		 * You can use this to down cast to the concrete shape.
		 * @return the shape type.
 		 */
		public function GetType():int
		{
			// TODO: b2Shape.CIRCLE
		}

		/**
		 * Get the child shape. You can modify the child shape, however you should not change the
		 * number of vertices because this will crash some collision caching mechanisms.
		 * Manipulating the shape may lead to non-physical behavior.
 		 */
		public function GetShape():b2Shape
		{
			// TODO:
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
		public function IsSensor():Boolean
		{
			// TODO:
		}

		/**
		 * Set the contact filtering data.
		 * This will not update contacts until the next time step when either parent body is active and awake.
		 * This automatically calls Refilter.
 		 */
		public function SetFilterData(p_filter:b2Filter):void
		{
			// TODO:
		}

		/**
		 * Get the contact filtering data.
		 */
		public function GetFilterData():b2Filter
		{
			// TODO:
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
		public function GetBody()/*:b2Body*/
		{
			// TODO:
		}

		/**
		 * Get the next fixture in the parent body's fixture list.
		 * @return the next shape.
 		 */
		public function GetNext():b2Fixture
		{
			// TODO:
		}

		/**
		 * Get the user data that was assigned in the fixture definition.
		 * Use this to store your application specific data.
		 */
		public function GetUserData():*
		{
			// TODO:
		}

		/**
		 * Set the user data. Use this to store your application specific data.
		 */
		public function SetUserData(p_data:*):void
		{
			// TODO:
		}

		/**
		 * Test a point for containment in this fixture.
		 * @param p_pX a X component of point in world coordinates.
		 * @param p_pY a Y component of point in world coordinates.
		 */
		public function TestPoint(p_pX:Number, p_pY:Number):Boolean
		{
			// TODO:
		}

		/**
		 * Cast a ray against this shape.
		 * @param p_rayCastData - input/output the ray-cast data.
		 * @param p_childIndex
		 */
		public function RayCast(p_rayCastData:b2RayCastData, p_childIndex:int):Boolean
		{
			// TODO:
		}

		/**
		 * Get the mass data for this fixture.
		 * The mass data is based on the density and the shape.
		 * The rotational inertia is about the shape's origin.
		 * This operation may be expensive.
		 */
		public function GetMassData(p_massData:b2MassData):void
		{
			// TODO:
		}

		/**
		 * Set the density of this fixture. This will _not_ automatically adjust the mass of the body.
		 * You must call b2Body::ResetMassData to update the body's mass.
		 */
		public function SetDensity(p_density:Number):void
		{
			// TODO:
		}

		/**
		 * Get the density of this fixture.
		 */
		public function GetDensity():Number
		{
			// TODO:
		}

		/**
		 * Get the coefficient of friction.
		 */
		public function GetFriction():Number
		{
			// TODO:
		}

		/**
		 * Set the coefficient of friction. This will _not_ change the friction of existing contacts.
		 */
		public function SetFriction(p_friction:Number):void
		{
			// TODO:
		}

		/**
		 * Get the coefficient of restitution.
		 */
		public function GetRestitution():Number
		{
			// TODO:
		}

		/**
		 * Set the coefficient of restitution.
		 * This will _not_ change the restitution of existing contacts.
		 */
		public function SetRestitution(p_restitution:Number):void
		{
			// TODO:
		}

		/**
		 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
		 * If you need a more accurate AABB, compute it using the shape and the body transform.
 		 */
		public function GetAABB(p_childIndex:int):b2AABB
		{
			// TODO:
		}

		/**
		 */
		override public function Clone():IDisposable
		{
			// TODO:
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
