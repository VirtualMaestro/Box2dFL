/**
 * User: VirtualMaestro
 * Date: 09.12.2014
 * Time: 20:21
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Def.b2FixtureDef;

	use namespace b2internal;

	public class b2Body
	{
		static public const STATIC:uint = 0;
		static public const KINEMATIC:uint = 1;
		static public const DYNAMIC:uint = 2;

		b2internal var m_type:uint;

		b2internal var m_flags:uint;

		b2internal var m_islandIndex:int;

		b2internal var m_xf:b2Mat22;		// the body origin transform
		b2internal var m_sweep:b2Sweep;		// the swept motion for CCD

		b2internal var m_linearVelocityX:Number;
		b2internal var m_linearVelocityY:Number;
		b2internal var m_angularVelocity:Number;

		b2internal var m_forceX:Number;
		b2internal var m_forceY:Number;
		b2internal var m_torque:Number;

		b2internal var m_world/*:b2World*/; // TODO:
		b2internal var m_prev:b2Body;
		b2internal var m_next:b2Body;

		b2internal var m_fixtureList:b2Fixture;
		b2internal var m_fixtureCount:int;

		b2internal var m_jointList/*:b2JointEdge*/; // TODO:
		b2internal var m_contactList/*:b2ContactEdge*/; // TODO:

		b2internal var m_mass:Number;
		b2internal var m_invMass:Number;

		// Rotational inertia about the center of mass.
		b2internal var m_I:Number;
		b2internal var m_invI:Number;

		/**
		 * Linear damping of the body.
		 */
		public var linearDamping:Number;

		/**
		 * Angular damping of the body.
		 */
		public var angularDamping:Number;

		/**
		 * Gravity scale of the body
		 */
		public var gravityScale:Number;

		b2internal var m_sleepTime:Number;

		public var userData:*;

		/**
		 */
		public function b2Body()
		{
		}

		/**
		 * Creates a fixture and attach it to this body. Use this function if you need
		 * to set some fixture parameters, like friction. Otherwise you can create the
		 * fixture directly from a shape.
		 * If the density is non-zero, this function automatically updates the mass of the body.
		 * Contacts are not created until the next time step.
		 * @param p_def the fixture definition.
		 * @warning This function is locked during callbacks.
		 */
		public function CreateFixture(p_def:b2FixtureDef):b2Fixture
		{
			// TODO:
		}

		/**
		 * Creates a fixture from a shape and attach it to this body.
		 * This is a convenience function. Use b2FixtureDef if you need to set parameters
		 * like friction, restitution, user data, or filtering.
		 * If the density is non-zero, this function automatically updates the mass of the body.
		 * @param p_shape the shape to be cloned.
		 * @param p_density the shape density (set to zero for static bodies).
		 * @warning This function is locked during callbacks.
		 */
		public function CreateFixture2(p_shape:b2Shape, p_density:Number):b2Fixture
		{
			// TODO:
		}

		/**
		 * Destroy a fixture. This removes the fixture from the broad-phase and
		 * destroys all contacts associated with this fixture. This will
		 * automatically adjust the mass of the body if the body is dynamic and the
		 * fixture has positive density.
		 * All fixtures attached to a body are implicitly destroyed when the body is destroyed.
		 * @param p_fixture the fixture to be removed.
		 * @warning This function is locked during callbacks.
		 */
		public function DestroyFixture(p_fixture:b2Fixture):void
		{
			// TODO:
		}

		/**
		 * Set the position of the body's origin and rotation.
		 * Manipulating a body's transform may cause non-physical behavior.
		 * Note: contacts are updated on the next call to b2World::Step.
		 * @param p_posX the world position of the body's local origin.
		 * @param p_posY the world position of the body's local origin.
		 * @param p_angle the world rotation in radians.
		 */
		public function SetTransform(p_posX:Number, p_posY:Number, p_angle:Number):void
		{
			// TODO:
		}

		/**
		 * Get the body transform for the body's origin.
		 * @return the world transform of the body's origin.
		 */
		[Inline]
		final public function GetTransform():b2Mat22
		{
			return m_xf;
		}

		/**
		 * Get the world body origin position.
		 * @return the world position of the body's origin.
		 * NOTICE! If param for result wasn't found, produces new instance of b2Vec2.
		 */
		public function GetPosition(p_positionOut:b2Vec2 = null):b2Vec2
		{
			if (p_positionOut)
			{
				p_positionOut.x = m_xf.tx;
				p_positionOut.y = m_xf.ty;
			}
			else
			{
				p_positionOut = b2Vec2.Get(m_xf.tx, m_xf.ty);
			}

			return p_positionOut;
		}

		/**
		 * Get the angle in radians.
		 * @return the current world rotation angle in radians.
		 */
		[Inline]
		final public function GetAngle():Number
		{
			return m_sweep.worldAngle;
		}

		/**
		 * Get the world position of the center of mass.
		 * NOTICE! If param for result wasn't found, produces new instance of b2Vec2.
		 */
		public function GetWorldCenter(p_worldCenterOut:b2Vec2 = null):b2Vec2
		{
			if (p_worldCenterOut)
			{
				p_worldCenterOut.x = m_sweep.worldCenterX;
				p_worldCenterOut.y = m_sweep.worldCenterY;
			}
			else
			{
				p_worldCenterOut = b2Vec2.Get(m_sweep.worldCenterX, m_sweep.worldCenterY);
			}

			return p_worldCenterOut;
		}

		/**
		 * Get the local position of the center of mass.
		 * NOTICE! If param for result wasn't found, produces new instance of b2Vec2.
		 */
		public function GetLocalCenter(p_localCenterOut:b2Vec2 = null):b2Vec2
		{
			if (p_localCenterOut)
			{
				p_localCenterOut.x = m_sweep.localCenterX;
				p_localCenterOut.y = m_sweep.localCenterY;
			}
			else
			{
				p_localCenterOut = b2Vec2.Get(m_sweep.localCenterX, m_sweep.localCenterY);
			}

			return p_localCenterOut;
		}

		/**
		 * Set the linear velocity of the center of mass.
		 * @param p_vX the new linear velocity of the center of mass.
		 * @param p_vY the new linear velocity of the center of mass.
		 */
		public function SetLinearVelocity(p_vX:Number, p_vY:Number):void
		{
			if (m_type != STATIC)
			{
				if ((p_vX * p_vX + p_vY * p_vY) > 0.0)
				{
					SetAwake(true);
				}

				m_linearVelocityX = p_vX;
				m_linearVelocityY = p_vY;
			}
		}

		/**
		 * Get the linear velocity of the center of mass.
		 * @return the linear velocity of the center of mass.
		 * NOTICE! If param for result wasn't found, produces new instance of b2Vec2.
		 */
		public function GetLinearVelocity(p_linearVelocityOut:b2Vec2 = null):b2Vec2
		{
			if (p_linearVelocityOut)
			{
				p_linearVelocityOut.x = m_linearVelocityX;
				p_linearVelocityOut.y = m_linearVelocityY;
			}
			else
			{
				p_linearVelocityOut = b2Vec2.Get(m_linearVelocityX, m_linearVelocityY);
			}

			return p_linearVelocityOut;
		}

		/**
		 * Set the angular velocity.
		 * @param p_omega the new angular velocity in radians/second.
		 */
		public function SetAngularVelocity(p_omega:Number):void
		{
			if (m_type != STATIC)
			{
				if (p_omega * p_omega > 0.0)
				{
					SetAwake(true);
				}

				m_angularVelocity = p_omega;
			}
		}

		/**
		 * Get the angular velocity.
		 * @return the angular velocity in radians/second.
		 */
		public function GetAngularVelocity():Number
		{
			return m_angularVelocity;
		}

		/**
		 * Apply a force at a world point. If the force is not
		 * applied at the center of mass, it will generate a torque and
		 * affect the angular velocity. This wakes up the body.
		 * @param p_forceX the world force vector, usually in Newtons (N).
		 * @param p_forceY the world force vector, usually in Newtons (N).
		 * @param p_pointX the world position of the point of application.
		 * @param p_pointY the world position of the point of application.
		 * @param p_wake also wake up the body
		 */
		public function ApplyForce(p_forceX:Number, p_forceY:Number, p_pointX:Number, p_pointY:Number, p_wake:Boolean):void
		{
			// TODO:
		}

		/**
		 * Apply a force to the center of mass. This wakes up the body.
		 * @param p_forceX the world force vector, usually in Newtons (N).
		 * @param p_forceY the world force vector, usually in Newtons (N).
		 * @param p_wake also wake up the body
		 */
		public function ApplyForceToCenter(p_forceX:Number, p_forceY:Number, p_wake:Boolean):void
		{
			// TODO:
		}

		/**
		 * Apply a torque. This affects the angular velocity
		 * without affecting the linear velocity of the center of mass.
		 * This wakes up the body.
		 * @param p_torque about the z-axis (out of the screen), usually in N-m.
		 * @param p_wake also wake up the body
		 */
		public function ApplyTorque(p_torque:Number, p_wake:Boolean):void
		{
			// TODO:
		}

		/**
		 * Apply an impulse at a point. This immediately modifies the velocity.
		 * It also modifies the angular velocity if the point of application
		 * is not at the center of mass. This wakes up the body.
		 * @param p_impulseX the world impulse vector, usually in N-seconds or kg-m/s.
		 * @param p_impulseY the world impulse vector, usually in N-seconds or kg-m/s.
		 * @param p_pointX the world position of the point of application.
		 * @param p_pointY the world position of the point of application.
		 * @param p_wake also wake up the body
		 */
		public function ApplyLinearImpulse(p_impulseX:Number, p_impulseY:Number, p_pointX:Number, p_pointY:Number, p_wake:Boolean):void
		{
			// TODO:
		}

		/**
		 * Apply an angular impulse.
		 * @param p_impulse the angular impulse in units of kg*m*m/s
		 * @param p_wake also wake up the body
		 */
		public function ApplyAngularImpulse(p_impulse:Number, p_wake:Boolean):void
		{
			// TODO:
		}

		/**
		 * Get the total mass of the body.
		 * @return the mass, usually in kilograms (kg).
		 */
		[Inline]
		final public function GetMass():Number
		{
			return m_mass;
		}

		/**
		 * Get the rotational inertia of the body about the local origin.
		 * @return the rotational inertia, usually in kg-m^2.
		 */
		public function GetInertia():Number
		{
			var lcX:Number = m_sweep.localCenterX;
			var lcY:Number = m_sweep.localCenterY;

			return m_I + m_mass * (lcX * lcX + lcY * lcY);
		}

		/**
		 * Get the mass data of the body.
		 * @return containing the mass, inertia and center of the body.
		 * NOTICE! If param for result wasn't found, produces new instance of b2MassData.
		 */
		public function GetMassData(p_data:b2MassData = null):b2MassData
		{
			var lcX:Number = m_sweep.localCenterX;
			var lcY:Number = m_sweep.localCenterY;

			if (p_data == null)
			{
				p_data = new b2MassData();
			}

			p_data.mass = m_mass;
			p_data.I = m_I + m_mass * (lcX * lcX + lcY * lcY);
			p_data.centerX = lcX;
			p_data.centerY = lcY;
		}

		/**
		 * Set the mass properties to override the mass properties of the fixtures.
		 * Note that this changes the center of mass position.
		 * Note that creating or destroying fixtures can also alter the mass.
		 * This function has no effect if the body isn't dynamic.
		 * @param p_data the mass properties.
		 */
		public function SetMassData(p_data:b2MassData):void
		{
			// TODO:
		}

		/**
		 * This resets the mass properties to the sum of the mass properties of the fixtures.
		 * This normally does not need to be called unless you called SetMassData to override
		 * the mass and you later want to reset the mass.
		 */
		public function ResetMassData():void
		{
			// TODO:
		}

		/**
		 * Get the world coordinates of a point given the local coordinates.
		 * @param p_localPointX a point on the body measured relative the the body's origin.
		 * @param p_localPointY a point on the body measured relative the the body's origin.
		 * @return the same point expressed in world coordinates.
		 * NOTICE! Produces new instance of b2Vec2.
		 */
		public function GetWorldPoint(p_localPointX:Number, p_localPointY:Number):b2Vec2
		{
			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;
			
			var rX:Number = (cos * p_localPointX - sin * p_localPointY) + m_xf.tx;
			var rY:Number = (sin * p_localPointX + cos * p_localPointY) + m_xf.ty;

			return b2Vec2.Get(rX, rY);
		}

		/**
		 * Get the world coordinates of a vector given the local coordinates.
		 * @param p_localVectorX a vector fixed in the body.
		 * @param p_localVectorY a vector fixed in the body.
		 * @return the same vector expressed in world coordinates.
		 */
		public function GetWorldVector(p_localVectorX:Number, p_localVectorY:Number):b2Vec2
		{
			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;

			var rX:Number = (cos * p_localVectorX - sin * p_localVectorY);
			var rY:Number = (sin * p_localVectorX + cos * p_localVectorY);

			return b2Vec2.Get(rX, rY);
		}

		/**
		 * Gets a local point relative to the body's origin given a world point.
		 * @param p_worldPointX point in world coordinates.
		 * @param p_worldPointY point in world coordinates.
		 * @return the corresponding local point relative to the body's origin.
		 * NOTICE! Produces new instance of b2Vec2.
		 */
		public function GetLocalPoint(p_worldPointX:Number, p_worldPointY:Number):b2Vec2
		{
			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;

			var px:Number = p_worldPointX - m_xf.tx;
			var py:Number = p_worldPointY - m_xf.ty;

			var rX:Number =  cos * px + sin * py;
			var rY:Number = -sin * px + cos * py;

			return b2Vec2.Get(rX, rY);
		}

		/**
		 * Gets a local vector given a world vector.
		 * @param p_worldVectorX vector in world coordinates.
		 * @param p_worldVectorY vector in world coordinates.
		 * @return the corresponding local vector.
		 * NOTICE! Produces new instance of b2Vec2.
		 */
		public function GetLocalVector(p_worldVectorX:Number, p_worldVectorY:Number):b2Vec2
		{
			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;

			var rX:Number =  cos * p_worldVectorX + sin * p_worldVectorY;
			var rY:Number = -sin * p_worldVectorX + cos * p_worldVectorY;

			return b2Vec2.Get(rX, rY);
		}

		/**
		 * Get the world linear velocity of a world point attached to this body.
		 * @param p_worldPointX point in world coordinates.
		 * @param p_worldPointY point in world coordinates.
		 * @return the world velocity of a point.
		 * NOTICE! Produces new instance of b2Vec2.
		 */
		public function GetLinearVelocityFromWorldPoint(p_worldPointX:Number, p_worldPointY:Number):b2Vec2
		{
			var rX:Number = p_worldPointX - m_sweep.worldCenterX;
			var rY:Number = p_worldPointY - m_sweep.worldCenterY;

			var r1X:Number = -m_angularVelocity * rY;
			var r1Y:Number =  m_angularVelocity * rX;

			var r2X:Number = m_linearVelocityX + r1X;
			var r2Y:Number = m_linearVelocityY + r1Y;

			return b2Vec2.Get(r2X, r2Y);
		}

		/**
		 * Get the world velocity of a local point.
		 * @param p_localPointX point in local coordinates.
		 * @param p_localPointY point in local coordinates.
		 * @return the world velocity of a point.
		 * NOTICE! Produces new instance of b2Vec2.
		 */
		public function GetLinearVelocityFromLocalPoint(p_localPointX:Number, p_localPointY:Number):b2Vec2
		{
			var wp:b2Vec2 = GetWorldPoint(p_localPointX, p_localPointY);
			var lv:b2Vec2 = GetLinearVelocityFromWorldPoint(wp.x, wp.y);
			wp.Dispose();

			return lv;
		}

		/**
		 * Set the type of this body. This may alter the mass and velocity.
		 * @param p_type - body type. E.g. b2Body.STATIC
		 */
		public function SetType(p_type:uint):void
		{
			// TODO:
		}

		/**
		 * Get the type of this body.
		 */
		[Inline]
		final public function GetType():uint
		{
			return m_type;
		}

		/**
		 * Should this body be treated like a bullet for continuous collision detection?
		 * @param p_flag
		 */
		public function SetBullet(p_flag:Boolean):void
		{
			// TODO:
		}

		/**
		 * Is this body treated like a bullet for continuous collision detection?
		 */
		public function IsBullet():Boolean
		{
			// TODO:
		}

		/**
		 * You can disable sleeping on this body. If you disable sleeping, the body will be woken.
		 */
		public function SetSleepingAllowed(p_flag:Boolean):void
		{
			// TODO:
		}

		/**
		 * Is this body allowed to sleep
		 * @return Boolean
		 */
		public function IsSleepingAllowed():Boolean
		{
			// TODO:
		}

		/**
		 * Set the sleep state of the body. A sleeping body has very low CPU cost.
		 * @param p_flag set to true to wake the body, false to put it to sleep.
		 */
		public function SetAwake(p_flag:Boolean):void
		{
			// TODO:
		}

		/**
		 * Get the sleeping state of this body.
		 * @return true if the body is awake.
		 */
		public function IsAwake():Boolean
		{
			// TODO:
		}

		/**
		 * Set the active state of the body. An inactive body is not
		 * simulated and cannot be collided with or woken up.
		 * If you pass a flag of true, all fixtures will be added to the
		 * broad-phase.
		 * If you pass a flag of false, all fixtures will be removed from
		 * the broad-phase and all contacts will be destroyed.
		 * Fixtures and joints are otherwise unaffected. You may continue
		 * to create/destroy fixtures and joints on inactive bodies.
		 * Fixtures on an inactive body are implicitly inactive and will
		 * not participate in collisions, ray-casts, or queries.
		 * Joints connected to an inactive body are implicitly inactive.
		 * An inactive body is still owned by a b2World object and remains
		 * in the body list.
		 */
		public function SetActive(p_flag:Boolean):void
		{
			// TODO:
		}

		/**
		 * Get the active state of the body.
		 * @return Boolean
		 */
		public function IsActive():Boolean
		{
			// TODO:
		}

		/**
		 * Set this body to have fixed rotation. This causes the mass to be reset.
		 * @param p_flag
		 */
		public function SetFixedRotation(p_flag:Boolean):void
		{
			// TODO:
		}

		/**
		 * Does this body have fixed rotation?
		 * @return Boolean
		 */
		public function IsFixedRotation():Boolean
		{
			// TODO:
		}

		/// Get the list of all fixtures attached to this body.
		public function GetFixtureList():b2Fixture
		{
			// TODO:
		}

		/**
		 * Get the list of all joints attached to this body.
		 * @return b2JointEdge
		 */
		public function GetJointList()/*:b2JointEdge*/
		{
			// TODO:
		}

		/**
		 * Get the list of all contacts attached to this body.
		 * @warning this list changes during the time step and you may
		 * miss some collisions if you don't use b2ContactListener.
		 */
		public function GetContactList()/*:b2ContactEdge*/
		{
			// TODO:
		}

		/**
		 * Get the next body in the world's body list.
		 * @return b2Body
		 */
		public function GetNext():b2Body
		{
			// TODO:
		}

		/**
		 * Get the parent world of this body.
		 * @return b2World
		 */
		public function GetWorld()/*:b2World*/
		{
			// TODO:
		}

		/**
		 *
		 */
		public function SynchronizeFixtures():void
		{
			// TODO:
		}

		/**
		 *
		 */
		public function SynchronizeTransform():void
		{
			// TODO:
		}

		/**
		 * This is used to prevent connected bodies from colliding.
		 * It may lie, depending on the collideConnected flag.
		 * @param p_other
		 * @return
		 */
		public function ShouldCollide(p_other:b2Body):Boolean
		{
			//TODO:
		}

		/**
		 *
		 * @param p_t
		 */
		public function Advance(p_t:Number):void
		{
			// TODO:
		}

	}
}
