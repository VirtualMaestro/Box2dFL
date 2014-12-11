/**
 * User: VirtualMaestro
 * Date: 09.12.2014
 * Time: 20:21
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2ContactEdge;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Def.b2FixtureDef;
	import Box2D.assert;

	use namespace b2internal;

	/**
	 * A rigid body.
	 * These are created via b2World.CreateBody().
	 * TODO: Implement b2Disposable
	 */
	public class b2Body
	{
		// Body type
		static public const STATIC:uint = 0;
		static public const KINEMATIC:uint = 1;
		static public const DYNAMIC:uint = 2;

		// Flags
		static b2internal const	e_islandFlag:uint = 0x0001;
		static b2internal const e_awakeFlag:uint = 0x0002;
		static b2internal const e_autoSleepFlag:uint = 0x0004;
		static b2internal const	e_bulletFlag:uint = 0x0008;
		static b2internal const e_fixedRotationFlag:uint = 0x0010;
		static b2internal const e_activeFlag:uint = 0x0020;
		static b2internal const	e_toiFlag:uint = 0x0040;


		//
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

		b2internal var m_world:b2World;
		b2internal var m_prev:b2Body;
		b2internal var m_next:b2Body;

		b2internal var m_fixtureList:b2Fixture;
		b2internal var m_fixtureCount:int;

		b2internal var m_jointList/*:b2JointEdge*/; // TODO:
		b2internal var m_contactList:b2ContactEdge;

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
			if (m_type == DYNAMIC)
			{
				if (p_wake && ((m_flags & e_awakeFlag) == 0))
				{
					SetAwake(true);
				}

				// Don't accumulate a force if the body is sleeping.
				if ((m_flags & e_awakeFlag) != 0)
				{
					m_forceX += p_forceX;
					m_forceY += p_forceY;

					var rX:Number = p_pointX - m_sweep.worldCenterX;
					var rY:Number = p_pointY - m_sweep.worldCenterY;

					m_torque += (rX * p_forceY - rY * p_forceX);
				}
			}
		}

		/**
		 * Apply a force to the center of mass. This wakes up the body.
		 * @param p_forceX the world force vector, usually in Newtons (N).
		 * @param p_forceY the world force vector, usually in Newtons (N).
		 * @param p_wake also wake up the body
		 */
		public function ApplyForceToCenter(p_forceX:Number, p_forceY:Number, p_wake:Boolean):void
		{
			if (m_type == DYNAMIC)
			{
				if (p_wake && ((m_flags & e_awakeFlag) == 0))
				{
					SetAwake(true);
				}

				// Don't accumulate a force if the body is sleeping.
				if ((m_flags & e_awakeFlag) != 0)
				{
					m_forceX += p_forceX;
					m_forceY += p_forceY;
				}
			}
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
			if (m_type == DYNAMIC)
			{
				if (p_wake && ((m_flags & e_awakeFlag) == 0))
				{
					SetAwake(true);
				}

				// Don't accumulate a torque if the body is sleeping.
				if ((m_flags & e_awakeFlag) != 0)
				{
					m_torque += p_torque;
				}
			}
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
			if (m_type == DYNAMIC)
			{
				if (p_wake && ((m_flags & e_awakeFlag) == 0))
				{
					SetAwake(true);
				}

				// Don't accumulate a velocity if the body is sleeping.
				if ((m_flags & e_awakeFlag) != 0)
				{
					m_linearVelocityX += m_invMass * p_impulseX;
					m_linearVelocityY += m_invMass * p_impulseY;

					var rX:Number = p_pointX - m_sweep.worldCenterX;
					var rY:Number = p_pointY - m_sweep.worldCenterY;

					m_angularVelocity += m_invI * (rX * p_impulseY - rY * p_impulseX);
				}
			}
		}

		/**
		 * Apply an angular impulse.
		 * @param p_impulse the angular impulse in units of kg*m*m/s
		 * @param p_wake also wake up the body
		 */
		public function ApplyAngularImpulse(p_impulse:Number, p_wake:Boolean):void
		{
			if (m_type == DYNAMIC)
			{
				if (p_wake && ((m_flags & e_awakeFlag) == 0))
				{
					SetAwake(true);
				}

				// Don't accumulate a velocity if the body is sleeping.
				if ((m_flags & e_awakeFlag) != 0)
				{
					m_angularVelocity += m_invI * p_impulse;
				}
			}
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
			m_mass = 0.0;
			m_invMass = 0.0;
			m_I = 0.0;
			m_invI = 0.0;
			m_sweep.localCenterX = 0.0;
			m_sweep.localCenterY = 0.0;

			// Static and kinematic bodies have zero mass.
			if (m_type == STATIC || m_type == KINEMATIC)
			{
				m_sweep.worldCenterX0 = m_xf.tx;
				m_sweep.worldCenterY0 = m_xf.ty;

				m_sweep.worldCenterX = m_xf.tx;
				m_sweep.worldCenterY = m_xf.ty;

				m_sweep.worldAngle0 = m_sweep.worldAngle;
			}

			CONFIG::debug
			{
				assert((m_type == DYNAMIC), "can't apply reset mass for non dynamic body");
			}

			var localCenterX:Number = 0;
			var localCenterY:Number = 0;
			var massData:b2MassData = new b2MassData();

			for (var f:b2Fixture = m_fixtureList; f; f = f.m_next)
			{
				if (f.m_density > 0.0)
				{
					f.GetMassData(massData);
					m_mass += massData.mass;
					localCenterX += massData.mass * massData.centerX;
					localCenterY += massData.mass * massData.centerY;
					m_I += massData.I;
				}
			}

			massData = null;

			// Compute center of mass.
			if (m_mass > 0.0)
			{
				m_invMass = 1.0 / m_mass;
				localCenterX *= m_invMass;
				localCenterY *= m_invMass;
			}
			else
			{
				// Force all dynamic bodies to have a positive mass.
				m_mass = 1.0;
				m_invMass = 1.0;
			}

			if (m_I > 0.0 && (m_flags & e_fixedRotationFlag) == 0)
			{
				// Center the inertia about the center of mass.
				m_I -= m_mass * (localCenterX * localCenterX + localCenterY * localCenterY);

				CONFIG::debug
				{
					assert(m_I > 0.0, "rotational inertia should be greater then 0");
				}

				m_invI = 1.0 / m_I;
			}
			else
			{
				m_I = 0.0;
				m_invI = 0.0;
			}

			// Move center of mass.
			var oldCenterX:Number = m_sweep.worldCenterX;
			var oldCenterY:Number = m_sweep.worldCenterY;
			m_sweep.localCenterX = localCenterX;
			m_sweep.localCenterY = localCenterY;

			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;

			var rX:Number = (cos * m_sweep.localCenterX - sin * m_sweep.localCenterY) + m_xf.tx;
			var rY:Number = (sin * m_sweep.localCenterX + cos * m_sweep.localCenterY) + m_xf.ty;

			m_sweep.worldCenterX = rX;
			m_sweep.worldCenterY = rY;
			m_sweep.worldCenterX0 = rX;
			m_sweep.worldCenterY0 = rY;

			// Update center of mass velocity.
			var r1X:Number = m_sweep.worldCenterX - oldCenterX;
			var r1Y:Number = m_sweep.worldCenterY - oldCenterY;
			
			m_linearVelocityX += -m_angularVelocity * r1Y ;
			m_linearVelocityY +=  m_angularVelocity * r1X;
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
			if (p_flag)
			{
				m_flags |= e_bulletFlag;
			}
			else
			{
				m_flags &= ~e_bulletFlag;
			}
		}

		/**
		 * Is this body treated like a bullet for continuous collision detection?
		 */
		public function IsBullet():Boolean
		{
			return (m_flags & e_bulletFlag) == e_bulletFlag;
		}

		/**
		 * You can disable sleeping on this body. If you disable sleeping, the body will be woken.
		 */
		public function SetSleepingAllowed(p_flag:Boolean):void
		{
			if (p_flag)
			{
				m_flags |= e_autoSleepFlag;
			}
			else
			{
				m_flags &= ~e_autoSleepFlag;
				SetAwake(true);
			}
		}

		/**
		 * Is this body allowed to sleep
		 * @return Boolean
		 */
		public function IsSleepingAllowed():Boolean
		{
			return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
		}

		/**
		 * Set the sleep state of the body. A sleeping body has very low CPU cost.
		 * @param p_flag set to true to wake the body, false to put it to sleep.
		 */
		public function SetAwake(p_flag:Boolean):void
		{
			if (p_flag)
			{
				if ((m_flags & e_awakeFlag) == 0)
				{
					m_flags |= e_awakeFlag;
					m_sleepTime = 0.0;
				}
			}
			else
			{
				m_flags &= ~e_awakeFlag;
				m_sleepTime = 0.0;
				m_linearVelocityX = 0.0;
				m_linearVelocityY = 0.0;
				m_angularVelocity = 0.0;
				m_forceX = 0.0;
				m_forceY = 0.0;
				m_torque = 0.0;
			}
		}

		/**
		 * Get the sleeping state of this body.
		 * @return true if the body is awake.
		 */
		public function IsAwake():Boolean
		{
			return (m_flags & e_awakeFlag) == e_awakeFlag;
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
			return (m_flags & e_activeFlag) == e_activeFlag;
		}

		/**
		 * Set this body to have fixed rotation. This causes the mass to be reset.
		 * @param p_flag
		 */
		public function SetFixedRotation(p_flag:Boolean):void
		{
			if (((m_flags & e_fixedRotationFlag) == e_fixedRotationFlag) != p_flag)
			{
				if (p_flag)
				{
					m_flags |= e_fixedRotationFlag;
				}
				else
				{
					m_flags &= ~e_fixedRotationFlag;
				}

				m_angularVelocity = 0.0;

				ResetMassData();
			}
		}

		/**
		 * Does this body have fixed rotation?
		 * @return Boolean
		 */
		public function IsFixedRotation():Boolean
		{
			return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
		}

		/// Get the list of all fixtures attached to this body.
		public function GetFixtureList():b2Fixture
		{
			return m_fixtureList;
		}

		/**
		 * Get the list of all joints attached to this body.
		 * @return b2JointEdge
		 */
		public function GetJointList()/*:b2JointEdge TODO:*/
		{
			return m_jointList;
		}

		/**
		 * Get the list of all contacts attached to this body.
		 * @warning this list changes during the time step and you may
		 * miss some collisions if you don't use b2ContactListener.
		 */
		public function GetContactList():b2ContactEdge
		{
			return m_contactList;
		}

		/**
		 * Get the next body in the world's body list.
		 * @return b2Body
		 */
		public function GetNext():b2Body
		{
			return m_next;
		}

		/**
		 * Get the parent world of this body.
		 * @return b2World
		 */
		public function GetWorld():b2World
		{
			return m_world;
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
		[Inline]
		final public function SynchronizeTransform():void
		{
			m_xf.SetAngle(m_sweep.worldAngle);

			var cos:Number = m_xf.c11;
			var sin:Number = m_xf.c12;

			var lcX:Number = m_sweep.localCenterX;
			var lcY:Number = m_sweep.localCenterY;

			m_xf.tx = m_sweep.worldCenterX - (cos * lcX - sin * lcY);
			m_xf.ty = m_sweep.worldCenterY - (sin * lcX + cos * lcY);
		}

		/**
		 * This is used to prevent connected bodies from colliding.
		 * It may lie, depending on the collideConnected flag.
		 * @param p_other
		 * @return
		 */
		public function ShouldCollide(p_other:b2Body):Boolean
		{
			// At least one body should be dynamic.
			if (m_type != DYNAMIC && p_other.m_type != DYNAMIC)
			{
				return false;
			}

			// Does a joint prevent collision? TODO: When joint will implemented
//			for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
//			{
//				if (jn->other == other)
//				{
//					if (jn->joint->m_collideConnected == false)
//					{
//						return false;
//					}
//				}
//			}

			return true;

		}

		/**
		 *
		 * @param p_t
		 */
		public function Advance(p_t:Number):void
		{
			// Advance to the new safe time. This doesn't sync the broad-phase.
			m_sweep.Advance(p_t);

			m_sweep.worldCenterX = m_sweep.worldCenterX0;
			m_sweep.worldCenterY = m_sweep.worldCenterY0;
			m_sweep.worldAngle = m_sweep.worldAngle0;
			m_xf.SetAngle(m_sweep.worldAngle);

			SynchronizeTransform();
		}
	}
}
