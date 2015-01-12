/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 0:03
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Contact.b2ContactSolver;
	import Box2D.Collision.Structures.b2ContactImpulse;
	import Box2D.Collision.Structures.b2ContactVelocityConstraint;
	import Box2D.Collision.Structures.b2SolverData;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Collision.Structures.b2VelocityConstraintPoint;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.Common.Math.b2Vec2;
	import Box2D.Common.Math.b2Vec3;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Callbacks.b2ContactListener;
	import Box2D.Dynamics.Def.b2ContactSolverDef;
	import Box2D.Dynamics.Joints.b2Joint;
	import Box2D.Dynamics.b2Body;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 * This is an internal class.
	 */
	internal class b2Island
	{
		b2internal var m_listener:b2ContactListener;

		b2internal var m_bodies:Vector.<b2Body>;
		b2internal var m_contacts:Vector.<b2Contact>;
		b2internal var m_joints:Vector.<b2Joint>;

		b2internal var m_positions:Vector.<b2Vec3>;
		b2internal var m_velocities:Vector.<b2Vec3>;

		b2internal var m_bodyCount:int;
		b2internal var m_jointCount:int;
		b2internal var m_contactCount:int;

		b2internal var m_bodyCapacity:int;
		b2internal var m_contactCapacity:int;
		b2internal var m_jointCapacity:int;

		private var _helperImpulse:b2ContactImpulse;
		private var _solverDataHelper:b2SolverData;
		private var _contactSolverDefHelper:b2ContactSolverDef;
		private var _contactSolver:b2ContactSolver;

		/**
		 */
		public function b2Island()
		{
			m_bodies = new <b2Body>[];
			m_contacts = new <b2Contact>[];
			m_joints = new <b2Joint>[];
			m_positions = new <b2Vec3>[];
			m_velocities = new <b2Vec3>[];

			m_bodyCount = 0;
			m_jointCount = 0;
			m_contactCount = 0;
			m_bodyCapacity = 0;
			m_contactCapacity = 0;
			m_jointCapacity = 0;

			_helperImpulse = new b2ContactImpulse();
			_solverDataHelper = new b2SolverData();
			_contactSolver = new b2ContactSolver();
		}

		/**
		 */
		public function Initializer(p_bodyCapacity:int, p_contactCapacity:int, p_jointCapacity:int, p_listener:b2ContactListener):void
		{
			m_bodyCapacity = p_bodyCapacity;
			m_contactCapacity = p_contactCapacity;
			m_jointCapacity = p_jointCapacity;

			m_bodyCount = 0;
			m_contactCount = 0;
			m_jointCount = 0;

			for (var i:int = m_positions.length; i < p_bodyCapacity; i++)
			{
				m_positions[i] = b2Vec3.Get();
				m_velocities[i] = b2Vec3.Get();
			}

			m_listener = p_listener;
		}

		/**
		 */
		[Inline]
		final public function AddBody(p_body:b2Body):void
		{
			p_body.m_islandIndex = m_bodyCount;
			m_bodies[m_bodyCount++] = p_body;
		}

		/**
		 */
		[Inline]
		final public function AddContact(p_contact:b2Contact):void
		{
			m_contacts[m_contactCount++] = p_contact;
		}

		/**
		 */
		[Inline]
		final public function AddJoint(joint:b2Joint):void
		{
			m_joints[m_jointCount++] = joint;
		}

		/**
		 */
		[Inline]
		final public function Report(p_constraints:Vector.<b2ContactVelocityConstraint>):void
		{
			if (m_listener)
			{
				var count:int = m_contactCount;
				var pointCount:int;
				var c:b2Contact;
				var vc:b2ContactVelocityConstraint;

				var normalImpulses:Vector.<Number> = _helperImpulse.normalImpulses;
				var tangentImpulses:Vector.<Number> = _helperImpulse.tangentImpulses;
				var points:Vector.<b2VelocityConstraintPoint>;
				var cp:b2VelocityConstraintPoint;

				for (var i:int = 0; i < count; ++i)
				{
					c = m_contacts[i];

					vc = p_constraints[i];

					points = vc.points;
					pointCount = vc.pointCount;
					_helperImpulse.count = pointCount;

					for (var j:int = 0; j < pointCount; ++j)
					{
						cp = points[j];
						normalImpulses[j] = cp.normalImpulse;
						tangentImpulses[j] = cp.tangentImpulse;
					}

					m_listener.PostSolve(c, _helperImpulse);
				}
			}
		}

		/**
		 */
		public function Solve(p_step:b2TimeStep, p_gravityX:Number, p_gravityY:Number, p_allowSleep:Boolean):void
		{
			var h:Number = p_step.dt;
			var bodyCount:int = m_bodyCount;
			var b:b2Body;
			var cX:Number;
			var cY:Number;
			var vX:Number;
			var vY:Number;
			var a:Number;
			var w:Number;
			var sweep:b2Sweep;
			var vec3:b2Vec3;
			var i:int;
			var j:int;

			// Integrate velocities and apply damping. Initialize the body state.
			for (i = 0; i < bodyCount; ++i)
			{
				b = m_bodies[i];

				sweep = b.m_sweep;
				cX = sweep.worldCenterX;
				cY = sweep.worldCenterY;
				a = sweep.worldAngle;

				vX = b.m_linearVelocityX;
				vY = b.m_linearVelocityY;
				w = b.m_angularVelocity;

				// Store positions for continuous collision.
				sweep.worldCenterX0 = cX;
				sweep.worldCenterY0 = cY;
				sweep.worldAngle0 = a;
		
				if (b.m_type == b2Body.DYNAMIC)
				{
					// Integrate velocities.
					vX += h * (b.m_gravityScale * p_gravityX + b.m_invMass * b.m_forceX);
					vY += h * (b.m_gravityScale * p_gravityY + b.m_invMass * b.m_forceY);
					w += h * b.m_invI * b.m_torque;

					// Apply damping.
					// ODE: dv/dt + c * v = 0
					// Solution: v(t) = v0 * exp(-c * t)
					// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
					// v2 = exp(-c * dt) * v1
					// Pade approximation:
					// v2 = v1 * 1 / (1 + c * dt)
					var t:Number = 1.0 / (1.0 + h * b.m_linearDamping);
					vX *= t;
					vY *= t;
					w *= 1.0 / (1.0 + h * b.m_angularDamping);
				}

				vec3 = m_positions[i];
				vec3.x = cX;
				vec3.y = cY;
				vec3.z = a;

				vec3 = m_velocities[i];
				vec3.x = vX;
				vec3.y = vY;
				vec3.z = w;
			}

			// Solver data
			_solverDataHelper.step = p_step;
			_solverDataHelper.positions = m_positions;
			_solverDataHelper.velocities = m_velocities;

			// Initialize velocity constraints.
			_contactSolverDefHelper.step = p_step;
			_contactSolverDefHelper.contacts = m_contacts;
			_contactSolverDefHelper.count = m_contactCount;
			_contactSolverDefHelper.positions = m_positions;
			_contactSolverDefHelper.velocities = m_velocities;

			_contactSolver.Initialize(_contactSolverDefHelper);
			_contactSolver.InitializeVelocityConstraints();

			if (p_step.warmStarting)
			{
				_contactSolver.WarmStart();
			}

			var jointCount:int = m_jointCount;

			for (i = 0; i < jointCount; ++i)
			{
				m_joints[i].InitVelocityConstraints(_solverDataHelper);
			}

			// Solve velocity constraints
			var velIterations:int = p_step.velocityIterations;

			for (i = 0; i < velIterations; ++i)
			{
				for (j = 0; j < jointCount; ++j)
				{
					m_joints[j].SolveVelocityConstraints(_solverDataHelper);
				}

				_contactSolver.SolveVelocityConstraints();
			}

			// Store impulses for warm starting
			_contactSolver.StoreImpulses();

			// Integrate positions
			var pos:b2Vec3;
			var vel:b2Vec3;

			for (i = 0; i < bodyCount; ++i)
			{
				pos = m_positions[i];
				cX = pos.x;
				cY = pos.y;
				a = pos.z;

				vel = m_velocities[i];
				vX = vel.x;
				vY = vel.y;
				w = vel.z;

				// Check for large velocities
				var translationX:Number = h * vX;
				var translationY:Number = h * vY;
				var ratio:Number;

				if (b2Math.DotSingle(translationX, translationY) > b2Settings.maxTranslationSquared)
				{
					ratio = b2Settings.maxTranslation / b2Math.Length(translationX, translationY);
					vX *= ratio;
					vY *= ratio;
				}

				var rotation:Number = h * w;
				if (rotation * rotation > b2Settings.maxRotationSquared)
				{
					ratio = b2Settings.maxRotation / b2Math.Abs(rotation);
					w *= ratio;
				}

				// Integrate
				cX += h * vX;
				cY += h * vY;
				a += h * w;

				pos.x = cX;
				pos.y = cY;
				pos.z = a;

				vel.x = vX;
				vel.y = vY;
				vel.z = w;
			}

			// Solve position constraints
			var positionSolved:Boolean = false;
			var posIterations:int = p_step.positionIterations;
			var contactsOkay:Boolean;
			var jointsOkay:Boolean;
			var jointOkay:Boolean;

			for (i = 0; i < posIterations; ++i)
			{
				contactsOkay = _contactSolver.SolvePositionConstraints();

				jointsOkay = true;

				for (j = 0; j < jointCount; ++j)
				{
					jointOkay = m_joints[j].SolvePositionConstraints(_solverDataHelper);
					jointsOkay = jointsOkay && jointOkay;
				}

				if (contactsOkay && jointsOkay)
				{
					// Exit early if the position errors are small.
					positionSolved = true;
					break;
				}
			}

			// Copy state buffers back to the bodies
			var body:b2Body;
			for (i = 0; i < bodyCount; ++i)
			{
				pos = m_positions[i];
				body = m_bodies[i];
				sweep = body.m_sweep;
				sweep.worldCenterX = pos.x;
				sweep.worldCenterY = pos.y;
				sweep.worldAngle = pos.z;

				vel = m_velocities[i];
				body.m_linearVelocityX = vel.x;
				body.m_linearVelocityY = vel.y;
				body.m_angularVelocity = vel.z;

				body.SynchronizeTransform();
			}

			//
			Report(_contactSolver.m_velocityConstraints);

			//
			if (p_allowSleep)
			{
				var minSleepTime:Number = Number.MAX_VALUE;
		
				var linTolSqr:Number = b2Settings.linearSleepTolerance * b2Settings.linearSleepTolerance;
				var angTolSqr:Number = b2Settings.angularSleepTolerance * b2Settings.angularSleepTolerance;
		
				for (i = 0; i < bodyCount; ++i)
				{
					b = m_bodies[i];

					if (b.GetType() == b2Body.STATIC)
					{
						continue;
					}
		
					if ((b.m_flags & b2Body.e_autoSleepFlag) == 0 ||
						b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
						b2Math.DotSingle(b.m_linearVelocityX, b.m_linearVelocityY) > linTolSqr)
					{
						b.m_sleepTime = 0.0;
						minSleepTime = 0.0;
					}
					else
					{
						b.m_sleepTime += h;
						minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
					}
				}
		
				if (minSleepTime >= b2Settings.timeToSleep && positionSolved)
				{
					for (i = 0; i < bodyCount; ++i)
					{
						b = m_bodies[i];
						b.SetAwake(false);
					}
				}
			}
		}

		/**
		 * TODO:
		 */
		public function SolveTOI(p_subStep:b2TimeStep, p_toiIndexA:int, p_toiIndexB:int):void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		[Inline]
		final public function Clear():void
		{
			m_bodyCount = 0;
			m_contactCount = 0;
			m_jointCount = 0;
		}
	}
}
