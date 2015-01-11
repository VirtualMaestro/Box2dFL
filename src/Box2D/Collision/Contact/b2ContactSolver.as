/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 2:09
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Manifold.b2Manifold;
	import Box2D.Collision.Manifold.b2ManifoldPoint;
	import Box2D.Collision.Manifold.b2WorldManifold;
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Collision.Structures.b2ContactPositionConstraint;
	import Box2D.Collision.Structures.b2ContactVelocityConstraint;
	import Box2D.Collision.Structures.b2PositionSolverManifold;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Collision.Structures.b2VelocityConstraintPoint;
	import Box2D.Common.Math.b2Mat22;
	import Box2D.Common.Math.b2Math;
	import Box2D.Common.Math.b2SPoint;
	import Box2D.Common.Math.b2Sweep;
	import Box2D.Common.Math.b2Vec3;
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Def.b2ContactSolverDef;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 */
	public class b2ContactSolver
	{
		public var m_step:b2TimeStep;
		public var m_positions:Vector.<b2Vec3>;
		public var m_velocities:Vector.<b2Vec3>;

		public var m_positionConstraints:Vector.<b2ContactPositionConstraint>;
		public var m_velocityConstraints:Vector.<b2ContactVelocityConstraint>;
		public var m_contacts:Vector.<b2Contact>;
		public var m_count:int;

		private var _psm:b2PositionSolverManifold;

		b2internal var g_blockSolve:Boolean = true;

		/**
		 */
		public function b2ContactSolver(def:b2ContactSolverDef)
		{
			_psm = new b2PositionSolverManifold();

			//
			m_step = def.step;
			m_count = def.count;
			var count:int = m_count;

			m_positionConstraints = new <b2ContactPositionConstraint>[];
			m_velocityConstraints = new <b2ContactVelocityConstraint>[];

			for (var i:int = 0; i < count; i++)
			{
				m_positionConstraints[i] = new b2ContactPositionConstraint();
				m_velocityConstraints[i] = new b2ContactVelocityConstraint();
			}

			m_positions = def.positions;
			m_velocities = def.velocities;
			m_contacts = def.contacts;

			// Initialize position independent portions of the constraints.
			for (i = 0; i < count; ++i)
			{
				var contact:b2Contact = m_contacts[i];

				var fixtureA:b2Fixture = contact.m_fixtureA;
				var fixtureB:b2Fixture = contact.m_fixtureB;
				var shapeA:b2Shape = fixtureA.GetShape();
				var shapeB:b2Shape = fixtureB.GetShape();
				var radiusA:Number = shapeA.m_radius;
				var radiusB:Number = shapeB.m_radius;
				var bodyA:b2Body = fixtureA.GetBody();
				var bodyB:b2Body = fixtureB.GetBody();
				var manifold:b2Manifold = contact.GetManifold();
				var pointCount:int = manifold.pointCount;

				CONFIG::debug
				{
					b2Assert(pointCount > 0, "count of points has to be greater then 0");
				}

				var vc:b2ContactVelocityConstraint = m_velocityConstraints[i];
				vc.friction = contact.m_friction;
				vc.restitution = contact.m_restitution;
				vc.tangentSpeed = contact.m_tangentSpeed;
				vc.indexA = bodyA.m_islandIndex;
				vc.indexB = bodyB.m_islandIndex;
				vc.invMassA = bodyA.m_invMass;
				vc.invMassB = bodyB.m_invMass;
				vc.invIA = bodyA.m_invI;
				vc.invIB = bodyB.m_invI;
				vc.contactIndex = i;
				vc.pointCount = pointCount;
				vc.K.SetZero();
				vc.normalMass.SetZero();

				var pc:b2ContactPositionConstraint = m_positionConstraints[i];
				pc.indexA = bodyA.m_islandIndex;
				pc.indexB = bodyB.m_islandIndex;
				pc.invMassA = bodyA.m_invMass;
				pc.invMassB = bodyB.m_invMass;
				var sweep:b2Sweep = bodyA.m_sweep;
				pc.localCenterAX = sweep.localCenterX;
				pc.localCenterAY = sweep.localCenterY;
				sweep = bodyB.m_sweep;
				pc.localCenterBX = sweep.localCenterX;
				pc.localCenterBY = sweep.localCenterY;
				pc.invIA = bodyA.m_invI;
				pc.invIB = bodyB.m_invI;
				pc.localNormalX = manifold.localNormalX;
				pc.localNormalY = manifold.localNormalY;
				pc.localPointX = manifold.localPointX;
				pc.localPointY = manifold.localPointY;
				pc.pointCount = pointCount;
				pc.radiusA = radiusA;
				pc.radiusB = radiusB;
				pc.type = manifold.type;

				var warmStarting:Boolean = m_step.warmStarting;
				var dtRatio:Number = m_step.dtRatio;
				var localPoints:Vector.<Number> = pc.localPoints;

				for (var j:int = 0; j < pointCount; ++j)
				{
					var cp:b2ManifoldPoint = manifold.points[j];
					var vcp:b2VelocityConstraintPoint = vc.points[j];

					if (warmStarting)
					{
						vcp.normalImpulse = dtRatio * cp.normalImpulse;
						vcp.tangentImpulse = dtRatio * cp.tangentImpulse;
					}
					else
					{
						vcp.normalImpulse = 0.0;
						vcp.tangentImpulse = 0.0;
					}

					vcp.rAX = 0;
					vcp.rAY = 0;
					vcp.rBX = 0;
					vcp.rBY = 0;
					vcp.normalMass = 0.0;
					vcp.tangentMass = 0.0;
					vcp.velocityBias = 0.0;

					b2Math.setXY(cp.localPointX, cp.localPointY, localPoints, j);
				}
			}
		}

		/**
		 * Initialize position dependent portions of the velocity constraints.
		 */
		public function InitializeVelocityConstraints():void
		{
			var xfA:b2Mat22 = b2Mat22.Get();
			var xfB:b2Mat22 = b2Mat22.Get();
			var worldManifold:b2WorldManifold = new b2WorldManifold();
			var helperPoint:b2SPoint = b2SPoint.Get();
			var count:int = m_count;

			for (var i:int = 0; i < count; ++i)
			{
				var vc:b2ContactVelocityConstraint = m_velocityConstraints[i];
				var pc:b2ContactPositionConstraint = m_positionConstraints[i];

				var radiusA:Number = pc.radiusA;
				var radiusB:Number = pc.radiusB;
				var manifold:b2Manifold = m_contacts[vc.contactIndex].GetManifold();

				var indexA:int = vc.indexA;
				var indexB:int = vc.indexB;

				var mA:Number = vc.invMassA;
				var mB:Number = vc.invMassB;
				var iA:Number = vc.invIA;
				var iB:Number = vc.invIB;
				var localCenterAX:Number = pc.localCenterAX;
				var localCenterAY:Number = pc.localCenterAY;
				var localCenterBX:Number = pc.localCenterBX;
				var localCenterBY:Number = pc.localCenterBY;

				//
				var pos:b2Vec3 = m_positions[indexA];
				var cAX:Number = pos.x;
				var cAY:Number = pos.y;
				var aA:Number = pos.z;

				//
				var vel:b2Vec3 = m_velocities[indexA];
				var vAX:Number = vel.x;
				var vAY:Number = vel.y;
				var wA:Number = vel.z;

				//
				pos = m_positions[indexB];
				var cBX:Number = pos.x;
				var cBY:Number = pos.y;
				var aB:Number = pos.z;

				//
				vel = m_velocities[indexB];
				var vBX:Number = vel.x;
				var vBY:Number = vel.y;
				var wB:Number = vel.z;

				CONFIG::debug
				{
					b2Assert(manifold.pointCount > 0, "manifold points count has to be greater then 0");
				}

				xfA.SetIdentity();
				xfB.SetIdentity();

				xfA.SetAngle(aA);
				xfB.SetAngle(aB);

				b2Math.MulRV(xfA, localCenterAX, localCenterAY, xfA);
				xfA.x = cAX - xfA.x;
				xfA.y = cAY - xfA.y;

				b2Math.MulRV(xfB, localCenterBX, localCenterBY, xfB);
				xfB.x = cBX - xfB.x;
				xfB.y = cBY - xfB.y;

				worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

				var vcNormalX:Number = worldManifold.normalX;
				var vcNormalY:Number = worldManifold.normalY;
				vc.normalX = vcNormalX;
				vc.normalY = vcNormalY;
				var manifoldPoints:Vector.<Number> = worldManifold.points;
				var pointCount:int = vc.pointCount;

				for (var j:int = 0; j < pointCount; ++j)
				{
					var vcp:b2VelocityConstraintPoint = vc.points[j];

					var manPointX:Number = b2Math.getX(manifoldPoints, j);
					var manPointY:Number = b2Math.getY(manifoldPoints, j);

					var rAX:Number = manPointX - cAX;
					var rAY:Number = manPointY - cAY;

					var rBX:Number = manPointX - cBX;
					var rBY:Number = manPointY - cBY;

					vcp.rAX = rAX;
					vcp.rAY = rAY;

					vcp.rBX = rBX;
					vcp.rBY = rBY;

					var rnA:Number = b2Math.CrossVectors(rAX, rAY, vcNormalX, vcNormalY);
					var rnB:Number = b2Math.CrossVectors(rBX, rBY, vcNormalX, vcNormalY);

					var kNormal:Number = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

					b2Math.CrossVectorScalar(vcNormalX, vcNormalY, 1.0, helperPoint);
					var tangentX:Number = helperPoint.x;
					var tangentY:Number = helperPoint.y;

					var rtA:Number = b2Math.CrossVectors(rAX, rAY, tangentX, tangentY);
					var rtB:Number = b2Math.CrossVectors(rBX, rBY, tangentX, tangentY);

					var kTangent:Number = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

					vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

					// Setup a velocity bias for restitution.
					vcp.velocityBias = 0.0;

					b2Math.CrossScalarVector(wB, rBX, rBY, helperPoint);
					var cross1X:Number = helperPoint.x;
					var cross1Y:Number = helperPoint.y;

					b2Math.CrossScalarVector(wA, rAX, rAY, helperPoint);
					var cross2X:Number = helperPoint.x;
					var cross2Y:Number = helperPoint.y;

					var vRel:Number = b2Math.Dot(vcNormalX, vcNormalY, (vBX + cross1X - vAX - cross2X), (vBY + cross1Y - vAY - cross2Y));

					if (vRel < -b2Settings.velocityThreshold)
					{
						vcp.velocityBias = -vc.restitution * vRel;
					}
				}

				// If we have two points, then prepare the block solver.
				if (vc.pointCount == 2 && g_blockSolve)
				{
					var vcp1:b2VelocityConstraintPoint = vc.points[0];
					var vcp2:b2VelocityConstraintPoint = vc.points[1];

					var rn1A:Number = b2Math.CrossVectors(vcp1.rAX, vcp1.rAY, vcNormalX, vcNormalY);
					var rn1B:Number = b2Math.CrossVectors(vcp1.rBX, vcp1.rBY, vcNormalX, vcNormalY);
					var rn2A:Number = b2Math.CrossVectors(vcp2.rAX, vcp2.rAY, vcNormalX, vcNormalY);
					var rn2B:Number = b2Math.CrossVectors(vcp2.rBX, vcp2.rBY, vcNormalX, vcNormalY);

					var k11:Number = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
					var k22:Number = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
					var k12:Number = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

					// Ensure a reasonable condition number.
					var k_maxConditionNumber:Number = 1000.0;
					if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
					{
						// K is safe to invert.
						var kMat:b2Mat22 = vc.K;
						kMat.Set22(k11, k12, k12, k22);
						kMat.GetInverse(vc.normalMass);
					}
					else
					{
						// The constraints are redundant, just use one.
						// TODO_ERIN use deepest?
						vc.pointCount = 1;
					}
				}
			}

			xfA.Dispose();
			xfB.Dispose();
			helperPoint.Dispose();
			worldManifold = null;
		}

		/**
		 */
		public function WarmStart():void
		{
			// Warm start.
			var count:int = m_count;
			var vc:b2ContactVelocityConstraint;
			var vecA:b2Vec3;
			var vecB:b2Vec3;
			var helperPoint:b2SPoint = b2SPoint.Get();

			for (var i:int = 0; i < count; ++i)
			{
				vc = m_velocityConstraints[i];

				var indexA:int = vc.indexA;
				var indexB:int = vc.indexB;
				var mA:Number = vc.invMassA;
				var iA:Number = vc.invIA;
				var mB:Number = vc.invMassB;
				var iB:Number = vc.invIB;
				var pointCount:int = vc.pointCount;

				//
				vecA = m_velocities[indexA];

				var vAX:Number = vecA.x;
				var vAY:Number = vecA.y;
				var wA:Number = vecA.z;

				//
				vecB = m_velocities[indexB];

				var vBX:Number = vecB.x;
				var vBY:Number = vecB.y;
				var wB:Number = vecB.z;

				var normalX:Number = vc.normalX;
				var normalY:Number = vc.normalY;

				b2Math.CrossVectorScalar(normalX, normalY, 1.0, helperPoint);

				var tangentX:Number = helperPoint.x;
				var tangentY:Number = helperPoint.y;

				for (var j:int = 0; j < pointCount; ++j)
				{
					var vcp:b2VelocityConstraintPoint = vc.points[j];
					var PX:Number = vcp.normalImpulse * normalX + vcp.tangentImpulse * tangentX;
					var PY:Number = vcp.normalImpulse * normalY + vcp.tangentImpulse * tangentY;

					wA -= iA * b2Math.CrossVectors(vcp.rAX, vcp.rAY, PX, PX);
					vAX -= mA * PX;
					vAY -= mA * PY;

					wB += iB * b2Math.CrossVectors(vcp.rBX, vcp.rBY, PX, PY);
					vBX += mB * PX;
					vBY += mB * PY;
				}

				vecA.x = vAX;
				vecA.y = vAY;
				vecA.z = wA;

				vecB.x = vBX;
				vecB.y = vBY;
				vecB.z = wB;
			}

			helperPoint.Dispose();
		}

		/**
		 * TODO:
		 */
		public function SolveVelocityConstraints():void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 */
		public function StoreImpulses():void
		{
			var count:int = m_count;

			for (var i:int = 0; i < count; ++i)
			{
				var vc:b2ContactVelocityConstraint = m_velocityConstraints[i];
				var manifold:b2Manifold = m_contacts[vc.contactIndex].GetManifold();
				var pointCount:int = vc.pointCount;
				var mpList:Vector.<b2ManifoldPoint> = manifold.points;
				var vcList:Vector.<b2VelocityConstraintPoint> = vc.points;
				var mp:b2ManifoldPoint;
				var vcp:b2VelocityConstraintPoint;

				for (var j:int = 0; j < pointCount; ++j)
				{
					mp = mpList[j];
					vcp = vcList[j];

					mp.normalImpulse = vcp.normalImpulse;
					mp.tangentImpulse = vcp.tangentImpulse;
				}
			}
		}

		/**
		 */
		public function SolvePositionConstraints():Boolean
		{
			var minSeparation:Number = 0.0;
			var count:int = m_count;
			var pc:b2ContactPositionConstraint;
			var indexA:int;
			var indexB:int;
			var localCenterAX:Number;
			var localCenterAY:Number;
			var mA:Number;
			var iA:Number;
			var localCenterBX:Number;
			var localCenterBY:Number;
			var mB:Number;
			var iB:Number;
			var pointCount:int;
			var cAX:Number;
			var cAY:Number;
			var aA:Number;
			var cBX:Number;
			var cBY:Number;
			var aB:Number;
			var vecA:b2Vec3;
			var vecB:b2Vec3;
			var xfA:b2Mat22 = b2Mat22.Get();
			var xfB:b2Mat22 = b2Mat22.Get();
			var normalX:Number;
			var normalY:Number;
			var pointX:Number;
			var pointY:Number;
			var separation:Number;
			var rAX:Number;
			var rAY:Number;
			var rBX:Number;
			var rBY:Number;
			var C:Number;
			var rnA:Number;
			var rnB:Number;
			var K:Number;
			var impulse:Number;
			var PX:Number;
			var PY:Number;

			for (var i:int = 0; i < count; i++)
			{
				pc = m_positionConstraints[i];

				indexA = pc.indexA;
				indexB = pc.indexB;

				localCenterAX = pc.localCenterAX;
				localCenterAY = pc.localCenterAY;

				mA = pc.invMassA;
				iA = pc.invIA;

				localCenterBX = pc.localCenterBX;
				localCenterBY = pc.localCenterBY;
				mB = pc.invMassB;
				iB = pc.invIB;

				pointCount = pc.pointCount;

				vecA = m_positions[indexA];
				cAX = vecA.x;
				cAY = vecA.y;
				aA = vecA.z;

				vecB = m_positions[indexB];
				cBX = vecB.x;
				cBY = vecB.y;
				aB = vecB.z;

				// Solve normal constraints
				for (var j:int = 0; j < pointCount; j++)
				{
					xfA.SetAngle(aA);
					xfB.SetAngle(aB);

					b2Math.MulRV(xfA, localCenterAX, localCenterAY, xfA);
					xfA.x = cAX - xfA.x;
					xfA.y = cAY - xfA.y;

					b2Math.MulRV(xfB, localCenterBX, localCenterBY, xfB);
					xfB.x = cBX - xfB.x;
					xfB.y = cBY - xfB.y;

					_psm.Initialize(pc, xfA, xfB, j);

					normalX = _psm.normalX;
					normalY = _psm.normalY;
					pointX = _psm.pointX;
					pointY = _psm.pointY;
					separation = _psm.separation;

					rAX = pointX - cAX;
					rAY = pointY - cAY;
					rBX = pointX - cBX;
					rBY = pointY - cBY;

					// Track max constraint error.
					minSeparation = b2Math.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					C = b2Math.Clamp(b2Settings.baumgarte * (separation + b2Settings.linearSlop), -b2Settings.maxLinearCorrection, 0.0);

					// Compute the effective mass.
					rnA = b2Math.CrossVectors(rAX, rAY, normalX, normalY);
					rnB = b2Math.CrossVectors(rBX, rBY, normalX, normalY);
					K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					// Compute normal impulse
					impulse = K > 0.0 ? -C / K : 0.0;

					PX = impulse * normalX;
					PY = impulse * normalY;

					cAX -= mA * PX;
					cAY -= mA * PY;
					aA -= iA * b2Math.CrossVectors(rAX, rAY, PX, PY);

					cBX += mB * PX;
					cBY += mB * PY;
					aB += iB * b2Math.CrossVectors(rBX, rBY, PX, PY);
				}

				vecA.x = cAX;
				vecA.y = cAY;
				vecA.z = aA;

				vecB.x = cBX;
				vecB.y = cBY;
				vecB.z = aB;
			}

			xfA.Dispose();
			xfB.Dispose();

			// We can't expect minSeparation >= -b2_linearSlop because we don't
			// push the separation above -b2_linearSlop.
			return minSeparation >= -3.0 * b2Settings.linearSlop;
		}

		/**
		 */
		public function SolveTOIPositionConstraints(toiIndexA:int, toiIndexB:int):Boolean
		{
			var minSeparation:Number = 0.0;
			var count:int = m_count;
			var pc:b2ContactPositionConstraint;
			var indexA:int;
			var indexB:int;
			var localCenterAX:Number;
			var localCenterAY:Number;
			var localCenterBX:Number;
			var localCenterBY:Number;
			var pointCount:int;
			var mA:Number;
			var iA:Number;
			var mB:Number;
			var iB:Number;
			var cAX:Number;
			var cAY:Number;
			var aA:Number;
			var cBX:Number;
			var cBY:Number;
			var aB:Number;
			var vecA:b2Vec3;
			var vecB:b2Vec3;
			var xfA:b2Mat22 = b2Mat22.Get();
			var xfB:b2Mat22 = b2Mat22.Get();
			var normalX:Number;
			var normalY:Number;
			var pointX:Number;
			var pointY:Number;
			var separation:Number;
			var rAX:Number;
			var rAY:Number;
			var rBX:Number;
			var rBY:Number;
			var C:Number;
			var rnA:Number;
			var rnB:Number;
			var K:Number;
			var impulse:Number;
			var PX:Number;
			var PY:Number;

			//
			for (var i:int = 0; i < count; i++)
			{
				pc = m_positionConstraints[i];

				indexA = pc.indexA;
				indexB = pc.indexB;

				localCenterAX = pc.localCenterAX;
				localCenterAY = pc.localCenterAY;
				localCenterBX = pc.localCenterBX;
				localCenterBY = pc.localCenterBY;

				pointCount = pc.pointCount;

				mA = 0.0;
				mB = 0.0;

				if (indexA == toiIndexA || indexA == toiIndexB)
				{
					mA = pc.invMassA;
					iA = pc.invIA;
				}

				mB = 0.0;
				iB = 0.0;

				if (indexB == toiIndexA || indexB == toiIndexB)
				{
					mB = pc.invMassB;
					iB = pc.invIB;
				}

				vecA = m_positions[indexA];
				cAX = vecA.x;
				cAY = vecA.y;
				aA = vecA.z;

				vecB = m_positions[indexB];
				cBX = vecB.x;
				cBY = vecB.y;
				aB = vecB.z;

				// Solve normal constraints
				for (var j:int = 0; j < pointCount; j++)
				{
					xfA.SetAngle(aA);
					xfB.SetAngle(aB);

					b2Math.MulRV(xfA, localCenterAX, localCenterAY, xfA);
					xfA.x = cAX - xfA.x;
					xfA.y = cAY - xfA.y;

					b2Math.MulRV(xfB, localCenterBX, localCenterBY, xfB);
					xfB.x = cBX - xfB.x;
					xfB.y = cBY - xfB.y;

					_psm.Initialize(pc, xfA, xfB, j);

					normalX = _psm.normalX;
					normalY = _psm.normalY;
					pointX = _psm.pointX;
					pointY = _psm.pointY;
					separation = _psm.separation;

					rAX = pointX - cAX;
					rAY = pointY - cAY;
					rBX = pointX - cBX;
					rBY = pointY - cBY;

					// Track max constraint error.
					minSeparation = b2Math.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					C = b2Math.Clamp(b2Settings.toiBaugarte * (separation + b2Settings.linearSlop), -b2Settings.maxLinearCorrection, 0.0);

					// Compute the effective mass.
					rnA = b2Math.CrossVectors(rAX, rAY, normalX, normalY);
					rnB = b2Math.CrossVectors(rBX, rBY, normalX, normalY);
					K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					// Compute normal impulse
					impulse = K > 0.0 ? -C / K : 0.0;

					PX = impulse * normalX;
					PY = impulse * normalY;

					cAX -= mA * PX;
					cAY -= mA * PY;
					aA -= iA * b2Math.CrossVectors(rAX, rAY, PX, PY);

					cBX += mB * PX;
					cBY += mB * PY;
					aB += iB * b2Math.CrossVectors(rBX, rBY, PX, PY);
				}

				vecA.x = cAX;
				vecA.y = cAY;
				vecA.z = aA;

				vecB.x = cBX;
				vecB.y = cBY;
				vecB.z = aB;
			}

			xfA.Dispose();
			xfB.Dispose();

			// We can't expect minSeparation >= -b2_linearSlop because we don't
			// push the separation above -b2_linearSlop.
			return minSeparation >= -1.5 * b2Settings.linearSlop;
		}
	}
}
