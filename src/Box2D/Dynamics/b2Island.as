/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 0:03
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Structures.b2ContactImpulse;
	import Box2D.Collision.Structures.b2ContactVelocityConstraint;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Collision.Structures.b2VelocityConstraintPoint;
	import Box2D.Common.Math.b2Vec3;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Callbacks.b2ContactListener;
	import Box2D.Dynamics.Joints.b2Joint;
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
		final public function Report(p_constraints:b2ContactVelocityConstraint):void
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
		 * TODO:
		 */
		public function Solve(p_step:b2TimeStep, p_gravityX:Number, p_gravityY:Number, p_allowSleep:Boolean):void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
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
