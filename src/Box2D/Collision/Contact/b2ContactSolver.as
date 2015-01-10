/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 2:09
 */
package Box2D.Collision.Contact
{
	import Box2D.Collision.Structures.b2ContactPositionConstraint;
	import Box2D.Collision.Structures.b2ContactVelocityConstraint;
	import Box2D.Collision.Structures.b2TimeStep;
	import Box2D.Common.Math.b2Vec3;
	import Box2D.Dynamics.Def.b2ContactSolverDef;
	import Box2D.b2Assert;

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

		/**
		 * TODO:
		 */
		public function b2ContactSolver(def:b2ContactSolverDef)
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function InitializeVelocityConstraints():void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * TODO
		 */
		public function WarmStart():void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function SolveVelocityConstraints():void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function StoreImpulses():void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * TODO:
		 */
		public function SolvePositionConstraints():Boolean
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
			return false;
		}

		/**
		 * TODO:
		 */
		public function SolveTOIPositionConstraints(toiIndexA:int, toiIndexB:int):Boolean
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
			return false;
		}
	}
}
