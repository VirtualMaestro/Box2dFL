/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 16:38
 */
package Box2D.Dynamics.Callbacks
{
	import Box2D.Dynamics.Joints.b2Joint;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	/**
	* Joints and fixtures are destroyed when their associated
	* body is destroyed. Implement this listener so that you
	* may nullify references to these joints and shapes.
	*/
	public class b2DestructionListener
	{
		/**
		 * Called when any joint is about to be destroyed due to the destruction of one of its attached bodies.
		 */
		virtual public function SayGoodbyeJoint(p_joint:b2Joint):void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}

		/**
		 * Called when any fixture is about to be destroyed due to the destruction of its parent body.
		 */
		virtual public function SayGoodbyeFixture(p_fixture:b2Fixture):void
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
		}
	}
}
