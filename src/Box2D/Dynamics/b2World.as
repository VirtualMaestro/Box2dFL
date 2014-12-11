/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * The world class manages all physics entities, dynamic simulation,
	 * and asynchronous queries. The world also contains efficient memory
	 * management facilities.
	 *
	 * TODO:
	 */
	public class b2World
	{
		//
		static b2internal var e_newFixture:uint = 0x0001;
		static b2internal var e_locked:uint = 0x0002;
		static b2internal var e_clearForces:uint = 0x0004;

		b2internal var m_contactManager:b2ContactManager;
		b2internal var m_flags:int;

		b2internal var m_bodyList:b2Body;
		b2internal var m_jointList/*:b2Joint*/;   // TODO:

		b2internal var m_bodyCount:int;
		b2internal var m_jointCount:int;

		b2internal var m_gravityX:Number;
		b2internal var m_gravityY:Number;
		b2internal var m_allowSleep:Boolean;

		// This is used to compute the time step ratio to
		// support a variable time step.
		b2internal var m_inv_dt0:Number;

		// TODO: add some debugging props from original

		/**
		 */
		public function b2World()
		{
		}

		/**
		 * Is the world locked (in the middle of a time step).
		 * @return
		 */
		[Inline]
		final public function IsLocked():Boolean
		{
			return (m_flags & e_locked) == e_locked;
		}
	}
}
