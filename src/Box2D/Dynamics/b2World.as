/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Joints.b2Joint;

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
		b2internal var m_jointList:b2Joint;

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

		/**
		* Get the world body list. With the returned body, use b2Body::GetNext to get
		* the next body in the world list. A NULL body indicates the end of the list.
		* @return the head of the world body list.
		*/
		final public function GetBodyList():b2Body
		{
			return m_bodyList;
		}

		/**
		* Get the world joint list. With the returned joint, use b2Joint::GetNext to get
		* the next joint in the world list. A NULL joint indicates the end of the list.
		* @return the head of the world joint list.
		*/
		final public function GetJointList():b2Joint
		{
			return m_jointList;
		}

		/**
		* Get the world contact list. With the returned contact, use b2Contact::GetNext to get
		* the next contact in the world list. A NULL contact indicates the end of the list.
		* @return the head of the world contact list.
		* @warning contacts are created and destroyed in the middle of a time step.
		* Use b2ContactListener to avoid missing contacts.
		*/
		final public function GetContactList():b2Contact
		{
			return m_contactManager.m_contactList
		}
	}
}
