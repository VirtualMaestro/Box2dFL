/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.b2BroadPhase;
	import Box2D.Dynamics.Filters.b2Filter;
	import Box2D.b2Assert;

	/**
	 * Delegate of b2World.
	 * TODO:
	 */
	public class b2ContactManager
	{
		public var m_broadPhase:b2BroadPhase;
		public var m_contactList:b2Contact;
		public var m_contactCount:int;
		public var m_contactFilter:b2Filter;
		public var m_contactListener/*:b2ContactListener TODO:*/;

		public function b2ContactManager()
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * Broad-phase callback.
		 * TODO
 		 */
		public function AddPair(p_proxyUserDataA:*, p_proxyUserDataB:*):void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO
		 */
		public function FindNewContacts():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 * TODO
		 */
		public function Collide():void
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
		}

		/**
		 *
		 * @param p_contact
		 */
		public function Destroy(p_contact:b2Contact):void
		{
			// TODO:
		}
	}
}
