/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.b2BroadPhase;

	/**
	 * Delegate of b2World.
	 * TODO:
	 */
	public class b2ContactManager
	{
		public var m_broadPhase:b2BroadPhase;

		public function b2ContactManager()
		{
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
