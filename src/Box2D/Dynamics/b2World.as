/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Common.b2internal;

	/**
	 * The world class manages all physics entities, dynamic simulation,
	 * and asynchronous queries. The world also contains efficient memory
	 * management facilities.
	 *
	 * TODO:
	 */
	public class b2World
	{
		b2internal var m_contactManager:b2ContactManager;


		public function b2World()
		{
		}
	}
}
