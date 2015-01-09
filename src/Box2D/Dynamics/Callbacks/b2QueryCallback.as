/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 15:55
 */
package Box2D.Dynamics.Callbacks
{
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	/**
	 * Abstract callback class for AABB queries.
	 * @See b2World::Query
	 */
	public class b2QueryCallback
	{
		/**
		 * Called for each fixture found in the query AABB.
		 * @param p_fixture
		 * @return false to terminate the query.
		 */
		virtual public function ReportFixture(p_fixture:b2Fixture):Boolean
		{
			b2Assert(false, "current method isn't implemented yet and can't be used!");
			return false;
		}
	}
}
