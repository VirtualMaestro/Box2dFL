/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 16:10
 */
package Box2D.Dynamics.Callbacks
{
	import Box2D.Dynamics.b2Fixture;
	import Box2D.b2Assert;

	/**
	 * Abstract callback class for ray casts.
	 * See b2World::RayCast
	 */
	public class b2RayCastCallback
	{
		/**
		 * Called for each fixture found in the query. You control how the ray cast
		 * proceeds by returning a float:
		 * return -1: ignore this fixture and continue
		 * return 0: terminate the ray cast
		 * return fraction: clip the ray to this point
		 * return 1: don't clip the ray and continue
		 * @param p_fixture the fixture hit by the ray
		 * @param p_pointX the point of initial intersection
		 * @param p_pointY the point of initial intersection
		 * @param p_normalX the normal vector at the point of intersection
		 * @param p_normalY the normal vector at the point of intersection
		 * @param p_fraction
		 * @return -1 to filter, 0 to terminate, fraction to clip the ray for
		 * closest hit, 1 to continue
		 */
		virtual public function ReportFixture(p_fixture:b2Fixture, p_pointX:Number, p_pointY:Number,
												p_normalX:Number, p_normalY:Number, p_fraction:Number):Number
		{
			b2Assert(false, "current method isn't implemented yet or abstract and can't be used!");
			return 0;
		}
	}
}
