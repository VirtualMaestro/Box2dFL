/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 15:59
 */
package Box2D.Dynamics.Callbacks
{
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;

	/**
	 *
	 */
	public class QueryCallback extends b2QueryCallback
	{
		public var m_pointX:Number;
		public var m_pointY:Number;
		public var m_fixture:b2Fixture;

		/**
		 */
		public function QueryCallback(p_pointX:Number, p_pointY:Number)
		{
			m_pointX = p_pointX;
			m_pointY = p_pointY;
		}

		/**
		 */
		override public function ReportFixture(p_fixture:b2Fixture):Boolean
		{
			var body:b2Body = p_fixture.GetBody();

			if (body.GetType() == b2Body.DYNAMIC)
			{
				var inside:Boolean = p_fixture.TestPoint(m_pointX, m_pointY);
				if (inside)
				{
					m_fixture = p_fixture;

					// We are done, terminate the query.
					return false;
				}
			}

			// Continue the query.
			return true;
		}
	}
}
