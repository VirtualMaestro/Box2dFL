/**
 * User: VirtualMaestro
 * Date: 09.01.2015
 * Time: 19:13
 */
package Box2D.Collision.Structures
{
	/**
	 * This is an internal structure.
	 */
	public class b2TimeStep
	{
		public var dt:Number;			// time step
		public var inv_dt:Number;		// inverse time step (0 if dt == 0).
		public var dtRatio:Number;	// dt * inv_dt0
		public var velocityIterations:int;
		public var positionIterations:int;
		public var warmStarting:Boolean;
	}
}
