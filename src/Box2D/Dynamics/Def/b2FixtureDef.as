/**
 * User: VirtualMaestro
 * Date: 02.12.2014
 * Time: 23:39
 */
package Box2D.Dynamics.Def
{
	import Box2D.Collision.Shapes.b2Shape;
	import Box2D.Dynamics.filter.b2Filter;

	/**
	 * A fixture definition is used to create a fixture. This class defines an
	 * abstract fixture definition. You can reuse fixture definitions safely.
	 */
	public class b2FixtureDef
	{
		/**
		 * The shape, this must be set. The shape will be cloned, so you
		 * can create the shape on the stack.
		 */
		public var shape:b2Shape;

		/**
		 * Use this to store application specific fixture data.
 		 */
		public var userData:*;

		/**
		 * The friction coefficient, usually in the range [0,1].
 		 */
		public var friction:Number;

		/**
		 * The restitution (elasticity) usually in the range [0,1].
 		 */
		public var restitution:Number;

		/**
		 * The density, usually in kg/m^2.
		 */
		public var density:Number;

		/**
		 * A sensor shape collects contact information but never generates a collision
		 * response.
		 */
		public var isSensor:Boolean;

		/**
		 * Contact filtering data.
 		 */
		public var filter:b2Filter;

		/**
		 */
		public function b2FixtureDef()
		{
			shape = null;
			userData = null;
			filter = null;
			friction = 0.2;
			restitution = 0.0;
			density = 0.0;
			isSensor = false;
		}
	}
}
