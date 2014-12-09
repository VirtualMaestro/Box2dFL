/**
 * User: VirtualMaestro
 * Date: 09.12.2014
 * Time: 20:52
 */
package Box2D.Dynamics.Def
{
	/**
	 * A body definition holds all the data needed to construct a rigid body.
	 * You can safely re-use body definitions. Shapes are added to a body after construction.
	 */
	public class b2BodyDef
	{
		public var type:int;

		/**
		 * The world position of the body. Avoid creating bodies at the origin
		 * since this can lead to many overlapping shapes.
		 */
		public var positionX:Number;
		public var positionY:Number;

		/**
		 * The world angle of the body in radians.
		 */
		public var angle:Number;

		/**
		 * The linear velocity of the body's origin in world co-ordinates.
		 */
		public var linearVelocityX:Number;
		public var linearVelocityY:Number;

		/**
		 * The angular velocity of the body.
		 */
		public var angularVelocity:Number;

		/**
		 * Linear damping is use to reduce the linear velocity. The damping parameter
		 * can be larger than 1.0f but the damping effect becomes sensitive to the
		 * time step when the damping parameter is large.
		 */
		public var linearDamping:Number;

		/**
		 * Angular damping is use to reduce the angular velocity. The damping parameter
		 * can be larger than 1.0f but the damping effect becomes sensitive to the
		 * time step when the damping parameter is large.
		 */
		public var angularDamping:Number;

		/**
		 * Set this flag to false if this body should never fall asleep. Note that
		 * this increases CPU usage.
		 */
		public var allowSleep:Boolean;

		/**
		 * Is this body initially awake or sleeping?
		 */
		public var awake:Boolean;

		/**
		 * Should this body be prevented from rotating? Useful for characters.
		 */
		public var fixedRotation:Boolean;

		/**
		 * Is this a fast moving body that should be prevented from tunneling through
		 * other moving bodies? Note that all bodies are prevented from tunneling through
		 * kinematic and static bodies. This setting is only considered on dynamic bodies.
		 * @warning You should use this flag sparingly since it increases processing time.
		 */
		public var bullet:Boolean;

		/**
		 * Does this body start out active?
		 */
		public var active:Boolean;

		/**
		 * Scale the gravity applied to this body.
		 */
		public var gravityScale:Number;

		/**
		 * Use this to store application specific body data.
		 */
		public var userData:*;

		/**
		 */
		public function b2BodyDef()
		{
			positionX = 0.0;
			positionY = 0.0;
			angle = 0.0;
			linearVelocityX = 0.0;
			linearVelocityY = 0.0;
			angularVelocity = 0.0;
			linearDamping = 0.0;
			angularDamping = 0.0;
			allowSleep = true;
			awake = true;
			fixedRotation = false;
			bullet = false;
			type = 0;
			active = true;
			gravityScale = 1.0;
		}
	}
}
