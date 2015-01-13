/**
 * User: VirtualMaestro
 * Date: 30.11.2014
 * Time: 16:04
 */
package Box2D.Common
{
	/**
	 * Global tuning constants based on meters-kilograms-seconds (MKS) units.
	 */
	public class b2Settings
	{
		/**
		 * The maximum number of contact points between two convex shapes.
		 * Do not change this value.
 		 */
		static public const maxManifoldPoints:int = 2;

		/**
		 * The maximum number of vertices on a convex polygon.
		 * You cannot increase this too much because b2BlockAllocator has a maximum object size.
		 */
		static public const maxPolygonVertices:int = 8;

		/**
		 * A small length used as a collision and constraint tolerance.
		 * Usually it is chosen to be numerically significant, but visually insignificant.
		 */
		static public const linearSlop:Number = 0.005;

		/**
		 * A small angle used as a collision and constraint tolerance.
		 * Usually it is chosen to be numerically significant, but visually insignificant.
		 */
		static public const angularSlop:Number = 2.0 / 180.0 * Math.PI;

		/**
		 * The radius of the polygon/edge shape skin. This should not be modified.
		 * Making this smaller means polygons will have an insufficient buffer for continuous collision.
		 * Making it larger may create artifacts for vertex collision.
 		 */
		static public const polygonRadius:Number = 2.0 * linearSlop;

		/**
		 * This is used to fatten AABBs in the dynamic tree.
		 * This allows proxies to move by a small amount without triggering a tree adjustment.
		 * This is in meters.
		 */
		static public const aabbExtension:Number = 0.1;

		/**
		 * This is used to fatten AABBs in the dynamic tree.
		 * This is used to predict the future position based on the current displacement.
		 * This is a dimensionless multiplier.
 		 */
		static public const aabbMultiplier:Number = 2.0;

		/**
		 * A velocity threshold for elastic collisions.
		 * Any collision with a relative linear velocity below this threshold will be treated as inelastic.
 		 */
		static public const velocityThreshold:Number = 1.0;

		/**
		* This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
		* that overlap is removed in one time step. However using values close to 1 often lead
		* to overshoot.
		*/
		static public const baumgarte:Number = 0.2;
		static public const toiBaugarte:Number = 0.75;

		/**
		 * The maximum linear position correction used when solving constraints.
		 * This helps to prevent overshoot.
 		 */
		static public const maxLinearCorrection:Number = 0.2;

		/**
		 * The maximum linear velocity of a body.
		 * This limit is very large and is used to prevent numerical problems.
		 * You shouldn't need to adjust this.
		 */
		static public const maxTranslation:Number = 2.0;
		static public const maxTranslationSquared:Number = (maxTranslation * maxTranslation);

		/**
		 * The maximum angular velocity of a body.
		 * This limit is very large and is used to prevent numerical problems.
		 * You shouldn't need to adjust this.
		 */
		static public const maxRotation:Number = (0.5 * Math.PI);
		static public const maxRotationSquared:Number = (maxRotation * maxRotation);

		/**
		 * A body cannot sleep if its linear velocity is above this tolerance.
		 */
		static public const linearSleepTolerance:Number = 0.01;

		/**
		 * A body cannot sleep if its angular velocity is above this tolerance.
		 */
		static public const angularSleepTolerance:Number = 	(2.0 / 180.0 * Math.PI);

		/**
		 * The time that a body must be still before it will go to sleep.
		 */
		static public const timeToSleep:Number = 0.5;

		/**
		 * Maximum number of contacts to be handled to solve a TOI impact.
		 */
		static public const maxTOIContacts:int = 32;

		/**
		 * Maximum number of sub-steps per contact in continuous physics simulation.
		 */
		static public const maxSubSteps:int = 8;


	}
}
