/**
 * User: VirtualMaestro
 * Date: 10.01.2015
 * Time: 0:15
 */
package Box2D.Common.Math
{
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * A 2D column vector with 3 elements.
	 */
	public class b2Vec3 extends b2SPoint
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var z:Number;

		/**
		 */
		public function b2Vec3(p_x:Number = 0, p_y:Number = 0, p_z:Number = 0)
		{
			super(p_x, p_y);
			z = p_z;
		}

		/**
		 * Copy x/y/z values from give object to current instance.
		 * It is possible to set any object which has x and y properties.
		 */
		override public function SetObject(p_xyz:*):void
		{
			x = p_xyz.x;
			y = p_xyz.y;
			z = p_xyz.z;
		}

		/**
		 * Returns copy of current b2Vec2 instance.
		 */
		final override public function Clone():IDisposable
		{
			return Get(x, y, z);
		}

		/**
		 * Dispose instance. After disposing there is no possible of using instance.
		 */
		final override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns new instance of b2SPoint.
		 * @return b2SPoint
		 */
		static public function Get(p_x:Number = 0, p_y:Number = 0, p_z:Number = 0):b2Vec3
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var point:b2Vec3;

			if (instance)
			{
				point = instance as b2Vec3;
				point.x = p_x;
				point.y = p_y;
				point.z = p_z;
			}
			else point = new b2Vec3(p_x, p_y, p_z);

			return point;
		}
	}
}
