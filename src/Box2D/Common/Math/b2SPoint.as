/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 14:32
 */
package Box2D.Common.Math
{
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Lightweight class represents of point.
	 */
	public class b2SPoint extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var x:Number;
		public var y:Number;

		/**
		 */
		public function b2SPoint(p_x:Number = 0, p_y:Number = 0)
		{
			x = p_x;
			y = p_y;
		}

		/**
		 * Copy x/y values from give object to current instance.
		 * It is possible to set any object which has x and y properties.
		 */
		public function SetObject(p_xy:*):void
		{
			x = p_xy.x;
			y = p_xy.y;
		}

		/**
		 * Dispose instance. After disposing there is no possible of using instance.
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			b2Disposable.Put(this, classId);
		}

		/**
		 * Makes a clone of current instance.
		 * @return instance of b2SPoint
		 */
		override public function Clone():IDisposable
		{
			return Get(x, y);
		}

		/**
		 * Returns new instance of b2SPoint.
		 * @return b2SPoint
		 */
		static public function Get(p_x:Number = 0, p_y:Number = 0):b2SPoint
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var point:b2SPoint;

			if (instance)
			{
				point = instance as b2SPoint;
				point.x = p_x;
				point.y = p_y;
			}
			else point = new b2SPoint(p_x, p_y);

			return point;
		}
	}
}
