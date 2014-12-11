/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.b2AABB;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2Disposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * This proxy is used internally to connect fixtures to the broad-phase
	 */
	internal class b2FixtureProxy extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		//
		public var aabb:b2AABB;
		public var fixture:b2Fixture;
		public var childIndex:int;
		public var proxyId:int = -1;

		/**
		 */
		public function b2FixtureProxy()
		{
			aabb = b2AABB.Get();
		}

		/**
		 * @param p_fixtureProxy
		 */
		public function SetTo(p_fixtureProxy:b2FixtureProxy):void
		{
			aabb.SetTo(p_fixtureProxy.aabb);
			fixture = p_fixtureProxy.fixture;
			childIndex = p_fixtureProxy.childIndex;
			proxyId = p_fixtureProxy.proxyId;
		}

		/**
		 * NOTICE! to fixture prop set ref from this instance.
		 */
		override public function Clone():IDisposable
		{
			var proxy:b2FixtureProxy = Get();
			proxy.aabb.SetTo(aabb);
			proxy.fixture = fixture;
			proxy.childIndex = childIndex;
			proxy.proxyId = proxyId;
		}

		/**
		 */
		override public function Dispose():void
		{
			CONFIG::debug
			{
				super.Dispose();
			}

			aabb.Dispose();
			aabb = null;
			fixture = null;

			b2Disposable.Put(this, classId);
		}

		/**
		 * Returns new instance of b2FixtureProxy.
		 * @return b2FixtureProxy
		 */
		static public function Get():b2FixtureProxy
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var proxy:b2FixtureProxy;

			if (instance)
			{
				proxy = instance as b2FixtureProxy;
				proxy.aabb = b2AABB.Get();
				proxy.childIndex = 0;
				proxy.proxyId = -1;
			}
			else proxy = new b2FixtureProxy();

			return proxy;
		}
	}
}
