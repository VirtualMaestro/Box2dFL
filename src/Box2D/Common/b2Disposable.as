/**
 * User: VirtualMaestro
 * Date: 29.11.2014
 * Time: 0:24
 */
package Box2D.Common
{
	import Box2D.b2Assert;

	use namespace b2internal;

	/**
	 * Abstract class. Partial implementation for IDisposable interface.
	 */
	public class b2Disposable implements IDisposable
	{
		/**
		 */
		b2internal var disposed:Boolean = false;

		/**
		 */
		public function b2Disposable()
		{
		}

		/**
		 * Disposed current instance and return to pool (if current implementation able to do this).
		 */
		public function Dispose():void
		{
			CONFIG::debug
			{
				b2Assert(!disposed, "attempt to destroy the already destroyed instance");
			}
		}

		/**
		 * Returns "true" if current instance is disposed.
		 */
		[Inline]
		final public function get IsDisposed():Boolean
		{
			return disposed;
		}

		/**
		 * Returns copy of current instance.
		 * @return IDisposable
		 */
		public function Clone():IDisposable
		{
			b2Assert(false, "for current class 'Clone' method wasn't overridden, so you can't use this method");
			return null;
		}

		//**************//
		// UTIL METHODS //
		//**************//

		static private var _classUID:uint = 0;

		/**
		 */
		static b2internal function getClassId():uint
		{
			return _classUID++;
		}

		/**
		 * Just remove elements from vector.
		 */
		[Inline]
		static public function clearVector(p_vector:*):void
		{
			var len:int = p_vector.length;
			for (var i:int = 0; i < len; i++)
			{
				p_vector[i] = null;
			}

			p_vector.length = 0;
		}

		/**
		 * Clear vector and every element will disposed (invokes its Dispose method)
		 * @param p_vector has to be Vector with type b2Disposable
		 */
		[Inline]
		static public function clearVectorWithDispose(p_vector:*):void
		{
			CONFIG::debug
			{
				b2Assert((p_vector is Vector.<b2Disposable>), "vector has to be Vector.<b2Disposable> type");
			}

			var vec:Vector.<b2Disposable> = p_vector as Vector.<b2Disposable>;
			var len:int = vec.length;

			for (var i:int = 0; i < len; i++)
			{
				vec[i].Dispose();
				vec[i] = null;
			}

			vec.length = 0;
		}

		//***********
		//** POOL****
		//***********

		static private var _pool:Vector.<Vector.<b2Disposable>> = new Vector.<Vector.<b2Disposable>>(b2Settings.maxTypesInPool);
		static private var _counts:Vector.<int> = new Vector.<int>(b2Settings.maxTypesInPool);

		/**
		 * Returns disposable instance corresponds to given class id.
		 * @return b2Disposable
		 */
		static b2internal function Get(p_classId:uint):b2Disposable
		{
			CONFIG::debug
			{
				b2Assert(p_classId < _counts.length, "not enough elements for pool");
			}

			var count:int = _counts[p_classId];
			var disposable:b2Disposable;

			if (count > 0)
			{
				var pool:Vector.<b2Disposable> = _pool[p_classId];

				disposable = pool[--count];
				disposable.disposed = false;
				pool[count] = null;
				_counts[p_classId] = count;
			}

			return disposable;
		}

		/**
		 * Put instance of b2Shape to pool.
		 */
		[Inline]
		static b2internal function Put(p_instance:b2Disposable, p_classId:uint):void
		{
			CONFIG::debug
			{
				b2Assert(!p_instance.disposed, "try to add already disposed shape to pool");
			}

			p_instance.disposed = true;

			var count:int = _counts[p_classId];

			if (count < 0)
			{
				_pool[p_classId] = new <b2Disposable>[];
			}

			_pool[p_classId][count++] = p_instance;
			_counts[p_classId] = count;
		}

		/**
		 * Clear pool for GC.
		 */
		[Inline]
		static b2internal function Rid():void
		{
			var len:int = _pool.length;

			for (var i:int = 0; i < len; i++)
			{
				b2Disposable.clearVector(_pool[i]);
				if (_counts[i] > 0) _counts[i] = 0;
			}
		}
	}
}
