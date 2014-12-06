/**
 * User: VirtualMaestro
 * Date: 27.11.2014
 * Time: 20:30
 */
package Box2D.Dynamics.filter
{
	import Box2D.Common.b2Disposable;
	import Box2D.Common.IDisposable;
	import Box2D.Common.b2internal;

	use namespace b2internal;

	/**
	 * Holds contact filtering data.
	 */
	public class b2Filter extends b2Disposable
	{
		static b2internal var classId:uint = b2Disposable.getClassId();

		/**
		 * The collision category bits. Normally you would just set one bit.
		 * Value by default 0x00000001
 		 */
		public var category:uint;

		/**
		 * The collision mask bits.
		 * This states the categories that this shape would accept for collision.
		 * Value by default 0xffffffff
 		 */
		public var mask:uint;

		/**
		 * Collision groups allow a certain group of objects to never collide (negative)
		 * or always collide (positive).
		 * Zero means no collision group.
		 * Non-zero group filtering always wins against the mask bits.
		 * Value by default 0x00000000
 		 */
		public var groupIndex:uint;

		/**
		 */
		public function b2Filter(p_category:uint = 0x00000001, p_mask:uint = 0xffffffff, p_groupIndex:uint = 0x00000000)
		{
			category = p_category;
			mask = p_mask;
			groupIndex = p_groupIndex;
		}

		/**
		 * Include category for possibility to collide with.
		 * @param p_includeCategory
		 */
		public function IncludeCollisionCategory(p_includeCategory:int):void
		{
			mask = IncludeCategory(mask, p_includeCategory);
		}

		/**
		 * Given category will not collide with current filter anymore
		 * @param p_excludeCategory
		 */
		public function ExcludeCollisionCategory(p_excludeCategory:int):void
		{
			mask = IncludeCategory(mask, p_excludeCategory);
		}

		/**
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
		 */
		override public function Clone():IDisposable
		{
			return Get(category, mask, groupIndex);
		}

		//************
		//*** UTILS **
		//************

		static private var _currentCategory:uint = 1;

		/**
		 * Method returns new unique category.
		 * There is 31 possible categories.
		 *
		 * To make sure that there is free category use 'hasCategory' method.
		 */
		static public function GetCategory():uint
		{
			return (1 << (++_currentCategory - 1));
		}

		/**
		 * Check whether there is free category.
		 */
		static public function HasCategory():Boolean
		{
			return (_currentCategory + 1) < 33;
		}

		/**
		 * Reset collision category's counter, so it is possible to get categories from the beginning.
		 */
		static public function ResetCategoryCounter():void
		{
			_currentCategory = 1;
		}

		/**
		 * Create setup filter.
		 *
		 * @param p_category - category for filter
		 * @param p_categoriesInclude - array with categories which should collide with this category.
		 * @param p_categoriesExclude - array with categories which shouldn't collide with this category.
		 * @return b2Filter
		 */
		static public function GetFilter(p_category:int = 1, p_categoriesInclude:Array = null, p_categoriesExclude:Array = null):b2Filter
		{
			return Get(p_category, GetMask(p_categoriesInclude, p_categoriesExclude));
		}

		/**
		 * Returns the mask by given include or exclude categories.
		 * We shouldn't set include and exclude categories at the same time - hasn't sense.
		 *
		 * For avoiding incorrect values for categories use getGroup method for generating category's value.
		 * Returns number which represents mask for given include/exclude categories.
		 */
		static private function GetMask(p_includeGroups:Array = null, p_excludeGroups:Array = null):int
		{
			if (p_includeGroups == null && p_excludeGroups == null) return 0xffffffff;

			var mask:int;
			var len:int;
			var i:int;

			//
			if (p_includeGroups)
			{
				len = p_includeGroups.length;

				if (len < 1) mask = 0xffffffff;
				else
				{
					mask = 0x00000000;
					for (i = 0; i < len; i++)
					{
						mask |= p_includeGroups[i];
					}
				}
			}
			else if (p_excludeGroups)
			{
				len = p_excludeGroups.length;

				if (len < 1) mask = 0x00000000;
				else
				{
					mask = 0xffffffff;

					for (i = 0; i < len; i++)
					{
						mask &= (~(p_excludeGroups[i]));
					}
				}
			}

			return mask;
		}

		/**
		 * Returns new mask with included new category for colliding.
		 */
		static public function IncludeCategory(p_filterMask:int, p_includingGroup:int):int
		{
			return p_filterMask | p_includingGroup;
		}

		/**
		 * Returns new mask with excluded given category for colliding.
		 */
		static public function ExcludeCategory(p_filterMask:int, p_excludingGroup:int):int
		{
			return p_filterMask & (~p_excludingGroup);
		}

		/**
		 * Returns new instance of b2Filter.
		 * @return b2Filter
		 */
		static public function Get(p_categoryBits:uint = 0x00000001, p_maskBits:uint = 0xffffffff, p_groupIndex:uint = 0x00000000):b2Filter
		{
			var instance:b2Disposable = b2Disposable.Get(classId);
			var filter:b2Filter;

			if (instance) filter = instance as b2Filter;
			else filter = new b2Filter();

			filter.category = p_categoryBits;
			filter.mask = p_maskBits;
			filter.groupIndex = p_groupIndex;

			return filter;
		}
	}
}
