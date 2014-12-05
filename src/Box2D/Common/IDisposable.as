/**
 * User: VirtualMaestro
 * Date: 22.11.2014
 * Time: 23:37
 */
package Box2D.Common
{
	public interface IDisposable
	{
		/**
		 * Dispose instance. After disposing there is no possible of using instance.
		 */
		function Dispose():void;

		/**
		 * Determines if current instance was disposed already.
		 */
		function get IsDisposed():Boolean;

		/**
		 * Makes a clone of current instance and returns IDisposable.
		 * @return IDisposable
		 */
		function Clone():IDisposable;
	}
}
