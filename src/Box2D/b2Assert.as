/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 20:07
 */
package Box2D
{
	import flash.utils.getQualifiedClassName;

	public function b2Assert(p_expression:Boolean, p_message:String, p_causeObject:* = null):void
	{
		if (!p_expression)
		{
			var error:Error = new Error();
			var stackTrace:String = error.getStackTrace();
			var eIndex:int = stackTrace.indexOf("]") + 1;
			var outStackTrace:String = stackTrace.substring(eIndex);

			trace("EXCEPTION!: " + p_message);

			if (p_causeObject)
			{
				trace("ADDITIONAL: ");
				trace(" - " + getQualifiedClassName(p_causeObject));
				trace(" - " + p_causeObject.toString());
			}

			trace("STACK TRACE:");
			trace(outStackTrace);

			throw error;
		}
	}
}
