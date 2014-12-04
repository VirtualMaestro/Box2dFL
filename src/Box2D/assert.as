/**
 * User: VirtualMaestro
 * Date: 23.11.2014
 * Time: 20:07
 */
package Box2D
{
	public function assert(p_expression:Boolean, p_message:String, p_where:String = null):void
	{
		 if (!p_expression)
		 {
			 var where:String = p_where ? " in " + p_where + ": " : ": ";
			 throw new Error("Exception" + where + p_message);
		 }
	}
}
