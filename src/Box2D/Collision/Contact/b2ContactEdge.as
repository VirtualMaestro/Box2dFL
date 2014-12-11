/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Collision.Contact
{
	import Box2D.Dynamics.b2Body;

	/**
	 * A contact edge is used to connect bodies and contacts together
	 * in a contact graph where each body is a node and each contact
	 * is an edge. A contact edge belongs to a doubly linked list
	 * maintained in each attached body.
	 * Each contact has two contact nodes, one for each attached body.
	 *
	 * TODO: Maybe need pool
	 */
	public class b2ContactEdge
	{
		public var other:b2Body;		///< provides quick access to the other body attached.
		public var contact:b2Contact;	///< the contact
		public var prev:b2ContactEdge;	///< the previous contact edge in the body's contact list
		public var next:b2ContactEdge;	///< the next contact edge in the body's contact list
	}
}
