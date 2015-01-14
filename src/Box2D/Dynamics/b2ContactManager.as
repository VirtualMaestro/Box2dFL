/**
 * Created by oreva on 11.12.2014.
 */
package Box2D.Dynamics
{
	import Box2D.Collision.Contact.b2Contact;
	import Box2D.Collision.Contact.b2ContactEdge;
	import Box2D.Collision.b2BroadPhase;
	import Box2D.Common.b2internal;
	import Box2D.Dynamics.Callbacks.b2ContactListener;
	import Box2D.Dynamics.Filters.b2Filter;

	use namespace b2internal;

	/**
	 * Delegate of b2World.
	 */
	public class b2ContactManager
	{
		public var m_broadPhase:b2BroadPhase;
		public var m_contactList:b2Contact;
		public var m_contactCount:int;
		public var m_contactFilter:b2Filter;
		public var m_contactListener:b2ContactListener;

		/**
		 */
		public function b2ContactManager()
		{
			m_contactCount = 0;
			m_contactFilter = new b2Filter();
			m_contactListener = new b2ContactListener();
			m_broadPhase = new b2BroadPhase();
		}

		/**
		 * Broad-phase callback.
		 */
		public function AddPair(p_proxyUserDataA:*, p_proxyUserDataB:*):void
		{
			var proxyA:b2FixtureProxy = p_proxyUserDataA;
			var proxyB:b2FixtureProxy = p_proxyUserDataB;

			var fixtureA:b2Fixture = proxyA.fixture;
			var fixtureB:b2Fixture = proxyB.fixture;

			var indexA:int = proxyA.childIndex;
			var indexB:int = proxyB.childIndex;

			var bodyA:b2Body = fixtureA.GetBody();
			var bodyB:b2Body = fixtureB.GetBody();

			// Are the fixtures on the same body?
			if (bodyA == bodyB)
			{
				return;
			}

			// TODO: use a hash table to remove a potential bottleneck when both bodies have a lot of contacts.
			// Does a contact already exist?
			var edge:b2ContactEdge = bodyB.GetContactList();

			while (edge)
			{
				if (edge.other == bodyA)
				{
					var contact:b2Contact = edge.contact;
					var fA:b2Fixture = contact.GetFixtureA();
					var fB:b2Fixture = contact.GetFixtureB();
					var iA:int = contact.GetChildIndexA();
					var iB:int = contact.GetChildIndexB();

					if (fA == fixtureA && fixtureB && iA == indexA && iB == indexB)
					{
						// A contact already exists
						return;
					}

					if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
					{
						// A contact already exists.
						return;
					}
				}

				edge = edge.next;
			}

			// Does a joint override collision? Is at least one body dynamic?
			if (bodyB.ShouldCollide(bodyA) == false)
			{
				return;
			}

			// Check user filtering.
			if (m_contactFilter && b2Filter.ShouldCollide(fixtureA, fixtureB) == false)
			{
				return;
			}

			// Call the factory.
			var c:b2Contact = b2Contact.Create(fixtureA, indexA, fixtureB, indexB);
			if (c == null)
			{
				return;
			}

			// Contact creation may swap fixtures.
			fixtureA = c.GetFixtureA();
			fixtureB = c.GetFixtureB();
//			indexA = c.GetChildIndexA();
//			indexB = c.GetChildIndexB();
			bodyA = fixtureA.GetBody();
			bodyB = fixtureB.GetBody();

			// Insert into the world.
			c.m_prev = null;
			c.m_next = m_contactList;

			if (m_contactList != null)
			{
				m_contactList.m_prev = c;
			}

			m_contactList = c;

			// Connect to island graph.

			// Connect to body A
			c.m_nodeA.contact = c;
			c.m_nodeA.other = bodyB;

			c.m_nodeA.prev = null;
			c.m_nodeA.next = bodyA.m_contactList;
			if (bodyA.m_contactList != null)
			{
				bodyA.m_contactList.prev = c.m_nodeA;
			}
			bodyA.m_contactList = c.m_nodeA;

			// Connect to body B
			c.m_nodeB.contact = c;
			c.m_nodeB.other = bodyA;

			c.m_nodeB.prev = null;
			c.m_nodeB.next = bodyB.m_contactList;
			if (bodyB.m_contactList != null)
			{
				bodyB.m_contactList.prev = c.m_nodeB;
			}
			bodyB.m_contactList = c.m_nodeB;

			// Wake up the bodies
			if (fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}

			++m_contactCount;
		}

		/**
		 */
		public function FindNewContacts():void
		{
			m_broadPhase.UpdatePairs(this);
		}

		/**
		 * This is the top level collision call for the time step. Here
		 * all the narrow phase collision is processed for the world
		 * contact list.
		 */
		public function Collide():void
		{
			// Update awake contacts.
			var c:b2Contact = m_contactList;
			var fixtureA:b2Fixture;
			var fixtureB:b2Fixture;
			var indexA:int;
			var indexB:int;
			var bodyA:b2Body;
			var bodyB:b2Body;
			var cNuke:b2Contact;

			while (c)
			{
				fixtureA = c.GetFixtureA();
				fixtureB = c.GetFixtureB();
				indexA = c.GetChildIndexA();
				indexB = c.GetChildIndexB();
				bodyA = fixtureA.GetBody();
				bodyB = fixtureB.GetBody();

				// Is this contact flagged for filtering?
				if ((c.m_flags & b2Contact.e_filterFlag) != 0)
				{
					// Should these bodies collide?
					if (bodyB.ShouldCollide(bodyA) == false)
					{
						cNuke = c;
						c = cNuke.GetNext();
						Destroy(cNuke);
						continue;
					}

					// Check user filtering.
					if (m_contactFilter && b2Filter.ShouldCollide(fixtureA, fixtureB) == false)
					{
						cNuke = c;
						c = cNuke.GetNext();
						Destroy(cNuke);
						continue;
					}

					// Clear the filtering flag.
					c.m_flags &= ~b2Contact.e_filterFlag;
				}

				var activeA:Boolean = bodyA.IsAwake() && bodyA.m_type != b2Body.STATIC;
				var activeB:Boolean = bodyB.IsAwake() && bodyB.m_type != b2Body.STATIC;

				// At least one body must be awake and it must be dynamic or kinematic.
				if (activeA == false && activeB == false)
				{
					c = c.GetNext();
					continue;
				}

				var proxyIdA:int = fixtureA.m_proxies[indexA].proxyId;
				var proxyIdB:int = fixtureB.m_proxies[indexB].proxyId;
				var overlap:Boolean = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

				// Here we destroy contacts that cease to overlap in the broad-phase.
				if (overlap == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}

				// The contact persists.
				c.Update(m_contactListener);
				c = c.GetNext();
			}
		}

		/**
		 */
		public function Destroy(c:b2Contact):void
		{
			var fixtureA:b2Fixture = c.GetFixtureA();
			var fixtureB:b2Fixture = c.GetFixtureB();
			var bodyA:b2Body = fixtureA.GetBody();
			var bodyB:b2Body = fixtureB.GetBody();

			if (m_contactListener && c.IsTouching())
			{
				m_contactListener.EndContact(c);
			}

			var prev:b2Contact = c.m_prev;
			var next:b2Contact = c.m_next;

			// Remove from the world.
			if (prev)
			{
				prev.m_next = next;
			}

			if (next)
			{
				next.m_prev = prev;
			}

			if (c == m_contactList)
			{
				m_contactList = next;
			}

			var nodeAPrev:b2ContactEdge = c.m_nodeA.prev;
			var nodeANext:b2ContactEdge = c.m_nodeA.next;

			// Remove from body 1
			if (nodeAPrev)
			{
				nodeAPrev.next = nodeANext;
			}

			if (nodeANext)
			{
				nodeANext.prev = nodeAPrev;
			}

			if (c.m_nodeA == bodyA.m_contactList)
			{
				bodyA.m_contactList = nodeANext;
			}

			var nodeBPrev:b2ContactEdge = c.m_nodeB.prev;
			var nodeBNext:b2ContactEdge = c.m_nodeB.next;

			// Remove from body 2
			if (nodeBPrev)
			{
				nodeBPrev.next = nodeBNext;
			}

			if (nodeBNext)
			{
				nodeBNext.prev = nodeBPrev;
			}

			if (c.m_nodeB == bodyB.m_contactList)
			{
				bodyB.m_contactList = nodeBNext;
			}

			// Call the factory.
			b2Contact.Destroy(c);

			--m_contactCount;
		}
	}
}
