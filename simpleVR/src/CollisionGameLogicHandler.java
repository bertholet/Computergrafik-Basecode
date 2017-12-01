import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.InternalTickCallback;
import com.bulletphysics.dynamics.RigidBody;

import jrtr.PhysicalShape;


/**
 * Example class to track collisions of a single object t
 * To make it a bit more elegant - use listener/observer pattern to 
 * subscribe to collisions of specific objects
 * @author Alf
 *
 */
public class CollisionGameLogicHandler extends InternalTickCallback{

	private RigidBody shape;
	private DynamicsWorld dynamicWorld;
	public CollisionGameLogicHandler(DynamicsWorld world, RigidBody s) {
		shape = s;
		dynamicWorld = world;
		
	}
	@Override
	public void internalTick(DynamicsWorld world, float timeStep) {
		//list of all current collisions.
		int numManifolds = dynamicWorld.getDispatcher().getNumManifolds();
		int numCurrentContacts = 0;
	    for (int i = 0; i < numManifolds; i++)
	    {
	        PersistentManifold contactManifold = dynamicWorld.getDispatcher().getManifoldByIndexInternal(i);
	        Object obA = contactManifold.getBody0();
	        Object obB = contactManifold.getBody1();
	        if(obA == shape || obB == shape) {
	        	numCurrentContacts++;
	        }
	    }
	    System.out.println(shape + " has: "+ numCurrentContacts + "collisions");
	}

}
