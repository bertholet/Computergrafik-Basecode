import javax.swing.JFrame;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;

import jrtr.Camera;
import jrtr.Material;
import jrtr.RenderContext;
import jrtr.Shader;
import jrtr.Shape;
import jrtr.SimpleSceneManager;
import jrtr.VertexData;
import jrtr.glrenderer.VRRenderPanel;

/**
 * Implements a simple VR application that renders to an HMD via
 * OpenVR. OpenVR functionality is provided by the {@link OpenVRRenderPanel}.
 * Also demonstrates tracking of a VR hand controller.
 */
public class SimpleOpenVRwithPhysics
{	
	
	//scene-geometry parameters
	static float ballRadius = 0.15f;
	static float room_size = 5.f;
	static float controllerSize = 0.015f;
	static float racketLength = 1.f;
	
	
	static VRRenderPanel renderPanel;
	public static RenderContext renderContext;
	
	//an object representing one of the controllers
	static Shape racket;

	//example scene. These two objects have two representations, one for rendering
	//one for the physics simulation
	static Shape dynamicCube; 						//for rendering
	static RigidBody dynamicCube_collisionShape;	//for simulation
	
	static Shape staticFloor;						//for rendering
	static RigidBody staticFloor_collisionShape;	//for simulation
	
	//The dynamicsWorld which manages the RigidObjects for the simulation
	public static DynamicsWorld dynamicWorld;
	//The scenemanager for rendering.
	public static SimpleSceneManager sceneManager;
	
	//Timestamp to compute correct timesteps for the physics simulation
	public static long animLastTimeInMS = -1;
	
	//updating frame per seconds in the title	
	public static JFrame jframe;
	public static int frameCounter = 0;
	public static long startTime;

	public static Camera camera;
	
	public static Shape makeBoxShape(RenderContext renderContext, float wx, float wy, float wz) {
		Shape box = new Shape(makeBoxVertexData(renderContext, wx, wy, wz));
		Material mat = new Material();
		box.setMaterial(mat);
		mat.specular = new Vector3f(0,0,0);
		mat.shininess = 0;
		return box;
	}
	
	public static VertexData makeBoxVertexData(RenderContext renderContext, float wx, float wy, float wz) {
		float vertices[] = {-wx,-wy,wz, wx,-wy,wz, wx,wy,wz, -wx,wy,wz,		// front face
							-wx,-wy,-wz, -wx,-wy,wz, -wx,wy,wz, -wx,wy,-wz, // left face
							wx,-wy,-wz,-wx,-wy,-wz, -wx,wy,-wz, wx,wy,-wz,  // back face
							wx,-wy,wz, wx,-wy,-wz, wx,wy,-wz, wx,wy,wz,		// right face
							wx,wy,wz, wx,wy,-wz, -wx,wy,-wz, -wx,wy,wz,		// top face
							-wx,-wy,wz, -wx,-wy,-wz, wx,-wy,-wz, wx,-wy,wz}; // bottom face								
	
		float normals[] = {0,0,1,  0,0,1,  0,0,1,  0,0,1,		// front face
						   -1,0,0, -1,0,0, -1,0,0, -1,0,0,		// left face
						   0,0,-1, 0,0,-1, 0,0,-1, 0,0,-1,		// back face
						   1,0,0,  1,0,0,  1,0,0,  1,0,0,		// right face
						   0,1,0,  0,1,0,  0,1,0,  0,1,0,		// top face
						   0,-1,0, 0,-1,0, 0,-1,0, 0,-1,0};		// bottom face
						   
		float colors[] = {1,0,0, 1,0,0, 1,0,0, 1,0,0,
						  0,1,0, 0,1,0, 0,1,0, 0,1,0,
						  1,0,0, 1,0,0, 1,0,0, 1,0,0,
						  0,1,0, 0,1,0, 0,1,0, 0,1,0,
						  0,0,1, 0,0,1, 0,0,1, 0,0,1,
						  0,0,1, 0,0,1, 0,0,1, 0,0,1};
		
		// Set up the vertex data
		VertexData vertexData = renderContext.makeVertexData(24);
	
		// Specify the elements of the vertex data:
		// - one element for vertex positions
		vertexData.addElement(vertices, VertexData.Semantic.POSITION, 3);
		// - one element for vertex colors
		vertexData.addElement(colors, VertexData.Semantic.COLOR, 3);
		// - one element for vertex normals
		vertexData.addElement(normals, VertexData.Semantic.NORMAL, 3);
		
		// The index data that stores the connectivity of the triangles
		int indices[] = {0,2,3, 0,1,2,			// front face
						 4,6,7, 4,5,6,			// left face
						 8,10,11, 8,9,10,		// back face
						 12,14,15, 12,13,14,	// right face
						 16,18,19, 16,17,18,	// top face
						 20,22,23, 20,21,22};	// bottom face
			
		vertexData.addIndices(indices);
		return vertexData;
	}
	

	
	public static void createRacket() {
		// Make a racket in form of a rectangular box
		float thickness = 0.05f;
		float width = 0.1f;
		racket = makeBoxShape(renderContext, thickness, width, racketLength);
		sceneManager.addShape(racket);
		
	}
	
	public static void setupScene() {
			
		float wall_thickness = 0.1f;
		createRacket();

		/* TODO: VR exercise one: Get to know JBullet better by integrating it into JRTR.
		 * refactor the code below, such that for example for the floor it reduces to something like
		 * 
		 * staticFloor = new BoxCollisionShape(makeBoxVertexData(...),floorDimensions,mass,dynamicScene);
		 * staticFloor.setTransformation(floorWorldTransf);
		 * staticFloor.setBehaviour(bouncyness, friction, damping);
		 * sceneManager.addShape(staticFloor);
		 */
		
		//--------------------
		//Example 1: static ground shape (the floor)
		//-------------------------
		//create a floor shape for graphics
		Vector3f floorDimensions = new Vector3f(room_size,wall_thickness,room_size);
		
		Vector3f floorPos = new Vector3f(0,-3,0);
		Matrix4f floorWorldTransf = new Matrix4f();
		floorWorldTransf.setIdentity();
		floorWorldTransf.setTranslation(floorPos);
		
		staticFloor = new Shape(makeBoxVertexData(renderContext,floorDimensions.x,floorDimensions.y,floorDimensions.z));
		staticFloor.setTransformation(floorWorldTransf);
		
		//create a representation of the floor for physics.
		//a shape...
		BoxShape shape = new BoxShape(floorDimensions);
		
		//...and its physical properties: mass(inertia), angular inertia etc. Play with those for
		//different behaviors.
		//by convention, a mass of zero makes a body static, and unaffected by forces like gravity
		float mass = 0f; 
		//The angular moment of inertia. All Jbullet shapes
		//have predifined functions to compute it.
	    Vector3f angularInertia = new Vector3f(0,0,0);
	    shape.calculateLocalInertia(mass, angularInertia);	  
		float bouncyness = 0.7f;
		float friction = 0.5f;
		float damping = 0.1f;
	    
		//The motion state is an abstract class to allow the simple communication between
		//the physics and the graphics world. One can implement its own by extending MotionState
		//to handle when a physical objects worldTransform changed, or one can just 
		//use a defaultMotionState and query the wordTransform as needed.
		//The world transforms of the physical and the graphic object should coincide!
	    MotionState ms = new DefaultMotionState(new Transform(floorWorldTransf));
	    
	    //All is bundled up in an rbInfo object
	    RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, ms, shape, angularInertia);
	    //rbInfo.restitution = 0.95f; restitution etc can be set either on rbInfo or the Rigid body
	    //And finally create the full RigidBody that can be used in the simulation
	    staticFloor_collisionShape = new RigidBody(rbInfo);
	    staticFloor_collisionShape.setRestitution(bouncyness);
	    staticFloor_collisionShape.setFriction(friction);
	    staticFloor_collisionShape.setDamping(damping, damping);
	    
	    sceneManager.addShape(staticFloor);
	    dynamicWorld.addRigidBody(staticFloor_collisionShape);
	    
	    //Example 2: Dynamic box shape.	
		// a test box which once the animation starts will fall down and bounce a few times
	    //the setup is the same as for the floor, only that the mass is nonzero
	    Vector3f boxDimensions = new Vector3f(0.6f,0.6f,0.6f);
	    Vector3f boxPos = new Vector3f(-1,3,0);
		Matrix4f boxWorldTransf = new Matrix4f();
		boxWorldTransf.setIdentity();
		boxWorldTransf.setTranslation(boxPos);
		
		dynamicCube = new Shape(makeBoxVertexData(renderContext,  boxDimensions.x,boxDimensions.y,boxDimensions.z));
		dynamicCube.setTransformation(boxWorldTransf);
		BoxShape cubeshape = new BoxShape(boxDimensions);
		//setting the mass proportional to the volume makes sense, or set it to your liking
		mass = boxDimensions.x*boxDimensions.y*boxDimensions.z; 
		friction = 10;
		damping = 0.1f;
		bouncyness = 0.8f;
	    angularInertia = new Vector3f(0,0,0);
	    cubeshape.calculateLocalInertia(mass, angularInertia);	  

	    ms = new DefaultMotionState(new Transform(boxWorldTransf));
	    rbInfo = new RigidBodyConstructionInfo(mass, ms, cubeshape, angularInertia);
	    
	    dynamicCube_collisionShape = new RigidBody(rbInfo);
	    dynamicCube_collisionShape.setRestitution(bouncyness);
	    dynamicCube_collisionShape.setFriction(friction);
	    dynamicCube_collisionShape.setDamping(damping, damping);
	    
	    sceneManager.addShape(dynamicCube);
	    dynamicWorld.addRigidBody(dynamicCube_collisionShape);
	

		// example of how to keeping track of collisions
	    //note JBullet also has GhostObject (s), an object that can be added
	    //to the physical world, cannot be collided but report intersections
	    //this can for example be used to implement triggers or to check if
	    //some target is hit.
		dynamicWorld.setInternalTickCallback(new CollisionGameLogicHandler(dynamicWorld, staticFloor_collisionShape), null);   
		
	}


	/**
	 * An extension of {@link OpenVRRenderPanel} to 
	 * provide a call-back function for initialization. 
	 */ 
	public final static class SimpleVRRenderPanel extends VRRenderPanel
	{	
		/**
		 * Initialization call-back. We initialize our renderer here.
		 * 
		 * @param r	the render context that is associated with this render panel
		 */
		public void init(RenderContext r)
		{	
			sceneManager = new SimpleSceneManager();
			renderContext = r;
			renderContext.setSceneManager(sceneManager);
			setupScene();
	     
			// Set camera and frustum
			camera = sceneManager.getCamera();
			Vector3f pos = new Vector3f(0,-1.f,-0.3f);
			Vector3f lookAt = new Vector3f(0,-1.f,1);
			Vector3f up = new Vector3f(0,1,0);

			sceneManager.getCamera().setCenterOfProjection(pos);
			sceneManager.getCamera().setLookAtPoint(lookAt);
			sceneManager.getCamera().setUpVector(up);
			
		    
		}
		
		public void dispose()
		{

		}

		/*
		 * Helper function to visualise the controller corresponding to the left hand.
		 * Optionally, for example, you can implement a feedback (visual, audio, etc.) when trigger is activated.
		 * Index determines the controller (left or right).
		 */
		private Matrix4f visualizeLeftHand() {
			// Show a hand (box, sphere, etc.)
			
			int index = renderPanel.controllerIndexHand;
			if (index != -1 && renderPanel.getTriggerTouched(index)) {
				/*
				 * Optionally do something when trigger of hand is activated
				 */
			}

			return new Matrix4f();
		}
		
		/*
		 * Helper function to visualise the controller corresponding to the racket.
		 * Returns the trafo of the controller. 
		 */
		private static Matrix4f visualizeRacket(int index)
		{		
			Matrix4f racketT = new Matrix4f();
			racketT.setIdentity();
			if(index != -1) 
			{	
    			//current shape follows the controller
    			racketT = new Matrix4f(sceneManager.getCamera().getCameraMatrix());
    			racketT.invert();
    			racketT.mul(renderPanel.poseMatrices[index]);
    			
    			Matrix4f translateRacket = new Matrix4f();
    			translateRacket.setIdentity();
    			translateRacket.setTranslation(new Vector3f(0,0,-racketLength));
    		
    			racketT.mul(translateRacket);
    			racket.setTransformation(racketT);
    		}			
			return racketT;
		}

		
		/*
		 * Override from base class. This function is called before every render pass
		 * about at 90 fps
		 */
		public void prepareDisplay()
		{
			// check if button on side is pressed
    		if(renderPanel.getSideTouched(renderPanel.controllerIndexHand))
    		{
    			// do something if button is pressed	
    		}
    		
    		countFrames();
    		
    		renderPanel.waitGetPoses();
			
			if(animLastTimeInMS<0 ) {
				animLastTimeInMS = System.currentTimeMillis();
			}
			long currentTime = System.currentTimeMillis();
			Matrix4f racketTransform = visualizeRacket(renderPanel.controllerIndexRacket);	
			
			// Time passed since last call
			float delta = (currentTime - animLastTimeInMS)/1000f;
    		dynamicWorld.stepSimulation(delta);
    		animLastTimeInMS = currentTime;
    		
    		
    		/* TODO if task 1 is completed, these lines are obsolete.
    		 * Here we manually update the graphic worldTransforms 
    		 * from the physical transforms
    		 */
    		Transform target = new Transform();
    		Matrix4f mat = new Matrix4f();
    		dynamicCube_collisionShape.getMotionState().getWorldTransform(target);
    		target.getMatrix(mat);
    		dynamicCube.setTransformation(mat);
    		System.out.println(mat.toString());
    		
    		/* not needed for static floor...
    		mat = new Matrix4f();
    		staticFloor_collisionShape.getWorldTransform(target);
    		target.getMatrix(mat);
    		staticFloor.setTransformation(mat);
    		System.out.println(mat.toString());*/
    		
    		
    		/* TODO
    		 * Add your own game logic
    		 */
    	}

		
		private void countFrames() {
			frameCounter++;
			if(frameCounter==10)
			{
				String s = new String(String.valueOf(1/((float)(System.currentTimeMillis()-startTime)/10000.f)));
				jframe.setTitle(s.concat(" FPS"));
				frameCounter = 0;
				startTime = System.currentTimeMillis();
			}
		}
		
		
}

	
	/**
	 * The main function opens a 3D rendering window, constructs a simple 3D
	 * scene, and starts a timer task to generate an animation.
	 */
	public static void main(String[] args)
	{		
		// Make a render panel. The init function of the renderPanel
		// (see above) will be called back for initialization.
		renderPanel = new SimpleVRRenderPanel();
		
		// Make the main window of this application and add the renderer to it
		jframe = new JFrame("simple");
		jframe.setSize(1680, 1680);
		jframe.setLocationRelativeTo(null); // center of screen
		jframe.getContentPane().add(renderPanel.getCanvas());// put the canvas into a JFrame window

		// Add a mouse listener
		//  renderPanel.getCanvas().addMouseListener(new SimpleMouseListener());
		renderPanel.getCanvas().setFocusable(true);
		
		//physics, and configuring the algorithms to be used. you will not need to modify this.
		//narrow phase method
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		Vector3f worldAabbMin = new Vector3f(-10000,-10000,-10000);
		Vector3f worldAabbMax = new Vector3f(10000,10000,10000);
		//broad phase method
		AxisSweep3 overlappingPairCache = new AxisSweep3(worldAabbMin, worldAabbMax);
		//algorithm that encapsulates the physics in collisions.
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();

		//creating the dynamicWorld
		dynamicWorld = new DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
		dynamicWorld.setGravity(new Vector3f(0,-5,0));
		dynamicWorld.getDispatchInfo().allowedCcdPenetration = 0f;
		
	    jframe.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    jframe.setVisible(true); // show window
	}
}
