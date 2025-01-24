#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <vector>
#include <vector>
#include <map>
#include <set>
// 
#include "sMesh.h"
#include "cVAOManager/cVAOManager.h"

class cPhysics
{
public:

	// Info for the physics movement, etc.
	struct sPhysInfo
	{
		sPhysInfo()
		{
			this->position = glm::vec3(0.0f);
			this->acceleration = glm::vec3(0.0f);
			this->velocity = glm::vec3(0.0f);
			this->bDoesntMove = false;
		}		
		glm::vec3 position;
		glm::vec3 velocity;
		glm::vec3 acceleration;
		// If true, then we skip the integration step
		bool bDoesntMove;	// i.e. doesn't move
		// TODO: 
//		float mass;
//		float inverseMass;		// Usually this is what we keep

		// The drawing mesh that's connected to this physics object
		sMesh* pAssociatedDrawingMeshInstance = NULL;

		// Maybe other things, too
//		float mass;
//		glm::vec3 orientation;
	};

	// Types of primatives
	struct sSphere 
	{
		sSphere() 
		{
			this->centre = glm::vec3(0.0f);
			this->radius = 0.0f;
			this->pPhysicInfo = new sPhysInfo();
		}
		~sSphere()
		{
			delete this->pPhysicInfo;
		}
		glm::vec3 centre;
		float radius;
		sPhysInfo* pPhysicInfo = NULL;
//		unsigned int uniqueID;
		//unsigned int matchingMeshID;
	};

	struct sAABB		// Axis Aligned Bounding Box
	{
		sAABB()
		{
			this->centreXYZ = glm::vec3(0.0f);
			this->minXYZ = glm::vec3(0.0f);
			this->maxXYZ = glm::vec3(0.0f);
			this->pPhysicInfo = new sPhysInfo();
		}
		~sAABB()
		{
			delete this->pPhysicInfo;
		}
		glm::vec3 centreXYZ;
		glm::vec3 minXYZ;
		glm::vec3 maxXYZ;

		// Calculate the other stuff
		// TODO:
		glm::vec3 getExtentsXYZ(void);	//	glm::vec3 extentsXYZ;	// height, width, depth

//		glm::vec3 getMinXYZ(void);
//		glm::vec3 getMaxYYZ(void);
//		glm::vec3 getHalfExtent(void);	// Distance from centre on that axis
		//
		sPhysInfo* pPhysicInfo = NULL;
	
		//		unsigned int uniqueID;
	};

	struct sLine
	{
		sLine()
		{
			this->startXYZ = glm::vec3(0.0f);
			this->endXYZ = glm::vec3(0.0f);
		}
		glm::vec3 startXYZ;
		glm::vec3 endXYZ;
		float getLength(void)
		{
			return glm::distance(this->endXYZ, this->startXYZ);
		}
	};

	struct sTriangle
	{
		glm::vec3 vertices[3];
		glm::vec3 normal;
		// Maybe other things?
		glm::vec3 intersectionPoint;
		// Side lengths
		float sideLengths[3];
		void calculateSideLengths(void)
		{
			sideLengths[0] = glm::distance(this->vertices[0], this->vertices[1]);
			sideLengths[1] = glm::distance(this->vertices[1], this->vertices[2]);
			sideLengths[2] = glm::distance(this->vertices[2], this->vertices[0]);
		}
	};
	
	struct sTriangleMesh
	{
		std::string meshModelName;
		std::string meshInstanceName;	// Friendly name?
		std::vector<sTriangle> vecTriangles;
	};

	
	// BROAD phase objects
	class cBroad_Cube		// Really an AABB
	{
	private:
		// Can't call default constructor
		cBroad_Cube() {
			this->m_minXYZ = glm::vec3(0.0f);
			this->m_maxXYZ = glm::vec3(0.0f);
			this->m_size = 0.0f;
			this->m_indexXYZ = 0;
		};
		unsigned long long m_indexXYZ;
		glm::vec3 m_minXYZ;
		glm::vec3 m_maxXYZ;
		float m_size;	// witdh, heitht, etc.
	public:
		// When creating, we need to be careful that 
		// our index matches these min and max
		cBroad_Cube(
			glm::vec3 minXYZ, glm::vec3 maxXYZ,
			float sizeOrWidth,
			unsigned long long indexXYZ);
		unsigned long long getGridIndexID(void);
		float getSize(void);
		glm::vec3 getMinXYZ(void);
		glm::vec3 getMaxXYZ(void);

		std::vector<sTriangle> vec_pTriangles;
	};

	// These thing together make up a broad phase accel structure
	// 
	// This is the ID of the AABB/Cube based on a location in space
	unsigned long long calcBP_GridIndex(float x, float y, float z, float sizeOrWidth);
	glm::vec3 calcBP_MinXYZ_FromID(unsigned long long BP_CubeID, float sizeOrWidth);
	//
	// This holds all the AABB/cubes that hold the triangle information
	std::map< unsigned long long /*index*/, cBroad_Cube* > map_BP_CubeGrid;
	//

	// Is this too big for the stack?
	// sBroad_Cube BP_CubeArray[1000][1000][1000];
	//std::vector< std::vector< std::vector< sBroad_Cube* > > > vec_3D_CubeArray;
//	void initBroadPhaseGrid(void);
	//
	// Using model space (i.e. model is at origin)
//	bool generateBroadPhaseGrid(std::string meshModelName, float AABBCubeSize_or_Width);
	// TODO: Model is NOT at the origin
	bool generateBroadPhaseGrid(std::string meshModelName, float AABBCubeSize_or_Width,
		glm::vec3 meshWorldPosition,
		glm::vec3 meshWorldOrientation,
		float uniformScale);

	// Other types soon, likely
	// struct sPlane
	// struct sCapsule
	// etc.

	// The objects that are handled by the integration step, etc.
	std::vector<sSphere*> vecSpheres;
	std::vector<sAABB*> vecAABBs;
	std::vector< sTriangleMesh*> vecMeshes;
	// This is anything else that is using the integration step
	// ...but DON'T have collision detection
	std::vector< sPhysInfo* > vecGeneralPhysicsObjects;

	// This is used to find a particular mesh that's connected to this phsyics object
	// Reutrns NULL if not found
	sPhysInfo* pFindAssociateMeshByFriendlyName(std::string friendlyName);


	// Looks up the mesh information from the VAO manager
	void setVAOManager(cVAOManager* pTheVAOManager);
private:
	cVAOManager* m_pVAOManager = NULL;
public:
	// Look up the mesh from the VAO manager
	// (if it's loaded)
	bool addTriangleMesh(std::string meshModelName);
	// What if the mesh has been placed somewhere else in the world? 
	// TODO: 
	bool addTriangleMesh(std::string meshModelName, 
		glm::vec3 meshWorldPosition,
		glm::vec3 meshWorldOrientation, 
		float uniformScale);
	bool addTriangleMesh(const std::vector<cVAOManager::sTriangle>& vecVAOTriangles, glm::vec3 meshWorldPosition, glm::vec3 meshWorldOrientation, float uniformScale);
	bool addTriangleMesh(std::string meshModelName, glm::mat4 modelMatrix);
	
	// Takes triangle information directly 
	bool addTriangleMesh(std::string meshModelName, sTriangleMesh& vecMeshes);

//	std::vector<sPhysicObject*> vecAllTheObjects;
//	std::vector< sPhysInfo* > vec_pPhysInfos;

	void ProjectionOnAxis(glm::vec3 axis, sTriangle& triangle, float& minProj, float& maxProj);

	bool CheckTriangleTriangleCollision(sTriangle& triA, sTriangle& triB);

	// Called every time we want a collision detection step
	void StepTick(double deltaTime);


////	bool DoesRayCollide(glm::vec3 rayStart, glm::vec3 rayEnd);
////	bool DoesRayCollide(glm::vec3 rayStart, glm::vec3 rayEnd, vec_pMeshesIHit);
//	// Store the "vec_pMeshesIHit" like the collision events
//	void DoesRayCollide(glm::vec3 rayStart, glm::vec3 rayEnd);
//	// If there's a intersection, it will be added to 
//	// std::vector<sMeshCollionEven> vec_pMeshesRayHit;


	// Could do this... like a callback or event 
	// This is 100% OK to do this
//	void SphereAABBCollided(sSphere* pSphere, sAABB* pAABB);

	// Here's a list of all the event that happened in the last step
	// 
	// These represent a collision "event" 
	struct sCollision_SphereAABB
	{
		double timeOfCollision;

		sSphere* pTheSphere;
		sAABB* pTheAABB;
		glm::vec3 collisionWorldPoint;
		float nearestDistance;
		glm::vec3 sphereVelocity;	glm::vec3 spherePosition;	
		// velocity will also give direction
		// glm::vec3 sphereDirection;

		glm::vec3 AABB_Velocity;	glm::vec3 AABB_Position;	
		// velocity will also give direction
		// glm::vec3 sphereDirection;
	};
	std::vector<sCollision_SphereAABB> vec_SphereAABB_Collisions;

	struct sCollision_SphereSphere
	{
		double timeOfCollision;

		sSphere* pSpheres[2];
		glm::vec3 collisionWorldPoint;
		glm::vec3 velocities[2];	
		glm::vec3 psition[2];	
		// velocity will also give direction
		// glm::vec3 sphereDirection;
	};
	std::vector<sCollision_SphereAABB> vec_SphereSphere_Collisions;

	struct sCollision_RayTriangleInMesh
	{
		double timeOfCollision;

		sLine theRay;
		std::string meshInstanceName;
		std::vector<sTriangle> vecTriangles;
		// Any other things you might want
	};
	std::vector<sCollision_RayTriangleInMesh> vec_RayTriangle_Collisions;


	// Returns immediately with the triangles we hit
	bool rayCast(glm::vec3 start, glm::vec3 end,
		std::vector<sCollision_RayTriangleInMesh>& vec_RayTriangle_Collisions, 
		bool bIgnoreBackFacingTriangles = true);

	bool rayCastSpecificTriangles(glm::vec3 start, glm::vec3 end,
		std::vector<sCollision_RayTriangleInMesh>& vec_RayTriangle_Collisions,
		std::vector<sTriangle> vecSpecificTriangles,
		bool bIgnoreBackFacingTriangles = true);
	// This one adds the collision to the vec_RayTriangle_Collisions
	void rayCast(glm::vec3 start, glm::vec3 end, bool bIgnoreBackFacingTriangles = true);



	// Check to see if they collided. 
	// We likely need other information, like where, when, etc.
	bool bSphereAABBCollision(sSphere* pSphere, sAABB* pAABB);
	bool bSphereSphereCollision(sSphere* pA, sSphere* pB);
	bool bAABB_ABBBCollision(sAABB* pA, sAABB* pB);
	// This is an infinite ray intersection test
	// The start and end define the line, but it extends forever
	bool bRay_TriangleCollision(sLine theLine, sTriangle theTri);
	// This is a line SEGMENT traingle test
	bool bLineSegment_TriangleCollision(sLine theLine, sTriangle theTri);
	// returns the barycentric coordinates(u, v, w)
	bool bLineSegment_TriangleCollision(sLine theLine, sTriangle theTri,
	                                    float& u, float& v, float& w, float& t);
	//bool bLineSegment_TriangleCollision(
	//	sLine theLine, sTriangle theTri,
	//	double& u, double& v, double& w, double& t);

	std::pair<float, float> projectTriangleOntoAxis(const sTriangle& tri, const glm::vec3& axis);

	bool bTriangleTriangleCollision(const sTriangle& triA, const sTriangle& triB);
	std::vector<glm::vec3> axes;

private:
	void m_CheckForCollisions(double deltaTime);

};

