#include "cVAOManager.h"

#include "../GLCommon.h"	// All the GLFW, etc. (OpenGL stuff)

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <vector>

#include <sstream>

#include <fstream>

sModelDrawInfo::sModelDrawInfo()
{
	this->VAO_ID = 0;

	this->VertexBufferID = 0;
	this->VertexBuffer_Start_Index = 0;
	this->numberOfVertices = 0;

	this->IndexBufferID = 0;
	this->IndexBuffer_Start_Index = 0;
	this->numberOfIndices = 0;
	this->numberOfTriangles = 0;

	// The "local" (i.e. "CPU side" temporary array)
	this->pVertices = 0;	// or NULL
	this->pIndices = 0;		// or NULL

	// You could store the max and min values of the 
	//  vertices here (determined when you load them):
//	glm::vec3 maxValues;
//	glm::vec3 minValues;

//	scale = 5.0/maxExtent;		-> 5x5x5
//	float maxExtent;

	return;
}

// Takes:
// * 
// Returns: true if the file is loaded
bool readPlyFile_XYZ(sModelDrawInfo& modelDrawInfo)
{

	// ************************************************************
		// Read the top of the file to get some info.

		// Read all the text until I get to the word "vertex"
//    std::ifstream plyFile("assets/models/bun_zipper_res3.ply");
	std::ifstream plyFile(modelDrawInfo.meshName);     // May also see .c_str() "c style string, char*"

	if ( ! plyFile.is_open()) 
	{
		return false;
	}

	std::string token = "";

	if (!plyFile.is_open())
	{
		return false;
	}

	//element vertex 1889
	while (token != "vertex")
	{
		plyFile >> token;
	};
	// Next info is the number of vertices
	//int numberOfVertices = 0;
	plyFile >> modelDrawInfo.numberOfVertices;


	//element face 3851
	while (token != "face")
	{
		plyFile >> token;
	};
	// Next info is the number of vertices
	//int numberOfTriangles = 0;
	plyFile >> modelDrawInfo.numberOfTriangles;

	//end_header
	// -0.0369122 0.127512 0.00276757 0.850855 0.5 
	while (token != "end_header")
	{
		plyFile >> token;
	};


	// 
//    std::cout << numberOfVertices << std::endl;
//    std::cout << numberOfTriangles << std::endl;

	//property float x
	//property float y
	//property float z

	//struct sPlyVertex
	//{
	//    float x, y, z, confidence, intensity;
	//};
	////
	//// and 
	//// 
	//// 3 572 584 1040 
	//struct sTriangle
	//{
	//    unsigned int vertIndex_0;
	//    unsigned int vertIndex_1;
	//    unsigned int vertIndex_2;
	//};

	// Load the data from the file
//    sPlyVertex* pPlyVertices = new sPlyVertex[numberOfVertices];
	modelDrawInfo.pVertices = new sVertex[modelDrawInfo.numberOfVertices];

	// end_header
	// -0.0369122 0.127512 0.00276757 0.850855 0.5
	for (unsigned index = 0; index != modelDrawInfo.numberOfVertices; index++)
	{
		plyFile >> modelDrawInfo.pVertices[index].x;
		plyFile >> modelDrawInfo.pVertices[index].y;
		plyFile >> modelDrawInfo.pVertices[index].z;

		// Only has xyz, so stop here
//        plyFile >> allFileInfo.pPlyVertices[index].confidence;
//        plyFile >> allFileInfo.pPlyVertices[index].intensity;
//		modelDrawInfo.pVertices[index].confidence = 0;
//		modelDrawInfo.pVertices[index].intensity = 0;
	}

	// Load the triangle info from the file
	struct sPlyFileTriangle
	{
		unsigned int vertIndex_0;
		unsigned int vertIndex_1;
		unsigned int vertIndex_2;
	};
//    sTriangle* pPlyTriangles = new sTriangle[numberOfTriangles];
	sPlyFileTriangle* pPlyFileTriangles = new sPlyFileTriangle[modelDrawInfo.numberOfTriangles];
	for (unsigned int index = 0; index != modelDrawInfo.numberOfTriangles; index++)
	{
		// 3 737 103 17 
		int discard = 0;
		plyFile >> discard;
		plyFile >> pPlyFileTriangles[index].vertIndex_0;
		plyFile >> pPlyFileTriangles[index].vertIndex_1;
		plyFile >> pPlyFileTriangles[index].vertIndex_2;
	}

	// Copy the triangle data to a 1D array...
	modelDrawInfo.numberOfIndices = modelDrawInfo.numberOfTriangles * 3;

	modelDrawInfo.pIndices = new unsigned int[modelDrawInfo.numberOfIndices];

	unsigned int index = 0;
	for (unsigned int triIndex = 0; triIndex != modelDrawInfo.numberOfTriangles; triIndex++)
	{
		modelDrawInfo.pIndices[index + 0] = pPlyFileTriangles[triIndex].vertIndex_0;
		modelDrawInfo.pIndices[index + 1] = pPlyFileTriangles[triIndex].vertIndex_1;
		modelDrawInfo.pIndices[index + 2] = pPlyFileTriangles[triIndex].vertIndex_1;
	}


	return true;
}

bool cVAOManager::LoadModelIntoVAO(
		std::string fileName, 
		sModelDrawInfo &drawInfo,
	    unsigned int shaderProgramID)

{
	// Load the model from file
	// (We do this here, since if we can't load it, there's 
	//	no point in doing anything else, right?)

	drawInfo.meshName = fileName;

	// TODO: Load the model from file
	if (!readPlyFile_XYZ(drawInfo))
	{
		return false;
	}


	// 
	// Model is loaded and the vertices and indices are in the drawInfo struct
	// 

	// Create a VAO (Vertex Array Object), which will 
	//	keep track of all the 'state' needed to draw 
	//	from this buffer...

	// Ask OpenGL for a new buffer ID...
	glGenVertexArrays( 1, &(drawInfo.VAO_ID) );
	// "Bind" this buffer:
	// - aka "make this the 'current' VAO buffer
	glBindVertexArray(drawInfo.VAO_ID);

	// Now ANY state that is related to vertex or index buffer
	//	and vertex attribute layout, is stored in the 'state' 
	//	of the VAO... 


	// NOTE: OpenGL error checks have been omitted for brevity
//	glGenBuffers(1, &vertex_buffer);
	glGenBuffers(1, &(drawInfo.VertexBufferID) );

//	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, drawInfo.VertexBufferID);
	// sVert vertices[3]
	glBufferData( GL_ARRAY_BUFFER, 
				  sizeof(sVert) * drawInfo.numberOfVertices,	// ::g_NumberOfVertsToDraw,	// sizeof(vertices), 
				  (GLvoid*) drawInfo.pVertices,							// pVertices,			//vertices, 
				  GL_STATIC_DRAW );


	// Copy the index buffer into the video card, too
	// Create an index buffer.
	glGenBuffers( 1, &(drawInfo.IndexBufferID) );

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, drawInfo.IndexBufferID);

	glBufferData( GL_ELEMENT_ARRAY_BUFFER,			// Type: Index element array
	              sizeof( unsigned int ) * drawInfo.numberOfIndices, 
	              (GLvoid*) drawInfo.pIndices,
                  GL_STATIC_DRAW );

	// Set the vertex attributes.

	GLint vpos_location = glGetAttribLocation(shaderProgramID, "vPos");	// program
	GLint vcol_location = glGetAttribLocation(shaderProgramID, "vCol");	// program;

	// Set the vertex attributes for this shader
	glEnableVertexAttribArray(vpos_location);	// vPos
	glVertexAttribPointer( vpos_location, 3,		// vPos
						   GL_FLOAT, GL_FALSE,
						   sizeof(float) * 6, 
						   ( void* )0);

	glEnableVertexAttribArray(vcol_location);	// vCol
	glVertexAttribPointer( vcol_location, 3,		// vCol
						   GL_FLOAT, GL_FALSE,
						   sizeof(float) * 6, 
						   ( void* )( sizeof(float) * 3 ));

	// Now that all the parts are set up, set the VAO to zero
	glBindVertexArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glDisableVertexAttribArray(vpos_location);
	glDisableVertexAttribArray(vcol_location);


	// Store the draw information into the map
	this->m_map_ModelName_to_VAOID[ drawInfo.meshName ] = drawInfo;


	return true;
}


// We don't want to return an int, likely
bool cVAOManager::FindDrawInfoByModelName(
		std::string filename,
		sModelDrawInfo &drawInfo) 
{
	std::map< std::string /*model name*/,
			sModelDrawInfo /* info needed to draw*/ >::iterator 
		itDrawInfo = this->m_map_ModelName_to_VAOID.find( filename );

	// Find it? 
	if ( itDrawInfo == this->m_map_ModelName_to_VAOID.end() )
	{
		// Nope
		return false;
	}

	// Else we found the thing to draw
	// ...so 'return' that information
	drawInfo = itDrawInfo->second;
	return true;
}

