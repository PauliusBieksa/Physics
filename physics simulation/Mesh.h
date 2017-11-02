#pragma once
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>
#include "OBJloader.h"
#include "Shader.h"

/*
** VERTEX CLASS
*/
class Vertex
{
public:

	Vertex()
	{
		m_coord = glm::vec3();
	}
	Vertex(const glm::vec3& coord)
	{
		this->m_coord = coord;
	}
	glm::vec3 getCoord() const { return m_coord; }

protected:
private:
	glm::vec3 m_coord;
};

enum MeshBufferPositions
{
	POSITION_VB,
	TEXCOORD_VB,
	NORMAL_VB,
	INDEX_VB
};

/*
** MESH CLASS
*/
class Mesh
{
public:

	enum MeshType
	{
		TRIANGLE,
		QUAD,
		CUBE
	};
	/*
	** CONSTRUCTORS
	*/

	// default constructor creates a horizontal plane or dimensions 1 x 1 centered on the origin
	Mesh();
	Mesh::Mesh(MeshType type)
	{
		Vertex vertices[36];

		switch (type)
		{
		case TRIANGLE:
			// Create triangle
			vertices[0] = Vertex(glm::vec3(-1.0, -1.0, 0.0));
			vertices[1] = Vertex(glm::vec3(0, 1.0, 0.0));
			vertices[2] = Vertex(glm::vec3(1.0, -1.0, 0.0));
			break;

		case QUAD:
			// create quad
			vertices[0] = Vertex(glm::vec3(-0.5f, 0.0f, -0.5f));
			vertices[1] = Vertex(glm::vec3(0.5f, 0.0f, -0.5f));
			vertices[2] = Vertex(glm::vec3(-0.5f, 0.0f, 0.5f));
			vertices[3] = Vertex(glm::vec3(0.5f, 0.0f, -0.5f));
			vertices[4] = Vertex(glm::vec3(-0.5f, 0.0f, 0.5f));
			vertices[5] = Vertex(glm::vec3(0.5f, 0.0f, 0.5f));
			break;

		case CUBE:
			// create cube
			/*vertices[0] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[1] = Vertex(glm::vec3(1.0f, -1.0f, -1.0f));
			vertices[2] = Vertex(glm::vec3(1.0f, 1.0f, -1.0f));
			vertices[3] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[4] = Vertex(glm::vec3(1.0f, 1.0f, -1.0f));
			vertices[5] = Vertex(glm::vec3(-1.0f, 1.0f, -1.0f));
			vertices[6] = Vertex(glm::vec3(-1.0f, -1.0f, 1.0f));
			vertices[7] = Vertex(glm::vec3(1.0f, -1.0f, 1.0f));
			vertices[8] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[9] = Vertex(glm::vec3(-1.0f, -1.0f, 1.0f));
			vertices[10] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[11] = Vertex(glm::vec3(-1.0f, 1.0f, 1.0f));
			vertices[12] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[13] = Vertex(glm::vec3(1.0f, -1.0f, -1.0f));
			vertices[14] = Vertex(glm::vec3(1.0f, -1.0f, 1.0f));
			vertices[15] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[16] = Vertex(glm::vec3(1.0f, -1.0f, 1.0f));
			vertices[17] = Vertex(glm::vec3(-1.0f, -1.0f, 1.0f));
			vertices[18] = Vertex(glm::vec3(-1.0f, 1.0f, -1.0f));
			vertices[19] = Vertex(glm::vec3(1.0f, 1.0f, -1.0f));
			vertices[20] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[21] = Vertex(glm::vec3(-1.0f, 1.0f, -1.0f));
			vertices[22] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[23] = Vertex(glm::vec3(-1.0f, 1.0f, 1.0f));
			vertices[24] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[25] = Vertex(glm::vec3(-1.0f, 1.0f, -1.0f));
			vertices[26] = Vertex(glm::vec3(-1.0f, 1.0f, 1.0f));
			vertices[27] = Vertex(glm::vec3(-1.0f, -1.0f, -1.0f));
			vertices[28] = Vertex(glm::vec3(-1.0f, 1.0f, 1.0f));
			vertices[29] = Vertex(glm::vec3(-1.0f, -1.0f, 1.0f));
			vertices[30] = Vertex(glm::vec3(1.0f, -1.0f, -1.0f));
			vertices[31] = Vertex(glm::vec3(1.0f, 1.0f, -1.0f));
			vertices[32] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[33] = Vertex(glm::vec3(1.0f, -1.0f, -1.0f));
			vertices[34] = Vertex(glm::vec3(1.0f, 1.0f, 1.0f));
			vertices[35] = Vertex(glm::vec3(1.0f, -1.0f, 1.0f));*/
			vertices[0] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[1] = Vertex(glm::vec3(0.5f, -0.5f, -0.5f));
			vertices[2] = Vertex(glm::vec3(0.5f, 0.5f, -0.5f));
			vertices[3] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[4] = Vertex(glm::vec3(0.5f, 0.5f, -0.5f));
			vertices[5] = Vertex(glm::vec3(-0.5f, 0.5f, -0.5f));
			vertices[6] = Vertex(glm::vec3(-0.5f, -0.5f, 0.5f));
			vertices[7] = Vertex(glm::vec3(0.5f, -0.5f, 0.5f));
			vertices[8] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[9] = Vertex(glm::vec3(-0.5f, -0.5f, 0.5f));
			vertices[10] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[11] = Vertex(glm::vec3(-0.5f, 0.5f, 0.5f));
			vertices[12] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[13] = Vertex(glm::vec3(0.5f, -0.5f, -0.5f));
			vertices[14] = Vertex(glm::vec3(0.5f, -0.5f, 0.5f));
			vertices[15] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[16] = Vertex(glm::vec3(0.5f, -0.5f, 0.5f));
			vertices[17] = Vertex(glm::vec3(-0.5f, -0.5f, 0.5f));
			vertices[18] = Vertex(glm::vec3(-0.5f, 0.5f, -0.5f));
			vertices[19] = Vertex(glm::vec3(0.5f, 0.5f, -0.5f));
			vertices[20] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[21] = Vertex(glm::vec3(-0.5f, 0.5f, -0.5f));
			vertices[22] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[23] = Vertex(glm::vec3(-0.5f, 0.5f, 0.5f));
			vertices[24] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[25] = Vertex(glm::vec3(-0.5f, 0.5f, -0.5f));
			vertices[26] = Vertex(glm::vec3(-0.5f, 0.5f, 0.5f));
			vertices[27] = Vertex(glm::vec3(-0.5f, -0.5f, -0.5f));
			vertices[28] = Vertex(glm::vec3(-0.5f, 0.5f, 0.5f));
			vertices[29] = Vertex(glm::vec3(-0.5f, -0.5f, 0.5f));
			vertices[30] = Vertex(glm::vec3(0.5f, -0.5f, -0.5f));
			vertices[31] = Vertex(glm::vec3(0.5f, 0.5f, -0.5f));
			vertices[32] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[33] = Vertex(glm::vec3(0.5f, -0.5f, -0.5f));
			vertices[34] = Vertex(glm::vec3(0.5f, 0.5f, 0.5f));
			vertices[35] = Vertex(glm::vec3(0.5f, -0.5f, 0.5f));
			break;
		}

		// generate unique vertex vector (no duplicates )
	//	m_vertices = std::vector<Vertex>(std::begin(vertices), std::end(vertices));
		std::vector<Vertex> allverts = std::vector<Vertex>(std::begin(vertices), std::end(vertices));
		/*
		* COMPLETE THIS PART
		*/
		m_vertices = std::vector<Vertex>();
		bool duplicate = false;
		for (Vertex v : allverts)
		{
			for (int i = 0; i < m_vertices.size(); i++)
			{
				if (m_vertices[i].getCoord() == v.getCoord())
				{
					duplicate = true;
					break;
				}
			}
			if (duplicate)
			{
				duplicate = false;
				continue;
			}
			m_vertices.push_back(v);
		}

		// create mesh
		initMesh(vertices, sizeof(vertices) / sizeof(vertices[0]));

		// create model matrix ( identity )
		initTransform();

	}
	// create mesh from a .obj file
	Mesh(const std::string& fileName);
	virtual ~Mesh();


	/*
	** GET AND SET METHODS
	*/

	// getModel computes the model matrix any time it is required
	std::vector<Vertex> getVertices() { return m_vertices; }
	glm::vec3 getPos() const { return getTranslate()[3]; }
	glm::mat4 getModel() const { return getTranslate() * getRotate() * getScale(); }
	glm::mat4 getTranslate() const { return m_translate; }
	glm::mat4 getRotate() const { return m_rotate; }
	glm::mat4 getScale() const { return m_scale; }
	Shader getShader() const { return m_shader; }
	GLuint getVertexArrayObject() const { return m_vertexArrayObject; }
	unsigned int getNumIndices() const { return m_numIndices; }

	// set position of mesh center to specified 3D position vector
	void setPos(const glm::vec3 &position)
	{
		m_translate[3][0] = position[0];
		m_translate[3][1] = position[1];
		m_translate[3][2] = position[2];
	}
	// set i_th coordinate of mesh center to float p (x: i=0, y: i=1, z: i=2)
	void setPos(int i, float p) { m_translate[3][i] = p; }

	// allocate shader to mesh
	void setShader(const Shader &shader) {
		m_shader = shader;
		m_shader.Use();
	}


	/*
	** INITIALISATION AND UTILITY METHODS
	*/

	// initialise transform matrices to identity
	void initTransform();
	// create mesh from vertices
	void initMesh(Vertex* vertices, unsigned int numVertices);
	// create mesh from model (typically loaded from a file)
	void InitMesh(const IndexedModel& model);
	// load .obj file
	void loadOBJ(const char * path,
		std::vector < glm::vec3 > & out_vertices,
		std::vector < glm::vec2 > & out_uvs,
		std::vector < glm::vec3 > & out_normals
	);


	/*
	** TRANSFORMATION METHODS
	*/

	// translate mesh by a vector
	void translate(const glm::vec3 &vect);
	// rotate mesh by a vector
	void rotate(const float &angle, const glm::vec3 &vect);
	// scale mesh by a vector
	void scale(const glm::vec3 &vect);

	void setRotate(const glm::mat4 R) { m_rotate = R; }


private:
	enum {
		POSITION_VB,
		NUM_BUFFERS
	};

	std::vector<Vertex> m_vertices;

	GLuint m_vertexArrayObject;
	GLuint m_vertexArrayBuffers[NUM_BUFFERS];
	unsigned int m_numIndices;
	glm::mat4 m_translate;
	glm::mat4 m_rotate;
	glm::mat4 m_scale;

	Shader m_shader;
};


