#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"


struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};

struct Face
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vertex v1;
    int v1_id;
    Vertex v2;
    int v2_id;
    Vertex v3;
    int v3_id;

    Face(Vertex v1, int v1_id, Vertex v2, int v2_id, Vertex v3, int v3_id){
        this->v1 = v1;
        this->v2 = v2;
        this->v3 = v3;
        this->v1_id = v1_id;
        this->v2_id = v2_id;
        this->v3_id = v3_id;
    }

};

bool isValidVertex(Vertex v){
    // Check if the vertex position is valid
    if(v.position[0] == MINF || v.position[1] == MINF || v.position[2] == MINF || v.position[3] == MINF){
        return false;
    }
    return true;
}

bool isValidTriangle(Vector4f v1, Vector4f v2, Vector4f v3, float edge_threshold){
    // Check if the edges formed by using the provided vertices have an edge length
    // strictly smaller than the edge threshold by using L2 norm

    // Check if this norm can be used for MINF
    if (((v1-v2).norm() < edge_threshold) && ((v1-v3).norm() < edge_threshold) && ((v2-v3).norm() < edge_threshold)){
        return true;
    }
    return false;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
    //	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
    //	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	// Use image size
	unsigned int nVertices = width * height;


	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

	std::vector<Face*> faces;

    for (int i = 0; i < height-1; i++) {
        for (int j = 0; j < width-1; j++){
            // Get a square (two triangles can be formed from the square, at most)
            // Calculate the vertex indices so that they are always neighboring
            // and correspond to the corners of a square

            int v1_id = width * i + j;
            Vertex v1 = vertices[v1_id];
            int v2_id = width * i + (j+1);
            Vertex v2 = vertices[v2_id];
            int v3_id = width * (i+1) + j;
            Vertex v3 = vertices[v3_id];
            int v4_id = width * (i+1) + (j+1);
            Vertex v4 = vertices[v4_id];

            // Upper triangle
            if (isValidTriangle(v1.position, v2.position, v3.position, edgeThreshold)){
                nFaces++;
                Face* face = new Face(v1, v1_id, v2, v2_id, v3, v3_id);
                faces.push_back(face);
            }
            // Lower triangle
            if (isValidTriangle(v2.position, v3.position, v4.position, edgeThreshold)){
                nFaces++;
                Face* face = new Face(v2, v2_id, v3, v3_id, v4, v4_id);
                faces.push_back(face);
            }
        }
    }


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;


	// TODO: save vertices
    for (int i = 0; i < nVertices; i++) {
        if(isValidVertex(vertices[i])){
            outFile << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2] << " "
                    << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3]
                    << std::endl;
        }
        else{

            outFile << "0.0 0.0 0.0 "
                    << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3]
                    << std::endl;
        }
    }

	// TODO: save valid faces
    for(const auto face : faces){
        outFile << "3 " << face->v1_id << " " << face->v2_id << " " << face->v3_id << std::endl;
    }

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();

		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

        // Back-project the image pixels to the world space
        for (int h = 0; h < sensor.GetDepthImageHeight(); h++) {
            for (int w = 0; w < sensor.GetDepthImageWidth(); w++) {
                int index = sensor.GetDepthImageWidth() * h + w;


                if (depthMap[index] != MINF){
                    // Depth information exists for the vertex, apply-back projection
                    float depth = depthMap[index];

                    // Inverse projection: (depth*x, depth*y, depth)
                    float x = depth * w;
                    float y = depth * h;

                    // Scale the screen space coordinates (u, v) by depth
                    Vector4f pinhole_coord = Vector4f(x, y, depth, 1);

                    Matrix4f depthIntrinsicsInv = MatrixXf::Identity(4, 4);
                    depthIntrinsicsInv(0, 0) = 1 / fX;
                    depthIntrinsicsInv(1, 1) = 1 / fY;
                    depthIntrinsicsInv(0, 2) = -cX / fX;
                    depthIntrinsicsInv(1, 2) = -cY / fY;
                    
                    Vector4f real_coord = trajectoryInv * depthExtrinsicsInv * depthIntrinsicsInv * pinhole_coord;
                    real_coord[3] = 1.0;

                    vertices[index].position = real_coord;
                    vertices[index].color[0] = colorMap[4 * index];
                    vertices[index].color[1] = colorMap[4 * index + 1];
                    vertices[index].color[2] = colorMap[4 * index + 2];
                    vertices[index].color[3] = colorMap[4 * index + 3];
                }
                else{
                    // Depth value is invalid, set the vertex position and color to default values
                    vertices[index].color    = Vector4uc(0,0,0,0);
                    vertices[index].position = Vector4f(MINF, MINF, MINF, MINF);

                }

            }

        }


		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
