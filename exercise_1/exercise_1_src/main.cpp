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
    unsigned int v1_idx;
    unsigned int v2_idx;
    unsigned int v3_idx;

    Face(unsigned int v1_idx, unsigned int v2_idx, unsigned int v3_idx)
    {
        this->v1_idx = v1_idx;
        this->v2_idx = v2_idx;
        this->v3_idx = v3_idx;
    }
};


bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename) {
    //std::cout << "width: " << width << ", height: " << height << std::endl;
    float edgeThreshold = 0.01f; // 1cm

    // TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
    // - have a look at the "off_sample.off" file to see how to store the vertices and triangles
    // - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
    // - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
    // - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
    // - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
    // - only write triangles with valid vertices and an edge length smaller then edgeThreshold

    // TODO: Get number of vertices
    unsigned int nVertices = width * height;


    // TODO: Determine number of valid faces
    unsigned nFaces = 0; //(width - 1) * (height - 1) * 2;
    std::vector<Face*> faces;
    for (unsigned int u = 0; u < width-1; u++)
    {
        for (unsigned int v = 0; v < height-1; v++)
        {
            unsigned int idx = v * width + u;
            if (vertices[idx].position(0) != MINF && vertices[idx].position(1) != MINF &&
                vertices[idx].position(2) != MINF && vertices[idx].position(3) != MINF)
            {
                // triangularization of 4 pixels as 2 triangles:
                /*
                 * shape:
                    v1 -- v3
                    |    / |
                    |   /  |
                    |  /   |
                    | /    |
                    v2 -- v4
                    with right hand rule, indices for 2 triangles becomes:
                    v1 v2 v3 and v3 v2 v4
                 */
                unsigned int v1_idx = v * width + u;
                unsigned int v2_idx = (v+1) * width + u;
                unsigned int v3_idx = v * width + u + 1;
                unsigned int v4_idx = (v+1) * width + u+1;
                Vertex v1 = vertices[v1_idx];
                Vertex v2 = vertices[v2_idx];
                Vertex v3 = vertices[v3_idx];
                Vertex v4 = vertices[v4_idx];


                // v1 v2 v3 triangle
                if ((v1.position - v2.position).norm() < edgeThreshold &&
                    (v2.position - v3.position).norm() < edgeThreshold &&
                    (v1.position - v3.position).norm() < edgeThreshold)
                {
                    Face* face = new Face(v1_idx, v2_idx, v3_idx);
                    faces.push_back(face);
                    nFaces++;

                }

                // v3 v2 v4 triangle
                if ((v3.position - v2.position).norm() < edgeThreshold &&
                    (v2.position - v4.position).norm() < edgeThreshold &&
                    (v3.position - v4.position).norm() < edgeThreshold)
                {
                    Face* face = new Face(v3_idx, v2_idx, v4_idx);
                    faces.push_back(face);
                    nFaces++;
                }


            }
        }
    }



    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    //outFile << "# numVertices numFaces numEdges" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    // TODO: save vertices
    //outFile << "# list of vertices" << std::endl;
    //outFile << "# X Y Z R G B A" << std::endl;
    for (int u = 0; u < width; u++) {
        for (int v = 0; v < height; v++) {
            int idx = v * width + u;
            if (vertices[idx].position(0) != MINF && vertices[idx].position(1) != MINF &&
                vertices[idx].position(2) != MINF && vertices[idx].position(3) != MINF) {
                outFile << vertices[idx].position(0) << " ";
                outFile << vertices[idx].position(1) << " ";
                outFile << vertices[idx].position(2) << " ";
                outFile << (int) vertices[idx].color(0) << " ";
                outFile << (int) vertices[idx].color(1) << " ";
                outFile << (int) vertices[idx].color(2) << " ";
                outFile << (int) vertices[idx].color(3) << std::endl;
            } else {
                outFile << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
            }
        }
    }

    // TODO: save valid faces
    //outFile << "# list of faces" << std::endl;
    //outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
    /*for (int u = 0; u < width-1; u++)
    {
        for (int v = 0; v < height-1; v++)
        {*/
            // triangularization of 4 pixels as 2 triangles:
            /*
             * shape:
                v1 -- v3
                |    / |
                |   /  |
                |  /   |
                | /    |
                v2 -- v4
                with right hand rule, indices for 2 triangles becomes:
                v1 v2 v3 and v3 v2 v4
             */
         /*   int v1 = v * width + u;
            int v2 = (v+1) * width + u;
            int v3 = v * width + u + 1;
            int v4 = (v+1) * width + u+1;

            //if (isValidTriangle())




        }
    }*/

         for (Face* face : faces)
         {
             outFile << "3 " << face->v1_idx << " ";
             outFile << face->v2_idx << " ";
             outFile << face->v3_idx << std::endl;
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
		int depthImageWidth =sensor.GetDepthImageWidth();
		int depthImageHeight = sensor.GetDepthImageHeight();

		Matrix4f depthIntrinsicsInv = MatrixXf::Identity(4, 4);
		depthIntrinsicsInv(0, 0) = 1/fX;
	    depthIntrinsicsInv(1, 1) = 1/fY;
	    depthIntrinsicsInv(0, 2) = -cX/fX;
	    depthIntrinsicsInv(1, 2) = -cY/fY;
		for (int u = 0; u < depthImageWidth; u++)
        {
		    for (int v = 0; v < depthImageHeight; v++)
            {
                int idx =v*depthImageWidth + u;
		        if (depthMap[v*depthImageWidth + u] != MINF)
                {

		            Vector4f pixelCoord(u* depthMap[idx], v*depthMap[idx], depthMap[idx], 1);
		            vertices[idx].position = trajectoryInv * depthExtrinsicsInv * depthIntrinsicsInv * pixelCoord;
		            vertices[idx].color(0) = (unsigned char) colorMap[4*idx];
                    vertices[idx].color(1) = (unsigned char) colorMap[4*idx+1];
                    vertices[idx].color(2) = (unsigned char) colorMap[4*idx+2];
                    vertices[idx].color(3) = (unsigned char) colorMap[4*idx+3];
                }
		        else
                {
                    vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                    vertices[idx].color = Vector4uc(0,0,0,0);
                }
		        //std::cout << "position (" << u << ", " << v << ")" << vertices[idx].position << std::endl;
		        //std::cout << "color (" << u << ", " << v << ")" << vertices[idx].color << std::endl;
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
