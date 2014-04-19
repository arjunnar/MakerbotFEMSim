#include "HexRendering.h"


void HexRendering::drawHexMesh(ElementMesh * mesh)
{
	// draw z coords 
	float particleRadius = 0.075f / 2.0f;
	for (int ii = 0; ii < mesh->coords.size(); ++ii)
	{
		Eigen::Vector3f posParticle = mesh->coords[ii];
        glPushMatrix();
        glTranslatef(posParticle(0), posParticle(1), posParticle(2));
        glutSolidSphere(particleRadius, 10.0f, 10.0f);
		glPopMatrix();
	}

	// draw lines for cubes 
	for (int ii = 0; ii < mesh->elements.size(); ++ii)
	{
		Element * elem = mesh->elements[ii];

		// TODO: check for repeated edges
		Eigen::Vector3f coord0 = mesh->coords[elem->vertices[0]];
		Eigen::Vector3f coord1 = mesh->coords[elem->vertices[1]];
		Eigen::Vector3f coord2 = mesh->coords[elem->vertices[2]];
		Eigen::Vector3f coord3 = mesh->coords[elem->vertices[3]];
		Eigen::Vector3f coord4 = mesh->coords[elem->vertices[4]];
		Eigen::Vector3f coord5 = mesh->coords[elem->vertices[5]];
		Eigen::Vector3f coord6 = mesh->coords[elem->vertices[6]];
		Eigen::Vector3f coord7 = mesh->coords[elem->vertices[7]];
	
		// left face
		drawLine(coord0, coord1);
		drawLine(coord1, coord3);
		drawLine(coord3, coord2);
		drawLine(coord2, coord0);

		// right face 
		drawLine(coord7, coord6);
		drawLine(coord6, coord4);
		drawLine(coord5, coord4);
		drawLine(coord7, coord5);

		// top face connections 
		drawLine(coord3, coord7);
		drawLine(coord2, coord6);

		// bottom face connections
		drawLine(coord0, coord4);
		drawLine(coord1, coord5);
	}
}

// helper functions 
void HexRendering::drawLine(Eigen::Vector3f x0, Eigen::Vector3f x1)
{
	glBegin(GL_LINES);
    glVertex3d(x0(0), x0(1), x0(2));
    glVertex3d(x1(0), x1(1), x1(2));
    glEnd();
}



