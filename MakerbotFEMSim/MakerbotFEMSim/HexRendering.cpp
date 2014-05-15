#include "HexRendering.h"

void HexRendering::drawHexMesh(ElementMesh * mesh)
{
	// draw z coords 
	float particleRadius = 0.075f / 50.0f;
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
	
		//glPushMatrix();
		//glDisable(GL_LIGHTING);
		//glColor3f(1.0f, 1.0f, 1.0f);

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
		
		//glPopMatrix();
		//glEnable(GL_LIGHTING);
	}
}

void HexRendering::drawHexMeshWithStress(ElementMesh * mesh, std::vector<Eigen::Matrix3f> &cauchyStressTensors)
{
	float minRed = std::numeric_limits<float>::infinity();
	float maxRed = -1;

	float minBlue = std::numeric_limits<float>::infinity();
	float maxBlue = -1;

	float minGreen = std::numeric_limits<float>::infinity();
	float maxGreen = -1;

	for (int ii = 0; ii < cauchyStressTensors.size(); ++ii)
	{
		Eigen::Matrix3f cauchyStress = cauchyStressTensors[ii];
		float r = cauchyStress.row(0).transpose().norm();
		float g = cauchyStress.row(1).transpose().norm();
		float b = cauchyStress.row(2).transpose().norm();

		minRed = min(r, minRed);
		maxRed = max(r, maxRed);

		minBlue = min(b, minBlue);
		maxBlue = max(b, maxBlue);

		minGreen = min(g, minGreen);
		maxGreen = max(g, maxGreen);
	}

	for (int elemI = 0; elemI < mesh->elements.size(); ++elemI)
	{
		Element * elem = mesh->elements[elemI];
		Eigen::Matrix3f cauchyStress = cauchyStressTensors[elemI];

		Eigen::Vector3f coord0 = mesh->coords[elem->vertices[0]];
		Eigen::Vector3f coord1 = mesh->coords[elem->vertices[1]];
		Eigen::Vector3f coord2 = mesh->coords[elem->vertices[2]];
		Eigen::Vector3f coord3 = mesh->coords[elem->vertices[3]];
		Eigen::Vector3f coord4 = mesh->coords[elem->vertices[4]];
		Eigen::Vector3f coord5 = mesh->coords[elem->vertices[5]];
		Eigen::Vector3f coord6 = mesh->coords[elem->vertices[6]];
		Eigen::Vector3f coord7 = mesh->coords[elem->vertices[7]];

		float r1 = cauchyStress(0,1);
		float r2 = cauchyStress(0,2);
		float r3 = cauchyStress(0,3);

		float g1 = cauchyStress(1,0);
		float g2 = cauchyStress(1,1);
		float g3 = cauchyStress(1,2);

		float b1 = cauchyStress(2,0);
		float b2 = cauchyStress(2,1);
		float b3 = cauchyStress(2,2);


		float r = sqrt(r1*r1 + r2*r2 + r3*r3);
		float g = sqrt(b1*b1 + b2*b2 + b3*b3);
		float b = sqrt(g1*g1 + g2*g2 + g3*g3);

		
		float rFinal = (r - minRed) / (maxRed - minRed);
		float gFinal = (g - minGreen) / (maxGreen - minGreen);
		float bFinal = (b - minBlue) / (maxBlue - minBlue);

		GLfloat materialColor[] = {rFinal, gFinal, bFinal, 1.0f};
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);

		// left face
		drawQuad(coord2, coord0, coord1, coord3);
		//drawQuad(coord3, coord1, coord0, coord2);

		// right face
		drawQuad(coord4, coord6, coord7, coord5);
		//drawQuad(coord5, coord7, coord6, coord4);

		// front face 
		drawQuad(coord7, coord3, coord1, coord5);
		//drawQuad(coord5, coord1, coord3, coord7);

		// back face 
		drawQuad(coord0, coord2, coord6, coord4);
		//drawQuad(coord4, coord6, coord2, coord0);
		
		// top face 
		drawQuad(coord2, coord3, coord7, coord6);
		//drawQuad(coord6, coord7, coord3, coord2);
		
		// bottom face 
		drawQuad(coord4, coord5, coord1, coord0);
		//drawQuad(coord0, coord1, coord5, coord4);
	}
}

// helper functions 
void HexRendering::drawLine(Eigen::Vector3f x0, Eigen::Vector3f x1)
{
	glBegin(GL_LINES);
    glVertex3f(x0(0), x0(1), x0(2));
    glVertex3f(x1(0), x1(1), x1(2));
    glEnd();
}

void HexRendering::drawQuad(Eigen::Vector3f coord0, Eigen::Vector3f coord1, Eigen::Vector3f coord2, Eigen::Vector3f coord3)
{
	glBegin(GL_POLYGON);
	glVertex3f(coord0(0), coord0(1), coord0(2));
	glVertex3f(coord1(0), coord1(1), coord1(2));
	glVertex3f(coord2(0), coord2(1), coord2(2));
	glVertex3f(coord3(0), coord3(1), coord3(2));
	glEnd();


}



