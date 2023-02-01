#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <GL/freeglut.h>
#include <fstream>
using namespace std;

#define DT 0.005f

dWorldID world;
dSpaceID space;
dGeomID geom;
dJointID joint;
dsFunctions window;
ofstream outfile;

dBodyID bolabody;
dMass bolamassa;
dReal boladiameter = 0.5f;

dVector3 vec, vec2;
dReal posx, posy, posz;
dReal posisi[] = { 0.f, 0.f, 0.f };

dReal theta = 0.5*(M_PI / 180.f);
dReal Phi = 0.0001f;
dReal phi = 0.5*(M_PI / 180.f);
dReal omega = 1.f;
dReal x1dot, z1dot, x2dot, z2dot, xtot, ztot;
dReal x, y, z, l = 2.5f;

void MenghitungXtotdanZtot()
{
	x1dot = Phi*cosf(phi);
	z1dot = Phi*sinf(phi);
	x2dot = omega*cosf(theta)*cosf(phi);
	z2dot = omega*cosf(theta)*sinf(phi);
	xtot = x1dot + x2dot;
	ztot = z1dot + z2dot;
}

void Kamera()
{
	static dReal xyz[] = { 0.f, -2.5f, 3.1f };
	static dReal hpr[] = { 90.f, 0.f, 0.f };
	dsSetViewpoint(xyz, hpr);
}

void Bandul()
{
	outfile.open("Hasil 1 Simpangan.txt");
	outfile << "Sumbu X" << "			" << "Sumbu Z" << endl;

	MenghitungXtotdanZtot();

	bolabody = dBodyCreate(world);

	dBodySetPosition(bolabody, posx, posy, posz + 2.f);

	geom = dCreateSphere(space, 0.1f);
	dGeomSetBody(geom, bolabody);
	dMassSetSphere(&bolamassa, 1.f, boladiameter);
	dBodySetMass(bolabody, &bolamassa);

	joint = dJointCreateDBall(world, 0);
	dJointAttach(joint, bolabody, 0);
	dJointSetDBallAnchor1(joint, 0.f, 0.f, 2.f);
	dJointSetDBallAnchor2(joint, 0.f, 0.f, 4.5f);
}

void Mulai()
{
	Bandul();

	Kamera();
}

void Berhenti()
{
	dGeomDestroy(geom);
	dSpaceDestroy(space);
	dWorldDestroy(world);
}

void Input(int key)
{
	switch (key)
	{
	case 'd':
		dBodyAddForceAtRelPos(bolabody, ztot, xtot, 0.f, 0.f, 0.f, 0.f);
		dBodySetPosition(bolabody, 0.5, 0.f, 2.f);
		break;
	}
}

void Gambar(dGeomID g)
{
	int gclass = dGeomGetClass(g);

	switch (gclass)
	{
	case dSphereClass:
		dsSetColor(0.f, 1.f, 0.f);
		dsSetTexture(DS_WOOD);
		dsSetSphereQuality(2);
		dsDrawSphere(dGeomGetPosition(g), dGeomGetRotation(g), dGeomSphereGetRadius(g));
		break;
	}
}

void Print(int x, int y, int z, char* string)
{
	glRasterPos2f(x, y);
	int len = (int)strlen(string);

	for (int i = 0; i < len; i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, string[i]);
	}
}

void Loop(int pause)
{
	if (!pause)
	{
		static dReal t = 0.f;
		const dReal *pos;

		const dReal step = 0.0005f;
		const unsigned steps = 4;

		for (unsigned i = 0; i < steps; ++i)
		{
			Print(1, 2.5, 3.1, "Bandul 3D");
			pos = dBodyGetPosition(bolabody);
			outfile << pos[1] << "			" << pos[0] << endl;

			t += step;

			dWorldQuickStep(world, step);
		}
	}

	unsigned ngeom = dSpaceGetNumGeoms(space);
	for (unsigned i = 0; i < ngeom; ++i)
	{
		dGeomID g = dSpaceGetGeom(space, i);
		Gambar(g);
	}

	dJointGetDBallAnchor1(joint, vec);
	dJointGetDBallAnchor2(joint, vec2);
	dsSetColor(1.f, 0.f, 0.f);
	dsDrawLine(vec, vec2);
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);

	window.version = DS_VERSION;
	window.start = &Mulai;
	window.step = &Loop;
	window.command = &Input;
	window.stop = Berhenti;
	window.path_to_textures = "res/textures";

	world = dWorldCreate();
	dWorldSetGravity(world, 0.f, 0.f, -9.8f);
	dWorldSetDamping(world, 0, 0); //gaya gesekan
	space = dHashSpaceCreate(0);

	dInitODE();

	dsSimulationLoop(argc, argv, 640, 480, &window);

	dCloseODE();
	outfile.close();

	return 0;
}