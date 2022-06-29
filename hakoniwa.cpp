

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
//---------------------------------------------------------------------------
#include <chai3d\chai3d.h>
#include <chai3d\CODE.h>
#include <chai3d\GEL3D.h>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// �ݒ�
//---------------------------------------------------------------------------

// �E�B���h�E�T�C�Y
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// �E�N���b�N���j���[
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

#define _WSIZEX 1680
#define _WSIZEY 1050
#define _FPS 60

double depth =15.00000;
double dist =0.50;

//---------------------------------------------------------------------------
// �L��ϐ�
//---------------------------------------------------------------------------

//���[���h
cWorld* world;

// �J����
cCamera* camera;

cViewport* viewport;

// ���C�g
cLight *light;

// �J�����g�E�B���h�E�f�B�X�v���C��Wight Height
int displayW  = 0;
int displayH  = 0;

// �f�o�C�X�̃n���h��
cHapticDeviceHandler* handler;

//�f�o�C�X�̏��
cHapticDeviceInfo info;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

//���f���̃��b�V��
cMesh* tooth;

//���߃��x��
double transparencyLevel = 0.3;

cMesh* hand;

// a small line used to display a grasp
cShapeLine* graspLine;

// temp variable to store positions and orientations
// of tooth and drill
cVector3d lastPosObject;
cMatrix3d lastRotObject;
cVector3d lastPosDevice;
cMatrix3d lastRotDevice;
cVector3d lastDeviceObjectPos;
cMatrix3d lastDeviceObjectRot;


//�������Z���E----------------------------------------------
cODEWorld* ODEWorld;

//����
cODEGenericBody* ODEBody;

//��
cODEGenericBody* ODEWall[6];

//GEL���E---------------------------------------------------
cGELWorld *GELWorld;

cGELMesh *gelhand;


// �V���~���[�V�������[�v�̃X�e�[�^�X
bool simulationRunning = false;
// ���\�[�X�̃p�X
string resourceRoot;
// �������[�h ON/OFF
bool useDamping = false;
// �͏ꃂ�[�h ON/OFF
bool useForceField = true;
// has exited haptics simulation thread
bool simulationFinished = false;

//---------------------------------------------------------------------------
// �}�N����`
//---------------------------------------------------------------------------
// ���\�[�X�p�X�̎擾
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// �֐�
//---------------------------------------------------------------------------

//�f�B�X�v���C�̃T�C�Y���ς�����ƌĂяo�����
void resizeWindow(int w, int h);
void resizeWindow3D(int w, int h);

//�L�[�{�[�h�ɑ΂���R�[���o�b�N
void keySelect(unsigned char key, int x, int y);

// �}�E�X�{�^�����E�N���b�N����ă��j���[��I�����ꂽ���̃R�[���o�b�N
void menuSelect(int value);

//�A�v���P�[�V�����I�����ɌĂяo�����
void close(void);

//�O���t�B�b�N�̃R�[���o�b�N
void updateGraphics(void);
void updateGraphics3D(void);//���̎���

// �f�o�C�X�̃R�[���o�b�N
void updateHaptics(void);

void TimerDisplay(int t);

//�`�揈��
void Draw(void);

//�������
void createCube(cMesh* a_mesh, double a_size);

void BuildDynamicModel(cGELMesh*);

int main(int argc, char* argv[])
{

    //���\�[�X�̃f�B���N�g���擾
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // 3D - ���E�̐ݒ�
    //-----------------------------------------------------------------------

    world = new cWorld();

    //�w�i�F
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // �J�����쐬
    camera = new cCamera(world);
    world->addChild(camera);

    // �J�����̈ʒu
	camera->set( cVector3d (3.3, 0.0, 0.0),    // �J�����ʒu (eye)
                 cVector3d (0.0, 0.0, 0.0),    // �œ_ (target)
                 cVector3d (0.0, 0.0, 1.0));   // "up" vector

	//�����_�����O����
    camera->setClippingPlanes(0.01, 10.0);

    // ���ߐݒ�
    camera->enableMultipassTransparency(true);    
	
	//���C�g�̍쐬
    light = new cLight(world);
    camera->addChild(light);                   // ���C�g��ǉ�
    light->setEnabled(true);
    light->setPos(cVector3d( 2.0, 0.5, 1.0));
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  //���C�g�̕���
	light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);

    //-----------------------------------------------------------------------
    // HAPTIC�f�o�C�X
    //-----------------------------------------------------------------------

    handler = new cHapticDeviceHandler();

	//�P�Ԃ̃f�o�C�X�ɐڑ�
    cGenericHapticDevice* hapticDevice;
    handler->getDevice(hapticDevice, 0);

	//�J�����g�f�o�C�X�̏����擾
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }
	
	// 3D�|�C���^���쐬���ǉ�
    tool = new cGeneric3dofPointer(world);
    world->addChild(tool);

	//tool���f�o�C�X�ɐڑ�
    tool->setHapticDevice(hapticDevice);

	//�ڑ���������
    tool->start();

	// �f�o�C�X�̕������[�N�X�y�[�X��傫�ȉ��z���[�N�X�y�[�X�ɃZ�b�g����
    tool->setWorkspaceRadius(1.3);

	// �|�C���^�̋��̑傫��(���������)
    tool->setRadius(0.05);

	//�f�o�C�X�̋����B���Aproxy�̂ݕ\��
    tool->m_deviceSphere->setShowEnabled(false);

    // set the physical radius of the proxy to be equal to the radius
    // of the tip of the mesh drill (see drill in the virtual scene section)
    proxyRadius = 0.05;
    tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
    tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;


    // the environmeny is static, you can set this parameter to "false"
    tool->m_proxyPointForceModel->m_useDynamicProxy = true;
    tool->m_proxyPointForceModel->m_useForceShading = true;

	// �{�^�����������Ɨc�����̍ގ����ꏏ�ɂ���
    tool->m_materialProxy = tool->m_materialProxyButtonPressed;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    //�d�x��ݒ�
    double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;


    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_ColorPointA.set(1.0, 1.0, 1.0);
    graspLine->m_ColorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);
	//-----------------------------------------------------------------------
    // ���z�I�ȃV�[�����\��
    //-----------------------------------------------------------------------
	//----------------------------------------------------------------------
	//�����G���W��
	//----------------------------------------------------------------------
	ODEWorld = new cODEWorld(world);

	world->addChild(ODEWorld);

	//�d�͂𐶐�
	ODEWorld->setGravity(cVector3d(-9.81,0.0,0.0));

	//���̂��쐬
	ODEBody = new cODEGenericBody(ODEWorld);
	cMesh* object = new cMesh(world);

	double boxsize = 1.5;

	createCube(object,boxsize);

	//�ގ��ݒ�
	cMaterial mat0;
    mat0.m_ambient.set(0.8, 0.1, 0.4);
    mat0.m_diffuse.set(1.0, 0.15, 0.5);
    mat0.m_specular.set(1.0, 0.2, 0.8);
    mat0.setStiffness(0.5 * stiffnessMax);
    mat0.setDynamicFriction(0.8);
    mat0.setStaticFriction(0.8);
    object->setMaterial(mat0);

	//���̂𕨗��G���W���ɓo�^
	ODEBody->setImageModel(object);
	ODEBody->createDynamicBox(boxsize,boxsize,boxsize);

	//���̂̐ݒ�
	ODEBody->setMass(0.05);

	//���̂̈ʒu�A��]
	ODEBody->setPosition(cVector3d(0.0,0,0));

	//object->setPos(1.0,1.0,1.0);

	//�ǂ𐶐�
	for(int i = 0;i < 6;i++)
	{
		ODEWall[i] = new cODEGenericBody(ODEWorld);
	}
	double size = 2.0;
    ODEWall[0]->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEWall[1]->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
    ODEWall[2]->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEWall[3]->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEWall[4]->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEWall[5]->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));

	//���˂̐ݒ�------------------------------------------------------------------
	cGenericObject* reflexion = new cGenericObject();
	reflexion->setAsGhost(true);
	world->addChild(reflexion);

	object->setUseCulling(false,true);

	// create a symmetry rotation matrix (z-plane)
    cMatrix3d rotRefexion;
    rotRefexion.set(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0);
    reflexion->setRot(rotRefexion);
    reflexion->setPos(-2.005, 0.0, 0.0);

    // add objects to the world
    reflexion->addChild(ODEWorld);
    reflexion->addChild(tool);


	//�n�ʂ��쐬------------------------------------------------------------------
    // ���b�V���쐬
    cMesh* ground = new cMesh(world);
    world->addChild(ground);

    // �n�ʂ̒��_
    double groundSize = 1.5;
	double b_depth = 1.5;
	double b_height = 1.5;
	double b_weight = 1.5;
	int vertices0 = ground->newVertex( -groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex( -groundSize, groundSize, 0.0);
    int vertices2 = ground->newVertex( -groundSize, -groundSize, groundSize);
    int vertices3 = ground->newVertex(-groundSize,  groundSize, groundSize);

	// ���_��ʂ�
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices2, vertices1, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    //�ʒu
    ground->setPos(0.0, 0.0, -0.5);

    // ���b�V���̍ގ�
    cMaterial matGround;
    matGround.setStiffness(stiffnessMax);//����
    matGround.setDynamicFriction(0.7);//�����C
    matGround.setStaticFriction(1.0);//�Ö��C
    matGround.m_ambient.set(0.1, 0.1, 0.1);
    matGround.m_diffuse.set(0.0, 0.0, 0.0);
    matGround.m_specular.set(0.0, 0.0, 0.0);
    ground->setMaterial(matGround);

    // ����
    ground->setTransparencyLevel(0.8);
    ground->setUseTransparency(true);


	//�ǂ��쐬---------------------------------------------------------------------
	// ���b�V���쐬
    cMesh* wall[4];
	for(int i = 0;i < 4;i++)
	{
		wall[i] = new cMesh(world);
		world->addChild(wall[i]);
	}
	
	// �ǂ̍ގ�
    cMaterial matWall;
    matWall.setStiffness(stiffnessMax);//����
    matWall.setDynamicFriction(0.8);//�����C��
    matWall.setStaticFriction(1.0);//�Ö��C��
    matWall.m_ambient.set(0.1, 0.1, 0.1);
    matWall.m_diffuse.set(0.9, 0.9, 0.9);
    matWall.m_specular.set(0.0, 0.0, 0.0);
    
	// �ǂ̐ݒ�
    vertices0 = wall[0]->newVertex(-b_depth, -groundSize, 0.0);
    vertices1 = wall[0]->newVertex( b_depth, -groundSize, 0.0);
    vertices2 = wall[0]->newVertex( b_depth, -groundSize, groundSize);
    vertices3 = wall[0]->newVertex(-b_depth, -groundSize, groundSize);
    wall[0]->newTriangle(vertices2, vertices1, vertices0);
    wall[0]->newTriangle(vertices3, vertices2, vertices0);
	wall[0]->setPos(0.0, 0.0, -0.5);

    vertices0 = wall[1]->newVertex(-b_depth, -groundSize, 0.0);
    vertices1 = wall[1]->newVertex( b_depth, -groundSize, 0.0);
    vertices2 = wall[1]->newVertex( b_depth,  groundSize, 0.0);
    vertices3 = wall[1]->newVertex(-b_depth,  groundSize, 0.0);
    wall[1]->newTriangle(vertices0, vertices1, vertices2);
    wall[1]->newTriangle(vertices0, vertices2, vertices3);
	wall[1]->setPos(0.0, 0.0, -0.5);

	vertices0 = wall[2]->newVertex( -b_depth, groundSize, 0.0);
    vertices1 = wall[2]->newVertex( -b_depth, groundSize, groundSize);
    vertices2 = wall[2]->newVertex( b_depth, groundSize, 0.0);
    vertices3 = wall[2]->newVertex( b_depth,  groundSize, groundSize);
    wall[2]->newTriangle(vertices2, vertices1, vertices0);
    wall[2]->newTriangle(vertices1, vertices2, vertices3);
	wall[2]->setPos(0.0, 0.0, -0.5);

	
	vertices0 = wall[3]->newVertex( b_depth, -groundSize, groundSize);
    vertices1 = wall[3]->newVertex( b_depth, groundSize, groundSize);
    vertices2 = wall[3]->newVertex( -b_depth, -groundSize, groundSize);
    vertices3 = wall[3]->newVertex( -b_depth,  groundSize, groundSize);
    wall[3]->newTriangle(vertices0, vertices2, vertices1);
    wall[3]->newTriangle(vertices2, vertices3, vertices1);
	wall[3]->setPos(0.0, 0.0, -0.5);
	

	for(int i = 0;i < 4;i++)
	{
		// compute surface normals
		wall[i]->computeAllNormals();
		// �ʒu
		wall[i]->setMaterial(matWall);
		// ����
		wall[i]->setTransparencyLevel(0.8);
		wall[i]->setUseTransparency(true);
	}


	//-----------------------------------------------------------------------------------
    //�V�������b�V���쐬
    tooth = new cMesh(world);

    //���b�V�������[���h�ɓo�^
    world->addChild(tooth);

    //���b�V���̈ʒu�Ɖ�]���Z�b�g
    tooth->setPos(0.0, 0.0, 0.0);
    tooth->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(-10));
    tooth->rotate(cVector3d(0.0, 1.0, 0.0), cDegToRad(10));

    //���b�V���̃I�u�W�F�N�g�����[�h
    bool fileload = tooth->loadFromFile(RESOURCE_PATH("resources/models/tooth/tooth.3ds"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = tooth->loadFromFile("../../../bin/resources/models/tooth/tooth.3ds");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.1\n");
        close();
        return (-1);
    }
	
	// �O�����C���t���[���ŕ`��
    ((cMesh*)(tooth->getChild(1)))->setWireMode(true);

    // �������ݒ�
    ((cMesh*)(tooth->getChild(1)))->setUseTransparency(false);
    ((cMesh*)(tooth->getChild(1)))->setTransparencyLevel(transparencyLevel);

    // compute a boundary box
    tooth->computeBoundaryBox(true);

    // ���T�C�Y
    tooth->scale(0.003);

    //�Փˌ��o�A���S���Y�����v�Z
    tooth->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

    //�I�u�W�F�N�g�̃f�t�H���g�̍������`
    tooth->setStiffness(20 * stiffnessMax, true);


	//����쐬--------------------------------------------------------------------------
	GELWorld = new cGELWorld();
	world->addChild(GELWorld);

	cGELSkeletonNode::default_kDampingPos = 0.1;//���`����
	cGELSkeletonNode::default_kDampingRot = 0.1;//��]�^����
	GELWorld->m_integrationTime = 1.0/400.0;//�V���~���[�V�������ݎ���

	gelhand = new cGELMesh(world);
	GELWorld->m_gelMeshes.push_front(gelhand);//m_gelMeshes:�ό`�\�Ȍő̂̃��X�g

	fileload = gelhand->loadFromFile(RESOURCE_PATH("resources/penhandComp.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = gelhand->loadFromFile("../../../bin/resources/penhandComp.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.2\n");
        close();
        return (-1);
    }

	
	gelhand->computeBoundaryBox(true);
	cVector3d min = gelhand->getBoundaryMin();
	cVector3d max = gelhand->getBoundaryMax();

	cVector3d span = cSub(max,min);
	double si = cMax(span.x,cMax(span.y,span.z));

	if(si > 0)
	{
		gelhand->scale(1.0/si);
	}

	gelhand->computeBoundaryBox(true);
	
	
	// setup default values for nodes
    cGELSkeletonNode::default_radius        = 0.04;//3;�m�[�h���a
    cGELSkeletonNode::default_kDampingPos   = 0.4;
    cGELSkeletonNode::default_kDampingRot   = 0.1;
    cGELSkeletonNode::default_mass          = 0.06;  // [kg]
    cGELSkeletonNode::default_showFrame     = true;
    cGELSkeletonNode::default_color.set(0.6, 0.6, 0.0);
    cGELSkeletonNode::default_useGravity      = false;
    cGELSkeletonNode::default_gravity.set(0.00, 0.00, 0.00);
    double radius = cGELSkeletonNode::default_radius;


    cGELSkeletonLink::default_kSpringElongation = 100.0; // [N/m]�@�L���o�l�萔
    cGELSkeletonLink::default_kSpringFlexion    = 0.1;   // [Nm/RAD] �p���x
    cGELSkeletonLink::default_kSpringTorsion    = 0.1;   // [Nm/RAD] �萔�˂���΂�
    cGELSkeletonLink::default_color.set(0.2, 0.2, 1.0);

    // build dynamic vertices
    gelhand->buildVertices();
    gelhand->m_useSkeletonModel = true;

	// create dynamic model (GEM)
    BuildDynamicModel(gelhand);


	gelhand->connectVerticesToSkeleton(true);

	gelhand->m_showSkeletonModel = false;

	GELWorld->m_integrationTime = 0.001;

	// create anchors
    //cGELSkeletonLink::default_kSpringElongation = 5.0; // [N/m]
    list<cGELSkeletonNode*>::iterator i;
    int num = 0;
    for(i = gelhand->m_nodes.begin(); i != gelhand->m_nodes.end(); ++i)
    {
        num++;
    }

    int counter1 = 0;
    int counter2 = 0;
    for(i = gelhand->m_nodes.begin(); i != gelhand->m_nodes.end(); ++i)
    {
        if (counter1 <= num)
        {
            if (counter2 > 3)
            {
                cGELSkeletonNode* nextItem = *i;
                cGELSkeletonNode* newNode = new cGELSkeletonNode();
                newNode->m_fixed = true;
                newNode->m_pos = nextItem->m_pos;
                cGELSkeletonLink* newLink = new cGELSkeletonLink(nextItem, newNode); gelhand->m_links.push_front(newLink);
                newLink->m_kSpringElongation = 5;
                counter2 = 0;
            }
            counter2 ++;
            counter1++;
        }
    }

	gelhand->deleteCollisionDetector(true);//�����蔻��Ȃ�

	gelhand->computeAllNormals(true);

	gelhand->setShowEnabled(true,true);

	

	
	
	
    //-----------------------------------------------------------------------
    // OpenGL�ݒ�
    //-----------------------------------------------------------------------


	
	glutInit(&argc ,argv);

	//GLUT�̃E�B���h�E�ݒ�
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	printf("�ڂ̊Ԋu:%f\n�œ_����:%f\n",camera->getStereoEyeSeparation(),camera->getStereoFocalLength());
	printf("�J��������:%f\n",camera->getFieldViewAngle());
	int mes = MessageBox( NULL, "���̎��\�����s���܂����H(Quadro���K�v)", "nividia3D", MB_YESNO | MB_ICONQUESTION | MB_DEFBUTTON2 | MB_TASKMODAL );
	if(mes == IDYES)
	{
		//���̎�
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);
		//glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		//�Q�[�����[�h�ɂ��ă��t���b�V�����[�g120Hz�ɂ���
		glutGameModeString("1680x1050:32@120");
		glutEnterGameMode();

		glutDisplayFunc(updateGraphics3D);
		glutKeyboardFunc(keySelect);
		glutReshapeFunc(resizeWindow3D);
		glEnable(GL_DEPTH_TEST);
		camera->setStereoFocalLength(depth);
		camera->setStereoEyeSeparation(dist);//�ڂ̋���
		HWND hWnd = FindWindow("GLUT","GLUT");
		viewport = new cViewport(hWnd,camera,true,0);
	}else{
		//�ʏ�
		glutInitWindowPosition(windowPosX, windowPosY);
		glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		glutCreateWindow(argv[0]);
		glutDisplayFunc(updateGraphics);
		glutKeyboardFunc(keySelect);
		glutReshapeFunc(resizeWindow);
		glutSetWindowTitle("CHAI 3D");
	}


		// �E�N���b�N���j���[�o�^
		glutCreateMenu(menuSelect);
		glutAddMenuEntry("�t���X�N���[��", OPTION_FULLSCREEN);
		glutAddMenuEntry("�E�B���h�E���[�h", OPTION_WINDOWDISPLAY);
		glutAttachMenu(GLUT_RIGHT_BUTTON);
    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    //�V���~���[�V�����J�n
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    //�����_�����O���[�v�J�n
    glutMainLoop();


    //��n��
	if(mes == IDYES)
	{
		glutLeaveGameMode();
	}
    close();

    return (0);
}

//CALLBACK---�E�B���h�E�̃T�C�Y���ύX���ꂽ��
void resizeWindow(int w, int h)
{
    // update the size of the viewport
	displayW = w;
    displayH = h;
    glViewport(0, 0, w, h);
}

void resizeWindow3D(int w, int h)
{
    // update the size of the viewport
	displayW = w;
    displayH = h;
    //glViewport(0, 0, w, h);
	RECT rect;
	rect.bottom = h;
	rect.top = 0;
	rect.right = w;
	rect.left = 0;
	viewport->setRenderArea(rect);
}


//�L�[�������ꂽ��
void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }
}

//�E�N���b�N���j���[���I�����ꂽ��
void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
            break;
    }
}




//---------------------------------------------------------------------------

//�`��ݒ�
void updateGraphics(void)
{

	camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // �G���[�`�F�b�N
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

void updateGraphics3D(void)
{

	//���p�o�b�t�@�̎w��
	glDrawBuffer(GL_BACK_LEFT);
	camera->renderView(displayW, displayH,CHAI_STEREO_LEFT);

	//�E�ڗp�o�b�t�@�̎w��
	glDrawBuffer(GL_BACK_RIGHT);
	camera->renderView(displayW,displayH,CHAI_STEREO_RIGHT);

	glutSwapBuffers();
	
	//�G���[�`�F�b�N
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// ������
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------
//�q�̒��_
vector<cVertex> *child_vertex;

//���ݔ���
bool graspActive = false;

// �͂�ł���ODE�I�u�W�F�N�g
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;


void updateHaptics(void)
{
    //�ėp�ϊ����W
	cVector3d vx, vy, vz;
	cMatrix3d tmp_matrix;
	
	cPrecisionClock simClock;
    simClock.start(true);
	

    while(simulationRunning)
    {
        //�O���[�o�����W
		world->computeGlobalPositions(true);

		
		// �ʒu���X�V and orientation of tool
        tool->updatePose();

        //�C���^���N�V�����̗͂��v�Z
        tool->computeInteractionForces();

        // �͊o���f�o�C�X�ɑ���
        tool->applyForces();

        //��̐ݒ�-------------------------------------------------
		cVector3d pos = tool->m_proxyPointForceModel->getProxyGlobalPosition();
        cMatrix3d rot = tool->m_deviceGlobalRot;

		cMatrix3d rot90;
		//rot90.setCol0(cVector3d(1,0,0));
		//rot90.setCol1(cVector3d(0,cos(3.14/2.0),-sin(3.14/2.0)));
		//rot90.setCol2(cVector3d(0,sin(3.14/2.0),cos(3.14/2.0)));
		rot90.setCol0(cVector3d(cos(3.14/2.0),0,-sin(3.14/2.0)));
		rot90.setCol1(cVector3d(0,1,0));
		rot90.setCol2(cVector3d(sin(3.14/2.0),0,cos(3.14/2.0)));
		
		//��̍��W�ݒ�
		gelhand->setPos(cMul(rot90,tool->m_deviceLocalPos));
		gelhand->setRot(cMul(rot,rot90));
		//---------------------------------------------------------
		
		//�c�[�����I�u�W�F�N�g�ɐG��Ă��邩�`�F�b�N
        cGenericObject* object = tool->m_proxyPointForceModel->m_contactPoint0->m_object;

		//�{�^����������Ă���
        int button = tool->getUserSwitch(0);

        if (graspActive && button)
        {
            // retrieve latest position and orientation of grasped ODE object in world coordinates
            cMatrix3d globalGraspObjectRot = graspObject->getGlobalRot();
            cVector3d globalGraspObjectPos = graspObject->getGlobalPos();

			// �ʒu���v�Z���� of the grasp point on object in global coordinates
            cVector3d globalGraspPos = globalGraspObjectPos + cMul(globalGraspObjectRot, graspPos);

			// retrieve the position of the tool in global coordinates
            cVector3d globalToolPos  = tool->getProxyGlobalPos();

            // compute the offset between the tool and grasp point on the object
            cVector3d offset = globalToolPos - globalGraspPos;

            // model a spring between both points
            double STIFFNESS = 4;
            cVector3d force = STIFFNESS * offset;

            // apply attraction force (grasp) onto object
            graspObject->addGlobalForceAtGlobalPos(force, globalGraspPos);
            // scale magnitude and apply opposite force to haptic device
            tool->m_lastComputedGlobalForce.add(cMul(-1.0, force));

            // update both end points of the line which is used for display purposes only
            graspLine->m_pointA = globalGraspPos;
            graspLine->m_pointB = globalToolPos;
			

			lastPosDevice = pos;
            lastRotDevice = rot;
            lastPosObject = tooth->getPos();
            lastRotObject = tooth->getRot();
            lastDeviceObjectPos = cTrans(lastRotDevice) * ((lastPosObject - lastPosDevice) + 0.01*cNormalize(lastPosObject - lastPosDevice));
            lastDeviceObjectRot = cMul(cTrans(lastRotDevice), lastRotObject);
            tooth->setHapticEnabled(true, true);
            tool->setShowEnabled(true, true);
            gelhand->setShowEnabled(true, true);
        }
        else
		{
            tool->setShowEnabled(true, true);
            cMatrix3d rotDevice01 = cMul(cTrans(lastRotDevice), rot);
            cMatrix3d newRot =  cMul(rot, lastDeviceObjectRot);
            cVector3d newPos = cAdd(pos, cMul(rot, lastDeviceObjectPos));
            tooth->setPos(newPos);
            tooth->setRot(newRot);
            world->computeGlobalPositions(true);
            tooth->setHapticEnabled(false, true);

			// was the user grasping the object at the previous simulation loop
            if (graspActive)
            {
                // we disable grasping
                graspActive = false;

                graspLine->setShowEnabled(false);

                // we enable haptics interaction between the tool and the previously grasped object
                if (graspObject != NULL)
                {
                    graspObject->m_imageModel->setHapticEnabled(true, true);
                }
            }

		    // ���[�U���I�u�W�F�N�g�ɐG��Ă���
			if (object != NULL)
			{
				// check if object is attached to an external ODE parent
				cGenericType* externalParent = object->getExternalParent();
				cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
				if (ODEobject != NULL)
				{
				    // �c�[���̈ʒu�擾
				    cVector3d pos = tool->m_proxyPointForceModel->m_contactPoint0->m_globalPos;
	
				    // �X�C�b�`�������Ă���Ƃ�����
				    if (button)
					{
						//�V�����I�u�W�F�N�g���͂܂��
						graspObject = ODEobject;

						// retrieve the grasp position on the object in local coordinates
						graspPos    = tool->m_proxyPointForceModel->m_contactPoint0->m_localPos;

						// grasp in now active!
						graspActive = true;

						// enable small line which display the offset between the tool and the grasp point
						graspLine->setShowEnabled(true);

					    // disable haptic interaction between the tool and the grasped device.
					    // this is performed for stability reasons.
					    graspObject->m_imageModel->setHapticEnabled(false, true);
					}

					// retrieve the haptic interaction force being applied to the tool
					cVector3d force = tool->m_lastComputedGlobalForce;

					// apply haptic force to ODE object
					cVector3d tmpfrc = cNegate(force);
					ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
				}
			}
        }

		 //�͂𑗂�
        tool->applyForces();

        // �V���~���[�V�����^�C���Z�b�g
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.001);

        //�N���b�N���Z�b�g
        simClock.reset();
        simClock.start();

        // �V���~���[�V�����A�b�v�f�[�g
        ODEWorld->updateDynamics(nextSimInterval);
    }

    //�V���~���[�V�����I��
    simulationFinished = true;
}


//���̃��b�V�������
void createCube(cMesh* a_mesh, double a_size)
{
    const double HALFSIZE = a_size / 2.0;
    int vertices [6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE,  HALFSIZE);
    vertices[0][3] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE,  HALFSIZE);

    // face +x
    vertices[1][0] = a_mesh->newVertex( HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = a_mesh->newVertex( HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex( HALFSIZE,  HALFSIZE,  HALFSIZE);
    vertices[1][3] = a_mesh->newVertex( HALFSIZE, -HALFSIZE,  HALFSIZE);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][1] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][2] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[2][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // face +y
    vertices[3][0] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[3][3] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[4][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][2] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][3] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);

    // face +z
    vertices[5][0] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[5][1] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // create triangles
    for (int i=0; i<6; i++)
    {
    a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
    a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    //�ގ��ݒ�
    a_mesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
}


//�I������
void close(void)
{
    // �V���~���[�V�����I��
    simulationRunning = false;
    // �������I������܂őҋ@
    while (!simulationFinished) { cSleepMs(100); }
    // �f�o�C�X�����
    tool->stop();
}

//---------------------------------------------------------------------------
void BuildDynamicModel(cGELMesh *defObject)
{
    cGELSkeletonNode* newNode;
    cGELSkeletonNode* prevNode;
    cGELSkeletonLink* newLink;

    //-----------------------------------
    newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.296,0.280,-0.213);
    //-----------------------------------
    prevNode = newNode; newNode = new cGELSkeletonNode(); defObject->m_nodes.push_front(newNode);
    newNode->m_pos.set(0.286,0.213,-0.254);
    newLink = new cGELSkeletonLink(prevNode, newNode); defObject->m_links.push_front(newLink);
}
