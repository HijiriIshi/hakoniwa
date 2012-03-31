

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
// 設定
//---------------------------------------------------------------------------

// ウィンドウサイズ
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// 右クリックメニュー
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

#define _WSIZEX 1680
#define _WSIZEY 1050
#define _FPS 60

double depth =15.00000;
double dist =0.50;

//---------------------------------------------------------------------------
// 広域変数
//---------------------------------------------------------------------------

//ワールド
cWorld* world;

// カメラ
cCamera* camera;

cViewport* viewport;

// ライト
cLight *light;

// カレントウィンドウディスプレイのWight Height
int displayW  = 0;
int displayH  = 0;

// デバイスのハンドル
cHapticDeviceHandler* handler;

//デバイスの情報
cHapticDeviceInfo info;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

//モデルのメッシュ
cMesh* tooth;

//透過レベル
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


//物理演算世界----------------------------------------------
cODEWorld* ODEWorld;

//物体
cODEGenericBody* ODEBody;

//壁
cODEGenericBody* ODEWall[6];

//GEL世界---------------------------------------------------
cGELWorld *GELWorld;

cGELMesh *gelhand;


// シュミレーションループのステータス
bool simulationRunning = false;
// リソースのパス
string resourceRoot;
// 減衰モード ON/OFF
bool useDamping = false;
// 力場モード ON/OFF
bool useForceField = true;
// has exited haptics simulation thread
bool simulationFinished = false;

//---------------------------------------------------------------------------
// マクロ定義
//---------------------------------------------------------------------------
// リソースパスの取得
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// 関数
//---------------------------------------------------------------------------

//ディスプレイのサイズが変えられると呼び出される
void resizeWindow(int w, int h);
void resizeWindow3D(int w, int h);

//キーボードに対するコールバック
void keySelect(unsigned char key, int x, int y);

// マウスボタンが右クリックされてメニューを選択された時のコールバック
void menuSelect(int value);

//アプリケーション終了時に呼び出される
void close(void);

//グラフィックのコールバック
void updateGraphics(void);
void updateGraphics3D(void);//立体視版

// デバイスのコールバック
void updateHaptics(void);

void TimerDisplay(int t);

//描画処理
void Draw(void);

//箱を作る
void createCube(cMesh* a_mesh, double a_size);

void BuildDynamicModel(cGELMesh*);

int main(int argc, char* argv[])
{

    //リソースのディレクトリ取得
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // 3D - 世界の設定
    //-----------------------------------------------------------------------

    world = new cWorld();

    //背景色
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // カメラ作成
    camera = new cCamera(world);
    world->addChild(camera);

    // カメラの位置
	camera->set( cVector3d (3.3, 0.0, 0.0),    // カメラ位置 (eye)
                 cVector3d (0.0, 0.0, 0.0),    // 焦点 (target)
                 cVector3d (0.0, 0.0, 1.0));   // "up" vector

	//レンダリング距離
    camera->setClippingPlanes(0.01, 10.0);

    // 透過設定
    camera->enableMultipassTransparency(true);    
	
	//ライトの作成
    light = new cLight(world);
    camera->addChild(light);                   // ライトを追加
    light->setEnabled(true);
    light->setPos(cVector3d( 2.0, 0.5, 1.0));
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  //ライトの方向
	light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);

    //-----------------------------------------------------------------------
    // HAPTICデバイス
    //-----------------------------------------------------------------------

    handler = new cHapticDeviceHandler();

	//１番のデバイスに接続
    cGenericHapticDevice* hapticDevice;
    handler->getDevice(hapticDevice, 0);

	//カレントデバイスの情報を取得
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }
	
	// 3Dポインタを作成し追加
    tool = new cGeneric3dofPointer(world);
    world->addChild(tool);

	//toolをデバイスに接続
    tool->setHapticDevice(hapticDevice);

	//接続を初期化
    tool->start();

	// デバイスの物理ワークスペースを大きな仮想ワークスペースにセットする
    tool->setWorkspaceRadius(1.3);

	// ポインタの球の大きさ(見えるもの)
    tool->setRadius(0.05);

	//デバイスの球を隠し、proxyのみ表示
    tool->m_deviceSphere->setShowEnabled(false);

    // set the physical radius of the proxy to be equal to the radius
    // of the tip of the mesh drill (see drill in the virtual scene section)
    proxyRadius = 0.05;
    tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
    tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;


    // the environmeny is static, you can set this parameter to "false"
    tool->m_proxyPointForceModel->m_useDynamicProxy = true;
    tool->m_proxyPointForceModel->m_useForceShading = true;

	// ボタン押した時と幼い時の材質を一緒にする
    tool->m_materialProxy = tool->m_materialProxyButtonPressed;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    //硬度を設定
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
    // 仮想的なシーンを構成
    //-----------------------------------------------------------------------
	//----------------------------------------------------------------------
	//物理エンジン
	//----------------------------------------------------------------------
	ODEWorld = new cODEWorld(world);

	world->addChild(ODEWorld);

	//重力を生成
	ODEWorld->setGravity(cVector3d(-9.81,0.0,0.0));

	//物体を作成
	ODEBody = new cODEGenericBody(ODEWorld);
	cMesh* object = new cMesh(world);

	double boxsize = 1.5;

	createCube(object,boxsize);

	//材質設定
	cMaterial mat0;
    mat0.m_ambient.set(0.8, 0.1, 0.4);
    mat0.m_diffuse.set(1.0, 0.15, 0.5);
    mat0.m_specular.set(1.0, 0.2, 0.8);
    mat0.setStiffness(0.5 * stiffnessMax);
    mat0.setDynamicFriction(0.8);
    mat0.setStaticFriction(0.8);
    object->setMaterial(mat0);

	//物体を物理エンジンに登録
	ODEBody->setImageModel(object);
	ODEBody->createDynamicBox(boxsize,boxsize,boxsize);

	//物体の設定
	ODEBody->setMass(0.05);

	//物体の位置、回転
	ODEBody->setPosition(cVector3d(0.0,0,0));

	//object->setPos(1.0,1.0,1.0);

	//壁を生成
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

	//反射の設定------------------------------------------------------------------
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


	//地面を作成------------------------------------------------------------------
    // メッシュ作成
    cMesh* ground = new cMesh(world);
    world->addChild(ground);

    // 地面の頂点
    double groundSize = 1.5;
	double b_depth = 1.5;
	double b_height = 1.5;
	double b_weight = 1.5;
	int vertices0 = ground->newVertex( -groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex( -groundSize, groundSize, 0.0);
    int vertices2 = ground->newVertex( -groundSize, -groundSize, groundSize);
    int vertices3 = ground->newVertex(-groundSize,  groundSize, groundSize);

	// 頂点を面に
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices2, vertices1, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    //位置
    ground->setPos(0.0, 0.0, -0.5);

    // メッシュの材質
    cMaterial matGround;
    matGround.setStiffness(stiffnessMax);//剛性
    matGround.setDynamicFriction(0.7);//動摩擦
    matGround.setStaticFriction(1.0);//静摩擦
    matGround.m_ambient.set(0.1, 0.1, 0.1);
    matGround.m_diffuse.set(0.0, 0.0, 0.0);
    matGround.m_specular.set(0.0, 0.0, 0.0);
    ground->setMaterial(matGround);

    // 透過
    ground->setTransparencyLevel(0.8);
    ground->setUseTransparency(true);


	//壁を作成---------------------------------------------------------------------
	// メッシュ作成
    cMesh* wall[4];
	for(int i = 0;i < 4;i++)
	{
		wall[i] = new cMesh(world);
		world->addChild(wall[i]);
	}
	
	// 壁の材質
    cMaterial matWall;
    matWall.setStiffness(stiffnessMax);//剛性
    matWall.setDynamicFriction(0.8);//動摩擦力
    matWall.setStaticFriction(1.0);//静摩擦力
    matWall.m_ambient.set(0.1, 0.1, 0.1);
    matWall.m_diffuse.set(0.9, 0.9, 0.9);
    matWall.m_specular.set(0.0, 0.0, 0.0);
    
	// 壁の設定
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
		// 位置
		wall[i]->setMaterial(matWall);
		// 透過
		wall[i]->setTransparencyLevel(0.8);
		wall[i]->setUseTransparency(true);
	}


	//-----------------------------------------------------------------------------------
    //新しいメッシュ作成
    tooth = new cMesh(world);

    //メッシュをワールドに登録
    world->addChild(tooth);

    //メッシュの位置と回転をセット
    tooth->setPos(0.0, 0.0, 0.0);
    tooth->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(-10));
    tooth->rotate(cVector3d(0.0, 1.0, 0.0), cDegToRad(10));

    //メッシュのオブジェクトをロード
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
	
	// 外をワイヤフレームで描画
    ((cMesh*)(tooth->getChild(1)))->setWireMode(true);

    // 半透明設定
    ((cMesh*)(tooth->getChild(1)))->setUseTransparency(false);
    ((cMesh*)(tooth->getChild(1)))->setTransparencyLevel(transparencyLevel);

    // compute a boundary box
    tooth->computeBoundaryBox(true);

    // リサイズ
    tooth->scale(0.003);

    //衝突検出アルゴリズムを計算
    tooth->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

    //オブジェクトのデフォルトの剛性を定義
    tooth->setStiffness(20 * stiffnessMax, true);


	//手を作成--------------------------------------------------------------------------
	GELWorld = new cGELWorld();
	world->addChild(GELWorld);

	cGELSkeletonNode::default_kDampingPos = 0.1;//線形衰退
	cGELSkeletonNode::default_kDampingRot = 0.1;//回転型衰退
	GELWorld->m_integrationTime = 1.0/400.0;//シュミレーション現在時刻

	gelhand = new cGELMesh(world);
	GELWorld->m_gelMeshes.push_front(gelhand);//m_gelMeshes:変形可能な固体のリスト

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
    cGELSkeletonNode::default_radius        = 0.04;//3;ノード半径
    cGELSkeletonNode::default_kDampingPos   = 0.4;
    cGELSkeletonNode::default_kDampingRot   = 0.1;
    cGELSkeletonNode::default_mass          = 0.06;  // [kg]
    cGELSkeletonNode::default_showFrame     = true;
    cGELSkeletonNode::default_color.set(0.6, 0.6, 0.0);
    cGELSkeletonNode::default_useGravity      = false;
    cGELSkeletonNode::default_gravity.set(0.00, 0.00, 0.00);
    double radius = cGELSkeletonNode::default_radius;


    cGELSkeletonLink::default_kSpringElongation = 100.0; // [N/m]　伸長バネ定数
    cGELSkeletonLink::default_kSpringFlexion    = 0.1;   // [Nm/RAD] 角速度
    cGELSkeletonLink::default_kSpringTorsion    = 0.1;   // [Nm/RAD] 定数ねじりばね
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

	gelhand->deleteCollisionDetector(true);//当たり判定なし

	gelhand->computeAllNormals(true);

	gelhand->setShowEnabled(true,true);

	

	
	
	
    //-----------------------------------------------------------------------
    // OpenGL設定
    //-----------------------------------------------------------------------


	
	glutInit(&argc ,argv);

	//GLUTのウィンドウ設定
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	printf("目の間隔:%f\n焦点距離:%f\n",camera->getStereoEyeSeparation(),camera->getStereoFocalLength());
	printf("カメラ視野:%f\n",camera->getFieldViewAngle());
	int mes = MessageBox( NULL, "立体視表示を行いますか？(Quadroが必要)", "nividia3D", MB_YESNO | MB_ICONQUESTION | MB_DEFBUTTON2 | MB_TASKMODAL );
	if(mes == IDYES)
	{
		//立体視
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);
		//glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		//ゲームモードにしてリフレッシュレート120Hzにする
		glutGameModeString("1680x1050:32@120");
		glutEnterGameMode();

		glutDisplayFunc(updateGraphics3D);
		glutKeyboardFunc(keySelect);
		glutReshapeFunc(resizeWindow3D);
		glEnable(GL_DEPTH_TEST);
		camera->setStereoFocalLength(depth);
		camera->setStereoEyeSeparation(dist);//目の距離
		HWND hWnd = FindWindow("GLUT","GLUT");
		viewport = new cViewport(hWnd,camera,true,0);
	}else{
		//通常
		glutInitWindowPosition(windowPosX, windowPosY);
		glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		glutCreateWindow(argv[0]);
		glutDisplayFunc(updateGraphics);
		glutKeyboardFunc(keySelect);
		glutReshapeFunc(resizeWindow);
		glutSetWindowTitle("CHAI 3D");
	}


		// 右クリックメニュー登録
		glutCreateMenu(menuSelect);
		glutAddMenuEntry("フルスクリーン", OPTION_FULLSCREEN);
		glutAddMenuEntry("ウィンドウモード", OPTION_WINDOWDISPLAY);
		glutAttachMenu(GLUT_RIGHT_BUTTON);
    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    //シュミレーション開始
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    //レンダリングループ開始
    glutMainLoop();


    //後始末
	if(mes == IDYES)
	{
		glutLeaveGameMode();
	}
    close();

    return (0);
}

//CALLBACK---ウィンドウのサイズが変更された時
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


//キーが押された時
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

//右クリックメニューが選択された時
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

//描画設定
void updateGraphics(void)
{

	camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // エラーチェック
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

	//左用バッファの指定
	glDrawBuffer(GL_BACK_LEFT);
	camera->renderView(displayW, displayH,CHAI_STEREO_LEFT);

	//右目用バッファの指定
	glDrawBuffer(GL_BACK_RIGHT);
	camera->renderView(displayW,displayH,CHAI_STEREO_RIGHT);

	glutSwapBuffers();
	
	//エラーチェック
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// 続ける
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------
//子の頂点
vector<cVertex> *child_vertex;

//つかみ判定
bool graspActive = false;

// 掴んでいるODEオブジェクト
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;


void updateHaptics(void)
{
    //汎用変換座標
	cVector3d vx, vy, vz;
	cMatrix3d tmp_matrix;
	
	cPrecisionClock simClock;
    simClock.start(true);
	

    while(simulationRunning)
    {
        //グローバル座標
		world->computeGlobalPositions(true);

		
		// 位置を更新 and orientation of tool
        tool->updatePose();

        //インタラクションの力を計算
        tool->computeInteractionForces();

        // 力覚をデバイスに送る
        tool->applyForces();

        //手の設定-------------------------------------------------
		cVector3d pos = tool->m_proxyPointForceModel->getProxyGlobalPosition();
        cMatrix3d rot = tool->m_deviceGlobalRot;

		cMatrix3d rot90;
		//rot90.setCol0(cVector3d(1,0,0));
		//rot90.setCol1(cVector3d(0,cos(3.14/2.0),-sin(3.14/2.0)));
		//rot90.setCol2(cVector3d(0,sin(3.14/2.0),cos(3.14/2.0)));
		rot90.setCol0(cVector3d(cos(3.14/2.0),0,-sin(3.14/2.0)));
		rot90.setCol1(cVector3d(0,1,0));
		rot90.setCol2(cVector3d(sin(3.14/2.0),0,cos(3.14/2.0)));
		
		//手の座標設定
		gelhand->setPos(cMul(rot90,tool->m_deviceLocalPos));
		gelhand->setRot(cMul(rot,rot90));
		//---------------------------------------------------------
		
		//ツールがオブジェクトに触れているかチェック
        cGenericObject* object = tool->m_proxyPointForceModel->m_contactPoint0->m_object;

		//ボタンが押されている
        int button = tool->getUserSwitch(0);

        if (graspActive && button)
        {
            // retrieve latest position and orientation of grasped ODE object in world coordinates
            cMatrix3d globalGraspObjectRot = graspObject->getGlobalRot();
            cVector3d globalGraspObjectPos = graspObject->getGlobalPos();

			// 位置を計算する of the grasp point on object in global coordinates
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

		    // ユーザがオブジェクトに触れている
			if (object != NULL)
			{
				// check if object is attached to an external ODE parent
				cGenericType* externalParent = object->getExternalParent();
				cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
				if (ODEobject != NULL)
				{
				    // ツールの位置取得
				    cVector3d pos = tool->m_proxyPointForceModel->m_contactPoint0->m_globalPos;
	
				    // スイッチを押しているときつかむ
				    if (button)
					{
						//新しいオブジェクトが掴まれる
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

		 //力を送る
        tool->applyForces();

        // シュミレーションタイムセット
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.001);

        //クロックリセット
        simClock.reset();
        simClock.start();

        // シュミレーションアップデート
        ODEWorld->updateDynamics(nextSimInterval);
    }

    //シュミレーション終了
    simulationFinished = true;
}


//箱のメッシュを作る
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

    //材質設定
    a_mesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
}


//終了処理
void close(void)
{
    // シュミレーション終了
    simulationRunning = false;
    // 処理が終了するまで待機
    while (!simulationFinished) { cSleepMs(100); }
    // デバイスを閉じる
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
