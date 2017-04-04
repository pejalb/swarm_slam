
#include "Aria.h"
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <ctime>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cmath>
#include "slam.h"
#define TESTE_CSV_ 0
#include "constantes.h"
#include <thread>
#include <functional>

int main(int argc, char **argv)
{
    using namespace std::placeholders;
    Aria::init();
    //ArLog::init(ArLog::StdOut, ArLog::Verbose);
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot robot;
    ArRobotConnector robotConnector(&parser, &robot);
    ArAnalogGyro gyro(&robot);
    std::vector<ponto > leituras;
    leituras.reserve(LEITURAS_POR_SCAN);
    //const double ESCALA = 500.0;
    const unsigned int numLinhas = 1000;
    const unsigned int numColunas = 1000;
    const double fatorGrauRad = M_PI/180.0;
    
    slam s(numLinhas, numColunas, ESCALA);
	int conta;

	if (!robotConnector.connectRobot())
    {
      if (!parser.checkHelpAndWarnUnparsed())
      {
        ArLog::log(ArLog::Terse, "Nao pode se conectar ao robo!");
      }
      else
      {
        ArLog::log(ArLog::Terse, "Erro, nao pode se conectar ao robo");
        Aria::logOptions();
        Aria::exit(1);
      }
    }
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
    ArCompassConnector compassConnector(&parser);
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
      Aria::logOptions();
      exit(1);
    }
    ArSonarDevice sonarDev;
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    printf("ESC para sair\n");
    robot.addRangeDevice(&sonarDev);

    robot.runAsync(true);
    if (!laserConnector.connectLasers(false, false, true))
    {
      printf("Nao conseguiu se conectar ao Laser do robo\n");
      Aria::exit(2);
    }

    ArTCM2 *compass = compassConnector.create(&robot);
    if(compass && !compass->blockingConnect()) {
      compass = NULL;
    }

    robot.lock();
    ArLog::log(ArLog::Normal, "principal: Conectado!");

    ArLog::log(ArLog::Normal, "principal: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
	//odometria.push_back(Eigen::Vector3d(robot.getX(), robot.getY(), robot.getTh()));//informacao odometrica
    robot.unlock();

    ArLog::log(ArLog::Normal, "principal: aguardando 3 segundos...");
    ArUtil::sleep(3000);

    //som indica pronto
    char buff[4];
    buff[0] = 25; buff[1] = 60;
    buff[2] = 25; buff[3] = 60;
    robot.comStrN(ArCommands::SAY,buff,4);

    //para text
    time_t rawtime;
    struct tm * timeinfo;
    char hora[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(hora,80,"%d_%m_%Y %I_%M_%S",timeinfo);
    std::string nome = std::string("dump") + hora;    
    const char* cNome = nome.c_str();
    std::ofstream dumpLaser;/*Stream de caracteres para armazenar os valores lidos pelo Laser*/
    nome = std::string("dumpPoses") + hora;
    dumpLaser.open(cNome, std::ios::out | std::ios::app);//abre stream
    dumpLaser << "Inicio leitura (formato: numLeitura541, angulo, distanciaLaser, X laser, Y laser, X roboGlobal, Y roboGlobal)\n"; 
    std::ofstream dumpPoses;
    dumpPoses.open(nome.c_str(), std::ios::out | std::ios::app);
    ArLog::log(ArLog::Normal, "Iniciando Laser Scan");
    std::list<ArPoseWithTime*>::iterator it;
    std::list<ArSensorReading*>::const_iterator itR;
    ArLog::log(ArLog::Normal, "Inicio movimento");/*O movimento só começa de fato quando o robo for destravado e houver uma velocidade não nula aplicada a cada roda*/
	robot.setCycleTime(500);//pode nao compilar
    ArModeTeleop teleop(&robot, "teleop", 't', 'T');/*Modo de teleoperação, tal qual utilizado pelo demo*/
    // activate the default mode

    teleop.activate();
    // turn on the motors
    robot.comInt(ArCommands::ENABLE, 1);/*Permite controlar diretamente por meio de um dispositivo como um joystick ou teclado*/
	/*FINAL DA INICIALIZAÇÃO*/
    while(robot.isConnected())
    {	
	robot.lock();
	robot.enableMotors();
	//robot.setVel(100.0);
	robot.unlock();/*Cuidado!!! Apartir desse ponto o movimento pode ocorrer de modo ABRUPTO!!!!*/
	int numLasers = 0;
	robot.lock();
	/*Cria um STL map com todos os lasers presentes no robô (instalados e RECONHECIDOS)*/
	std::map<int, ArLaser*> *lasers = robot.getLaserMap();/*Note que o uso do mapa permite agrupar a chave e o ponteiro para */
	
	for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)/*STL iterator para varrer os lasers em função das chaves que os identificam*/
	{
	    int laserIndex = (*i).first;/*Identificador numerico*/
	    ArLaser* laser = (*i).second;
	    if(!laser)
	        continue;
	    ++numLasers;
	    laser->lockDevice();/*Trava para a leitura*/
	    int contador = 0;
	    std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer(); //X,Y e atributos
	    const std::list<ArSensorReading*> *rawReadings = laser->getRawReadings();
		/*Leituras não processadas são lidas abaixo*/
		conta = 0;
		//double odoX = (*itR)->getXTaken()/ESCALA;
		//double odoY = (*itR)->getYTaken()/ESCALA;
		//double odoTH = (*itR)->getThTaken();
		
		dumpLaser << "\n";
	    for (itR = rawReadings->begin(); itR != rawReadings->end(); ++itR)/*Varre as leituras em sequência da direita para a esquerda*/
	    {
			ArPose posRobo = (*itR)->getPose();/*Pose fornecida pela biblioteca Aria*/
			(*itR)->getAdjusted();
	        dumpLaser << (*itR)->getRange() <<",";
            contador++;
	    
            leituras.push_back(ponto((*itR)->getLocalX()/ESCALA, (*itR)->getLocalY()/ESCALA));
	    }
	    dumpPoses << posRobo.getX()<<","<<posRobo.getY()<<","<<(*itR)->getThRad() <<std::endl;
        s.atualiza(leituras,true,(*itR)->getXTaken()/ESCALA,(*itR)->getYTaken()/ESCALA,(*itR)->getThTaken()*fatorGrauRad);
	//dumpPoses << (*itR)->getXTaken()/ESCALA,(*itR)->getYTaken()/ESCALA,(*itR)->getThTaken()*fatorGrauRad <<std::endl;
	
        leituras.clear();
        laser->unlockDevice();	/*destrava o laser até a próxima tentativa de leitura*/
	}	
	salvaWrapper(&s,"mapa");
	if(numLasers == 0)/*Não foi possível conectar ao laser*/
		ArLog::log(ArLog::Normal, "Nenhum laser detectado.");
	else
			//ArLog::log(ArLog::Normal, "");

			//0,1 segundos para prox leitura
		robot.unlock();	
  		ArUtil::sleep(1500);/*Intervalo entre as leituras*/
		
    }
    dumpLaser.close();/*Fecha o stream de leitura*/
    dumpPoses.close();
	/*...........................................................................................................................................*/

	/*...........................................................................................................................................*/
    ArLog::log(ArLog::Normal, "principal, saindo da thread do robo...");
    robot.stopRunning();
    robot.waitForRunExit();
    ArLog::log(ArLog::Normal, "principal, saindo...");
	/*...........................................................................................................................................*/
	/*debugging*/
	getchar();
	/*...........................................................................................................................................*/
    return 0;
}
