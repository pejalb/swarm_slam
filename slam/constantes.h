#ifndef CONSTANTES_
#define CONSTANTES_

#ifndef NULL
#define NULL (0)
#endif

#ifndef M_PI
#define M_PI (3.14159265358979323846)             
#endif

#ifndef M_2PI
#define M_2PI (6.28318530717958647692)
#endif // !M_2PI



#ifdef GRIDMAP_
#define TAM_MAX_NOME_ARQ (80) //tamanho maximo para o nome de um arquivo de mapa
#define TAM_MAX_NUM (10) //numero maximo de digitos que podem suceder o numero de arquivo
#endif


#ifdef TESTE_SLAM_BASE_
#define A_270_GRAUS (4.712388980384690)
#define A_180_GRAUS M_PI
#define A_135_GRAUS (2.356194490192345)
#define A_90_GRAUS (1.570796326794897)
#endif



#ifdef PSO_
#define SEED_PADRAO (0)
#endif

#ifdef SCAN_MATCH_
#define DX 10.0
#define DY 10.0
#define D_ANG 0.5*M_PI
#define ERRO_CELULA_MAPA_INICIAL 0.1
#define ERRO_MAXIMO 100

#endif

#ifdef SLAM_
//constantes inerentes ao problema

#if TESTE_CSV_ == 1//caso seja um teste com arquivo csv de leituras 
#define ESCALA (0.1)
#define ESPACO_ANG A_180_GRAUS
#define MEIO_ESPACO_ANG A_90_GRAUS
#define LEITURAS_POR_SCAN (361)
//numero minimo de poses cujo armazenamento e garantido
#define NUM_MINIMO_POSES (700)
#define SCANS_DE_TESTE (0)
#define MAX_ALCANCE 15.0/ESCALA
#else //caso se trate de um ensaio rodando onboard no p3dx
#define ESCALA 100.0
#define ESPACO_ANG A_270_GRAUS
#define MEIO_ESPACO_ANG A_135_GRAUS
#define SCANS_DE_TESTE (1000)
#define NUM_MAX_SCANS (700)
#define LEITURAS_POR_SCAN (541) //para aria 541 leituras por scan
//numero minimo de poses cujo armazenamento e garantido
#define NUM_MINIMO_POSES (600)
#define MAX_ALCANCE 20000/ESCALA
#endif // TESTE_CSV==1

#if TESTE_MOBILE_SIM_ == 1
#define ESCALA 1000
#define ESPACO_ANG A_180_GRAUS
#define MEIO_ESPACO_ANG A_90_GRAUS
#define LEITURAS_POR_SCAN (181)
#define NUM_MINIMO_POSES (700)
#define MAX_ALCANCE 20000/ESCALA
#endif
  
#endif


#endif
