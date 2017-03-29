#ifndef CONSTANTES_
#define CONSTANTES_

#ifndef NULL
#define NULL (0)
#endif

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

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
#define DX 2.0
#define DY 2.0
#define D_ANG M_PI
#endif

#ifdef SLAM_
//constantes inerentes ao problema

#if TESTE_CSV_ //caso seja um teste com arquivo csv de leituras 
#define ESCALA (2.0)
#define ESPACO_ANG A_270_GRAUS
#define MEIO_ESPACO_ANG A_135_GRAUS
#define LEITURAS_POR_SCAN (361)
//numero minimo de poses cujo armazenamento e garantido
#define NUM_MINIMO_POSES (700)
#define SCANS_DE_TESTE (0)

#else //caso se trate de um ensaio rodando onboard no p3dx
#define ESCALA 1000.0
#define ESPACO_ANG A_180_GRAUS
#define MEIO_ESPACO_ANG A_90_GRAUS
#define SCANS_DE_TESTE (1000)
#define NUM_MAX_SCANS (700)
#define LEITURAS_POR_SCAN (541) //para aria 541 leituras por scan
//numero minimo de poses cujo armazenamento e garantido
#define NUM_MINIMO_POSES (600)
#endif // TESTE_CSV==1

#endif


#endif
