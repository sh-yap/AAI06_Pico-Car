#ifndef MOTION_TEST_TKERNEL_H
#define MOTION_TEST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int INT;
typedef int ID;
typedef int ER;
typedef int BOOL;
#define TRUE 1
#define FALSE 0
typedef unsigned int UINT;
typedef uint32_t UW;
typedef uint32_t RELTIM;
typedef unsigned char UB;
typedef struct { UINT mbfatr; INT bufsz; INT maxmsz; void *bufptr; } T_CMBF;
typedef struct { UINT mtxatr; } T_CMTX;
typedef struct { UINT flgatr; } T_CFLG;
typedef struct { UINT tskatr; void (*task)(INT, void *); INT itskpri;
                 INT stksz; void *bufptr; UINT assprc; } T_CTSK;
#define E_OK 0
#define EXPORT
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0U
#define TA_USERBUF 0U
#define TA_INHERIT 0U
#define TA_WMUL 0U
#define TA_HLNG 0U
#define TA_RNG3 0U
#define TA_ASSPRC 0U
#define TP_PRC1 1U
#define TWF_ORW 0U
#define TWF_CLR 0U
ER tk_loc_mtx(ID id, INT timeout);
ER tk_unl_mtx(ID id);
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *pattern, INT timeout);
ER tk_set_flg(ID id, UINT bits);
INT tk_rcv_mbf(ID id, void *data, INT timeout);
ER tk_snd_mbf(ID id, const void *data, INT size, INT timeout);
ID tk_cre_mbf(const T_CMBF *p);
ID tk_cre_mtx(const T_CMTX *p);
ID tk_cre_flg(const T_CFLG *p);
ID tk_cre_tsk(const T_CTSK *p);
ER tk_sta_tsk(ID id, INT code);
ER tk_dly_tsk(RELTIM ms);
INT tk_get_prc(void);
#endif
